"""Mission XML -> Promela. A behaviour tree, compiled to a model.

This turns a behaviour tree into a Promela model and the set of atomic
propositions it establishes. It used to be the model half of a verification
pair -- checked against a temporal formula generated from the mission text,
with SPIN deciding whether the model satisfied the formula -- but that check
has been removed. What's left is used directly: ``action_pool`` names, from
the XSD's ``ActionGroup``, what the compiler knows how to represent, and
``test_ontology.py`` checks the ontology table against it to catch schema
drift; ``test_task_synthesis.py`` compiles a synthesized task to check it is
structurally sound.

**Atomic propositions are named by content.** ``MoveToTreeID id="10"
approach_tree="true"`` defines ``at_tree_10``; the ``SampleLeaf`` that follows
it defines ``sampled_tree_10``. The names are a function of the mission's
subject matter -- which trees, which actions.

The alternative, binding the *i*-th proposition to the *i*-th action by
position, does not survive a replan: drop one task and every proposition after
it silently rebinds to the wrong action.

**The trace.** Each action leaf becomes one ``d_step`` -- one indivisible
transition, so each leaf is exactly one observable state and no intermediate
state exists where the step has advanced but its parameters have not. A plan is
therefore a straight line: ``amiga_btcpp.xsd`` has every condition node
commented out, so a conforming tree has nothing to branch on. ``Fallback`` is
emitted as a nondeterministic choice anyway, because the XSD does allow it and
a silently ignored branch is worse than a modelled one.

**Achievement latches, position does not.** ``sampled_tree_10`` stays true once
true, so ``<>(sampled_tree_1 && <>sampled_tree_2)`` means what its author
expects. ``at_tree_10`` holds only while the robot is there, because a
proposition claiming the robot is in two places at once is not useful.

No ROS, no LLM, no I/O. lxml and the XSD, so this is testable against the real
files in ``amiga_ros2_behavior_tree/examples/``.
"""

import re
from dataclasses import dataclass, field
from typing import Dict, List, Set

from lxml import etree

from amiga_ros2_agents.mission import ontology

#: Control nodes the emitter walks. Matches the XSD's ControlFlowGroup; kept as
#: a literal rather than parsed out of the schema because each one needs its own
#: emission rule below, so a new tag here is never a no-op.
SEQUENCE_TAGS = frozenset({"Sequence", "ReactiveSequence"})
CHOICE_TAGS = frozenset({"Fallback"})
DECORATOR_TAGS = frozenset({"RetryUntilSuccessful", "AlwaysSuccess", "Inverter"})

#: Identifier for "no action in progress" -- the initial and final step value.
IDLE = "idle"


class PromelaError(ValueError):
    """The tree cannot be modelled. Carries a reason fit for a rejection."""


@dataclass
class Model:
    """A Promela model of one behaviour tree, plus what it can talk about.

    ``defined_aps`` is what this plan establishes -- read by
    ``test_ontology.py`` and ``test_task_synthesis.py`` to check a plan
    actually does what it claims to.
    """

    source: str
    defined_aps: Set[str] = field(default_factory=set)
    #: Objective tree IDs, in the order the plan visits them. Diagnostics only.
    visit_order: List[str] = field(default_factory=list)


def action_pool(xsd_path: str) -> List[str]:
    """The action vocabulary, read out of the XSD's ActionGroup.

    Derived rather than hand-maintained on purpose. The upstream planner keeps
    the equivalent list as a literal in a separate file, and it has drifted out
    of sync with the schema -- a tree of valid actions compiles there to an
    empty model, which verifies vacuously. A list that cannot drift cannot do
    that.
    """
    doc = etree.parse(xsd_path)
    ns = {"xs": "http://www.w3.org/2001/XMLSchema"}
    group = doc.find(".//xs:group[@name='ActionGroup']", namespaces=ns)
    if group is None:
        raise PromelaError(f"no ActionGroup in {xsd_path}")
    names = [el.get("name") for el in group.findall(".//xs:element", namespaces=ns)]
    names = [n for n in names if n]
    if not names:
        raise PromelaError(f"ActionGroup in {xsd_path} declares no elements")
    return names


def _ident(text: str) -> str:
    """Reduce arbitrary text to a Promela identifier."""
    return re.sub(r"[^A-Za-z0-9_]", "_", text).strip("_").lower()


class _Emitter:
    """Walks one tree, accumulating declarations, statements and macros.

    Split from ``compile_mission`` so the recursion can carry state without
    threading five accumulators through every call.
    """

    def __init__(self, actions: List[str], orchard=None):
        self.actions = actions
        self.orchard = orchard
        self.stmts: List[str] = []
        self.macros: Dict[str, str] = {}
        self.latches: Set[str] = set()
        self.visit_order: List[str] = []
        #: What the walk knows is true here -- where the robot is, which aisle
        #: it is in, what it has already done. This is what lets a SampleLeaf
        #: know which tree it samples: the leaf itself carries no location,
        #: which is exactly why the ontology refuses an orphaned one.
        self._state = ontology.State()

    # -- macros ---------------------------------------------------------

    def _macro(self, name: str, body: str) -> None:
        self.macros.setdefault(name, body)

    def _latch(self, name: str) -> None:
        self.latches.add(name)
        self._macro(name, name + "_f")

    # -- walk -----------------------------------------------------------

    def walk(self, node, indent: str = "    ") -> None:
        for child in node:
            if not isinstance(child.tag, str):
                continue  # comment / processing instruction
            tag = child.tag
            if tag in SEQUENCE_TAGS:
                self.walk(child, indent)
            elif tag in CHOICE_TAGS:
                self._emit_choice(child, indent)
            elif tag in DECORATOR_TAGS:
                # A decorator changes when or how often its child runs, not what
                # the child establishes. For a co-safe property over a finite
                # trace those are the same trace, so it is transparent here.
                self.walk(child, indent)
            elif tag in self.actions:
                self._emit_action(child, indent)
            else:
                raise PromelaError(f"unmodellable element <{tag}>")

    def _emit_choice(self, node, indent: str) -> None:
        """Fallback -> nondeterministic if/fi, one branch per child."""
        branches = [c for c in node if isinstance(c.tag, str)]
        if not branches:
            return
        self.stmts.append(f"{indent}if")
        before = self._state
        ends: List[ontology.State] = []
        for branch in branches:
            self.stmts.append(f"{indent}:: true ->")
            self._state = before
            # Wrap the branch so a bare action still forms a valid option body.
            holder = etree.Element("Sequence")
            holder.append(_copy(branch))
            self.walk(holder, indent + "    ")
            ends.append(self._state)
        self.stmts.append(f"{indent}fi;")
        # Only what every branch agrees on survives the choice.
        merged = ends[0]
        for state in ends[1:]:
            merged = merged.merge(state)
        self._state = merged

    def _emit_action(self, node, indent: str) -> None:
        """One action leaf -> one d_step, and one entry in the vocabulary.

        Which proposition it defines is the ontology's call, not this module's.
        Deciding it here from the tag was how the emitter and the arbiter came
        to hold two copies of the same rule, and the copies disagreed: a plan
        that approaches a tree in one ``RetryUntilSuccessful`` and samples it in
        the next modelled fine here and was rejected there.
        """
        tag = node.tag
        assigns = [f"step = {tag}"]

        step, self._state = ontology.advance(self._state, node, self.orchard)
        if step.violation:
            # Refusing to model it is better than modelling a sample of nowhere.
            raise PromelaError(step.violation)

        where = next((f for f in step.establishes if f.kind in ontology.WHERE), None)
        if where is not None and where.kind in (ontology.AT_TREE, ontology.AT_WAYPOINT):
            assigns.append(f"tree = {where.arg}")

        if step.proposition == ontology.OBJECTIVE:
            # Positional, not latching: a proposition claiming the robot is in
            # two places at once is not useful.
            ap = f"{ontology.AT_TREE}_{where.arg}"
            self._macro(ap, f"(step == {tag} && tree == {where.arg})")
            self.visit_order.append(where.arg)
        elif step.proposition == ontology.ACHIEVEMENT:
            fact = step.establishes[0]
            ap = f"{fact.kind}_{fact.arg}"
            self._latch(ap)
            assigns.append(f"{ap}_f = true")
        elif step.proposition == ontology.DONE:
            # The action advances the trace and can be referred to by an
            # <action>_done proposition, but carries no location.
            ap = f"{_ident(tag)}_done"
            self._latch(ap)
            assigns.append(f"{ap}_f = true")
        # SILENT: transit. It moves the robot without being somewhere the
        # mission asked for, so it defines nothing -- see _objective_tree_ids.

        self.stmts.append(f"{indent}d_step {{ {'; '.join(assigns)} }}")


def _copy(node):
    """Deep copy without dragging the parent's tail text along."""
    clone = etree.fromstring(etree.tostring(node))
    clone.tail = None
    return clone


def compile_mission(mission_xml: str, xsd_path: str, orchard=None) -> Model:
    """Compile a mission's behaviour tree into a Promela model.

    Raises ``PromelaError`` with a reason a planner can act on. Returning a
    model that says nothing would be worse: it would verify.

    ``orchard`` is optional and changes nothing about the propositions -- it
    only lets the ontology bind ``in_aisle`` facts to real aisle numbers, which
    matters to the closure and to synthesis rather than to the model.
    """
    try:
        doc = etree.fromstring(mission_xml.encode("utf-8"))
    except etree.XMLSyntaxError as exc:
        raise PromelaError(f"not well-formed XML: {exc}") from exc

    trees = doc.findall("BehaviorTree")
    if not trees:
        raise PromelaError("no <BehaviorTree> in mission")
    if len(trees) > 1:
        # The XSD permits several; nothing in this system emits them, and
        # guessing which one runs would be inventing a semantics.
        raise PromelaError(f"{len(trees)} <BehaviorTree> elements, expected 1")

    actions = action_pool(xsd_path)
    emitter = _Emitter(actions, orchard)
    emitter.walk(trees[0])

    if not emitter.stmts:
        raise PromelaError("behaviour tree contains no actions")

    return Model(
        source=_render(actions, emitter),
        defined_aps=set(emitter.macros),
        visit_order=list(emitter.visit_order),
    )


def _render(actions: List[str], em: "_Emitter") -> str:
    lines: List[str] = []
    lines.append(f"mtype = {{ {IDLE}, " + ", ".join(actions) + " };")
    lines.append("")
    lines.append(f"mtype step = {IDLE};")
    lines.append("int tree = 0;")
    for latch in sorted(em.latches):
        lines.append(f"bool {latch}_f = false;")
    lines.append("")
    for name in sorted(em.macros):
        lines.append(f"#define {name} {em.macros[name]}")
    lines.append("")
    lines.append("init {")
    lines.extend(em.stmts)
    # Park the trace in a state where no action is in progress. SPIN extends a
    # terminated run by stuttering this final state, which is what gives a
    # finite plan a well-defined infinite word for the never claim to read.
    lines.append(f"    step = {IDLE};")
    lines.append("}")
    return "\n".join(lines) + "\n"
