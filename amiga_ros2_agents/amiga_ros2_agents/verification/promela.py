"""Mission XML -> Promela. The model half of the verification pair.

``ltl_gen`` turns the mission's English into a temporal formula; this turns the
behaviour tree into a model SPIN can check that formula against. The two never
see each other's output -- the LTL agent is given the ``<Mission>`` text and
nothing else -- so agreement between them is evidence, not construction. That
decoupling is the whole point, and it is easy to destroy by accident: derive the
atomic propositions from the tree and the tree satisfies them by definition.

**Atomic propositions are named by content.** ``MoveToTreeID id="10"
approach_tree="true"`` defines ``at_tree_10``; the ``SampleLeaf`` that follows
it defines ``sampled_tree_10``. The names are a function of the mission's
subject matter -- which trees, which actions -- and the same convention is
written into ``prompts/ltl_gen/system.j2``, so the two sides land on the same
identifier without ever consulting each other.

The alternative, binding the *i*-th proposition to the *i*-th action by
position, does not survive a replan: drop one task and every proposition after
it silently rebinds to the wrong action. Since replanning is the only reason
this module exists, position is not available to us.

**The trace.** Each action leaf becomes one ``d_step`` -- one indivisible
transition, so each leaf is exactly one observable state and no intermediate
state exists where the step has advanced but its parameters have not. A plan is
therefore a straight line, and SPIN is being used as an LTL-over-finite-trace
checker rather than a state-space explorer. That is not a limitation we chose:
``amiga_btcpp.xsd`` has every condition node commented out, so a conforming tree
has nothing to branch on. ``Fallback`` is emitted as a nondeterministic choice
anyway, because the XSD does allow it and a silently ignored branch is worse
than a modelled one.

**Achievement latches, position does not.** ``sampled_tree_10`` stays true once
true, so ``<>(sampled_tree_1 && <>sampled_tree_2)`` means what its author
expects. ``at_tree_10`` holds only while the robot is there, because a
proposition claiming the robot is in two places at once is not useful.

No ROS, no LLM, no I/O. lxml and the XSD, so this is testable against the real
files in ``amiga_ros2_behavior_tree/examples/``.
"""

import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from lxml import etree

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

    ``defined_aps`` is the contract with the formula: any proposition the LTL
    agent used that is not in here is not something this plan establishes, and
    SPIN would reject the combination as an undeclared variable rather than as
    a property violation. Callers check coverage first so the failure arrives
    as "the plan never samples tree 35" instead of a parser error.
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

    def __init__(self, actions: List[str]):
        self.actions = actions
        self.stmts: List[str] = []
        self.macros: Dict[str, str] = {}
        self.latches: Set[str] = set()
        self.visit_order: List[str] = []
        #: Tree the robot is currently standing at, as the walk sees it. This is
        #: what lets a SampleLeaf know which tree it samples -- the leaf itself
        #: carries no location, which is exactly why the arbiter refuses an
        #: orphaned one.
        self._at: Optional[str] = None

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
        at_before = self._at
        ends: List[Optional[str]] = []
        for branch in branches:
            self.stmts.append(f"{indent}:: true ->")
            self._at = at_before
            # Wrap the branch so a bare action still forms a valid option body.
            holder = etree.Element("Sequence")
            holder.append(_copy(branch))
            self.walk(holder, indent + "    ")
            ends.append(self._at)
        self.stmts.append(f"{indent}fi;")
        # Position after the choice is only known if every branch agrees.
        self._at = ends[0] if len(set(ends)) == 1 else None

    def _emit_action(self, node, indent: str) -> None:
        tag = node.tag
        assigns = [f"step = {tag}"]

        if tag == "MoveToTreeID":
            tree_id = node.get("id")
            if tree_id is None:
                raise PromelaError("<MoveToTreeID> without id")
            assigns.append(f"tree = {tree_id}")
            self._at = tree_id
            if node.get("approach_tree", "").lower() == "true":
                ap = f"at_tree_{tree_id}"
                self._macro(ap, f"(step == MoveToTreeID && tree == {tree_id})")
                self.visit_order.append(tree_id)
        elif tag == "SampleLeaf":
            if self._at is None:
                # The arbiter's orphan check should have caught this first; if
                # it did not, refusing to model it is better than modelling a
                # sample of nowhere.
                raise PromelaError("<SampleLeaf> with no preceding MoveToTreeID")
            ap = f"sampled_tree_{self._at}"
            self._latch(ap)
            assigns.append(f"{ap}_f = true")
        else:
            # Every other action still advances the trace and can be referred to
            # by an <action>_done proposition, but carries no location.
            ap = f"{_ident(tag)}_done"
            self._latch(ap)
            assigns.append(f"{ap}_f = true")

        self.stmts.append(f"{indent}d_step {{ {'; '.join(assigns)} }}")


def _copy(node):
    """Deep copy without dragging the parent's tail text along."""
    clone = etree.fromstring(etree.tostring(node))
    clone.tail = None
    return clone


def compile_mission(mission_xml: str, xsd_path: str) -> Model:
    """Compile a mission's behaviour tree into a Promela model.

    Raises ``PromelaError`` with a reason a planner can act on. Returning a
    model that says nothing would be worse: it would verify.
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
    emitter = _Emitter(actions)
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


def formula_aps(formula: str) -> Set[str]:
    """The atomic propositions a formula refers to.

    Every bare identifier that is not an operator. Deliberately syntactic: the
    point is to catch a proposition the plan never defines *before* SPIN sees
    it, because SPIN reports that as an undeclared variable at parse time --
    indistinguishable, in the output, from a broken model.
    """
    reserved = {"true", "false", "U", "V", "W", "X", "R"}
    return {
        tok
        for tok in re.findall(r"[A-Za-z_][A-Za-z0-9_]*", formula)
        if tok not in reserved
    }


def coverage_gap(formula: str, model: Model) -> Tuple[bool, str]:
    """Does the plan define everything the formula talks about?

    Returns (ok, reason). A gap means the two agents disagree about the mission:
    either the plan omits something the mission asked for, or the LTL agent
    invented a fact. Both are rejections, and naming the proposition is what
    makes the rejection actionable by the planner.
    """
    missing = formula_aps(formula) - model.defined_aps
    if not missing:
        return True, ""
    known = ", ".join(sorted(model.defined_aps)) or "none"
    return False, (
        f"plan does not establish {sorted(missing)}; "
        f"propositions this plan defines: {known}"
    )
