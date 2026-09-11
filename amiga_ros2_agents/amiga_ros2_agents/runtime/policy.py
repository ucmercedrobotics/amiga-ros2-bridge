"""
policy.py

The seam this whole package can be split on: everywhere a node would otherwise
call a language model, it asks this module first whether to call one at all.

``amiga_ros2_coordinator/ports/reasoning.py`` draws the same line one layer up,
for the three judgements the *coordinator* cannot make without reading
unstructured evidence. This module is the agents-side mirror of it, and exists
for the same reason: a system whose interesting behaviour lives entirely
inside a model call is a system nobody can run an ablation against. Swap every
call for something with no model in it, hold everything else fixed, and the
difference between the two runs is what the model was actually worth.

That only works if "something with no model in it" is a real policy and not a
rigged one. A function that special-cased the scenarios this fleet is tested
on would not be measuring what a general-purpose fallback buys a robot; it
would be measuring how well this module's author remembered the demo script.
So every function below is a closed-form rule over *structured* state --
typed fields, the mission ontology's facts and preconditions, the XSD action
grammar, the peer registry -- and none of the first five opens the fault
text, a ``/rosout`` line, the VLM's description of what the camera sees, or
the mission's free-text summary. Where a decision genuinely depends on one of
those, the honest answer is not a heuristic dressed up to look principled --
it is the narrowest safe action, documented here as a finding rather than
apologised for. ``interpret_anomaly`` never synthesising new work, and
``interpret_note`` never doing anything but ``keep``, are exactly that: the
result of asking "what can a rule over structured fields alone actually do
here", not gaps this module ran out of time to close.

The sixth, ``permanence``, is the one deliberate exception, and is documented
as one where it is defined rather than smoothed over here: the question it
answers -- will this failure ever succeed if retried -- has no structured
proxy anywhere in this package, so the choice was never "read the fault text
or find a rule that doesn't", it was "read it honestly in both arms, or
pretend one of them isn't doing what it has always done."

Selection is two env vars, both read once at import for the same reason
``runtime/llm.py`` reads its own env vars at import: model *selection* has to
work with nothing installed and nothing running, so a test that never touches
a model must not pay for deciding whether it should.

    AGENT_POLICY      "llm" (default) or "deterministic"
    AGENT_PERMANENCE  "keywords" (default) or "llm"

The second of those exists only because of the sixth seam below. It does not
choose an arm the way ``AGENT_POLICY`` does -- see ``permanence``'s own
section for why permanence never had an arm to choose between in the first
place, and why folding its opt-in into ``AGENT_POLICY`` would have made the
default do something no configuration of this fleet has ever done.

Every function here is pure: explicit arguments in, a value in the same
closed schema the language-model branch would have produced out. No ROS, no
node state, no I/O, no global mutable state. The one thing that looks like
state -- how many times a node has already been routed -- is not kept here;
it is threaded in by the caller (``triage_node`` already keeps ``self.routes``
for its own caching, and that dict is the argument, not a copy of it living in
this module too). A policy module that accumulated its own history would be a
second place a mission's routing state could disagree with the node's own
record of it.

One more thing is threaded in rather than imported, and for a boundary reason
rather than a state one: ``replan`` decides over the mission ontology and the
task grammar, both of which live in ``mission``, and this file lives in
``runtime`` -- the layer ``runtime/__init__.py`` says knows nothing about
missions, and ``test_layering.py`` holds every module in this package to it.
Rather than carve an exception into a rule that exists to catch exactly this
kind of creeping dependency, ``replan`` takes ``mission_tasks`` and
``ontology`` as arguments, the way it already takes ``pruned_xml``. Its
caller -- ``mission_planner_node`` -- already imports both for the surrounding
pipeline, so nothing new is loaded; only where the import *lives* changes.
"""

import json
import os
from typing import Dict, Optional, Tuple

from lxml import etree

from amiga_ros2_comms.codec import CAPABILITY_BY_ELEMENT, ReasonCode

from . import llm

# ---------------------------------------------------------------------------
# Selection
# ---------------------------------------------------------------------------

_POLICIES = frozenset({"llm", "deterministic"})

#: Read once, at import, exactly like ``llm.MODEL`` and friends. Anything
#: other than the two names below is almost certainly a typo in a launch file
#: or a research-harness script, and the failure mode for a typo silently
#: falling back to "llm" is a whole ablation arm quietly re-running the
#: baseline it meant to compare against -- a wrong number in a paper, found
#: (if it is found at all) long after the run that produced it is gone. Fail
#: at import instead, while the mistake is still one line in one log.
ACTIVE = os.environ.get("AGENT_POLICY", "llm").strip().lower()
if ACTIVE not in _POLICIES:
    raise RuntimeError(f"AGENT_POLICY={ACTIVE!r} is not one of {sorted(_POLICIES)}")


def deterministic() -> bool:
    """Whether the deterministic arm is active. Checked at every call site."""
    return ACTIVE == "deterministic"


def label() -> str:
    """What produced a decision, for anything that logs or reports a model.

    ``llm.MODEL`` names the model when the policy is "llm"; the deterministic
    arm has no model to name, so it names the arm instead. A results file
    whose ``model`` column read the same string whichever arm actually ran a
    mission could never say which one produced the row in front of it -- the
    one thing an ablation study cannot afford to be ambiguous about. Reads
    ``llm.MODEL`` fresh on every call, not a value cached at import, so a test
    that monkeypatches it after the fact still sees the patch.
    """
    return "deterministic" if deterministic() else llm.MODEL


# ---------------------------------------------------------------------------
# 1. Routing -- triage_node._decide_route
# ---------------------------------------------------------------------------


def route(task: Optional[object], prior_routes: Dict, node: str) -> Dict:
    """``repair`` or ``escalate``, from the retry budget this replaced and
    nothing else.

    Before triage read the fault at all, this question was answered by a
    counter: the mission planner replanned until ``MAX_RETRIES`` ran out, and
    running out was taken to mean the robot was incapable. That counter is
    exactly the structured signal this policy has: not the fault text, not the
    logs, not what the camera saw -- only whether the fleet could be offered
    anything for this failure, and whether local repair has already been tried
    once and failed again.

    ``task`` is whatever ``mission_tasks.task_for_node`` returned for the
    failing leaf -- a ``MissionTask`` or ``None`` -- typed here as a bare
    ``object`` rather than imported, for the reason the module docstring
    gives. Only its presence is read. ``task is None`` means the leaf could
    not be resolved to a delegable unit, which makes ``repair`` the only
    sensible answer regardless of anything else: there is nothing here an
    ``escalate`` verdict could hand to the fleet.

    Otherwise: if ``node`` already has an entry in ``prior_routes`` whose
    ``route`` was ``"repair"``, local repair has had its one attempt and the
    fault recurred, so this escalates. Everything else -- the first fault on
    a node, or a node whose only prior verdict was already ``escalate`` --
    stays (or returns to) ``repair``. That second clause cannot actually
    recur in production today, since ``triage_node`` republishes a cached
    ``escalate`` without calling this function again, but a pure function is
    tested on its own terms, not on today's caller.

    ``prior_routes`` is read, never written: this module keeps no history of
    its own. The caller already has one -- ``TriageNode.routes`` -- and
    threading it in as an argument is what keeps a mission's routing state
    from existing in two places that could disagree.

    ``guidance`` is always empty. Advice about *how* to repair is exactly the
    kind of thing that can only come from reading the fault, and this policy
    does not. ``reason_code`` is always ``UNSPECIFIED``: without the fault
    text there is no basis for a more specific diagnostic label, and a label
    invented to look informative would be worse than none.
    """
    if task is None:
        return {
            "route": "repair",
            "reason_code": int(ReasonCode.UNSPECIFIED),
            "rationale": (
                "this fault does not resolve to a unit of work the fleet "
                "could be offered, so there is nothing to escalate"
            ),
            "guidance": "",
        }

    prior = prior_routes.get(node)
    if prior is not None and str(prior.get("route")) == "repair":
        return {
            "route": "escalate",
            "reason_code": int(ReasonCode.UNSPECIFIED),
            "rationale": (
                f"{node!r} was already routed to local repair once this "
                "mission and the fault recurred"
            ),
            "guidance": "",
        }

    return {
        "route": "repair",
        "reason_code": int(ReasonCode.UNSPECIFIED),
        "rationale": f"first fault on {node!r} this mission; try local repair",
        "guidance": "",
    }


# ---------------------------------------------------------------------------
# 2. Anomaly interpretation -- triage_node._on_interpret
# ---------------------------------------------------------------------------


def interpret_anomaly(required_capabilities: int, peers_json: str) -> Dict:
    """``re_delegate`` when the fleet can plausibly help, ``drop_task`` when
    it cannot, and never ``add_task``.

    The only structured evidence this decision has is the peer registry: who
    is out there, whether they are idle, and what they can do. Shedding a
    task to a peer that is already busy, or that cannot do the work, is not a
    real re-delegation -- it just moves the failure to whichever robot the
    coordinator's own retry logic finds next. So this scans ``peers_json``
    (the same list the language-model branch is shown, in the same shape
    ``triage_client._peers_json`` builds it in: ``id``, ``capabilities`` as
    element names, ``idle``) for the first peer that is both idle and whose
    capability set covers ``required_capabilities``, and re-delegates if one
    exists.

    ``add_task`` is not in this function's range, on purpose, and that is a
    finding of this study rather than a gap in it: taking on *different* work
    than the task that failed requires having noticed something in the world
    -- a tree with fruit on it nobody had planned for, a person who needs
    following -- and nothing structured carries that. Every field available
    here describes the task that already failed and the peers who might take
    it, never anything new to do instead. A policy that emitted ``add_task``
    anyway would have to invent the "something noticed", and an invented
    observation is worse than none.

    ``note`` is always empty: a note is a sentence written for whoever wins
    the task, and this policy writes no sentences -- there is nothing here
    that was not already in the structured fields the announcement carries
    regardless.

    Malformed or empty ``peers_json`` reads as no peers, which drops the task
    exactly as an empty fleet should: there is no one for a re-delegation to
    reach.
    """
    peer = _capable_idle_peer(required_capabilities, peers_json)
    if peer is not None:
        return {
            "action": "re_delegate",
            "reason_code": int(ReasonCode.UNSPECIFIED),
            "fallback": "hold",
            "disposition": "",
            "task_id": 0,
            "capabilities": 0,
            "target": None,
            "priority": 0,
            "rationale": (
                f"peer {peer.get('id')!r} is idle and its capabilities cover "
                "what this task requires"
            ),
            "note": "",
        }

    return {
        "action": "drop_task",
        "reason_code": int(ReasonCode.UNSPECIFIED),
        "fallback": "",
        "disposition": "hold",
        "task_id": 0,
        "capabilities": 0,
        "target": None,
        "priority": 0,
        "rationale": "no idle peer in the peer list covers this task's required capabilities",
        "note": "",
    }


def _capable_idle_peer(required_capabilities: int, peers_json: str) -> Optional[Dict]:
    """The first idle peer in ``peers_json`` whose capabilities cover the
    requirement, or None.

    ``peers_json`` names capabilities by XML element name (the same words the
    mission is written in -- see ``triage_client._peers_json``), so this
    rebuilds the mask from ``CAPABILITY_BY_ELEMENT`` rather than asking the
    wire for a number it does not carry. An element name the table does not
    recognise is skipped for that one peer rather than failing the whole
    lookup: a peer whose advertisement this robot cannot parse is exactly a
    peer this robot cannot confirm can help, which is the same as it not
    covering the requirement.
    """
    try:
        peers = json.loads(peers_json) if peers_json else []
    except (TypeError, ValueError):
        return None
    if not isinstance(peers, list):
        return None

    for peer in peers:
        if not isinstance(peer, dict) or not peer.get("idle"):
            continue
        names = peer.get("capabilities")
        if not isinstance(names, list):
            continue
        mask = 0
        for name in names:
            capability = CAPABILITY_BY_ELEMENT.get(str(name))
            if capability is not None:
                mask |= 1 << int(capability)
        if (mask & required_capabilities) == required_capabilities:
            return peer
    return None


# ---------------------------------------------------------------------------
# 3. Note interpretation -- note_node._on_interpret
# ---------------------------------------------------------------------------


def interpret_note() -> Dict:
    """Always ``keep``. Mirrors ``ports/reasoning.IgnoreNotes`` by name and
    by behaviour, because the two are the same policy for the same reason.

    A note is free text, in full: the coordinator's own protocol carries
    nothing else in it, and this module reads no free text anywhere. There is
    therefore no structured field for a general-purpose rule to act on here
    -- not a smaller one than the other four seams have, none at all. That is
    itself a result worth stating plainly rather than working around: this
    seam is not meaningfully ablatable, because a deterministic policy and
    "no policy" are the same function. ``IgnoreNotes`` already is that
    function, in the coordinator's own vocabulary, which is why this does not
    invent a second name for it.

    Takes no arguments at all, rather than accepting the note text and
    ignoring it, because a signature with room for the text would invite a
    future edit to start reading it "just this once". A function that cannot
    be called with the text in hand cannot be tempted.
    """
    return {
        "revision": "keep",
        "cost_delta": 0,
        "reason": "no note interpreter configured",
        "rationale": (
            "the deterministic policy has no structured field to revise a "
            "bid from; see ports.reasoning.IgnoreNotes"
        ),
    }


# ---------------------------------------------------------------------------
# 4. Replanning -- mission_planner_node._run_planner
# ---------------------------------------------------------------------------


def replan(pruned_xml: str, failure_node: str, mission_tasks, ontology) -> str:
    """Cut the failed unit out of the plan and repair what that leaves
    behind, using only the action grammar.

    ``mission_tasks`` and ``ontology`` are ``mission_planner_node``'s own
    imports of those two modules, handed in rather than imported by this file
    -- see the module docstring for why: this file lives in ``runtime``, and
    the mission vocabulary lives one layer over, in ``mission``.

    The language-model branch is asked to rewrite the plan; this does the one
    edit that needs no judgement at all: the leaf named in the fault has
    already failed, so the unit it belongs to is resolved with
    ``mission_tasks.task_for_node`` and cut out whole with
    ``mission_tasks.remove_task`` -- the same two functions ``triage_node``
    uses to say what an escalation is offering the fleet, applied here to say
    what is no longer this robot's problem.

    Removing a unit can leave the rest of the plan in either of two states
    ``mission.ontology`` already has names for:

    * a **violation** -- a step whose required precondition nothing before it
      establishes any more. ``mission_tasks.remove_task`` is careful not to
      cause this for a *later* surviving unit (a shared prerequisite is kept
      out of the removed unit's own element list precisely so this cannot
      happen), but the repair loop below checks for it anyway rather than
      trusting that invariant silently: a violating leaf cannot run, and the
      only thing an action-grammar-only policy can do about a leaf that
      cannot run is take it out too.
    * something **dangling** -- a step whose established fact nothing after
      it needs any more, e.g. an aisle move that only ever served the unit
      just removed. Left in, it is not wrong, only pointless; taken out, the
      plan is smaller and no less correct. Missing *expected* preconditions
      (``ontology.dangling``'s other finding) are deliberately left alone --
      they are advice for a planner with judgement to act on, not a defect,
      and inventing a fix for one would mean guessing at a prerequisite this
      policy has no aisle map to place correctly anyway.

    Both are fixed the same way: find the offending leaf, cut it, prune any
    control node that removal left with nothing to run, and repeat until
    neither finding remains. The result never touches which tree, which
    aisle, or which robot anything names -- only the ``Fact``/``Need`` kinds
    ``mission.ontology`` already defines for the action grammar itself.

    **Only the findings this removal actually introduced.** ``ontology.
    dangling`` reports a mission's own trailing moves too -- a plan that ends
    ``..., SampleLeaf, MoveToAisleHead`` has that last aisle move flagged as
    establishing a fact nothing after it needs, simply because nothing comes
    after it, and the module's own docstring is explicit that this is a
    finding for a planner to weigh, not a defect: "a mission that ends by
    driving out of its last aisle is perfectly sensible". Repairing *every*
    finding ``dangling`` produces on the edited plan would delete that closing
    move -- present, and just as harmless, before this policy touched
    anything. So this compares findings before the removal against findings
    after it, by the element's own ``name``, and repairs only the ones that
    were not already there: the trailing move stays, and whatever the removal
    itself orphaned does not.

    Returns "" -- which the caller's own downstream ``xsd.validate`` already
    treats as "not well-formed, do not publish" -- when the failed node
    cannot be resolved to a unit at all, or when cutting it out leaves no
    task in the plan. Both are "no candidate", the same outcome an LLM call
    that raised would have produced.
    """
    task = mission_tasks.task_for_node(pruned_xml, failure_node)
    if task is None:
        return ""

    removed_xml = mission_tasks.remove_task(pruned_xml, task.task_id)
    if removed_xml is None or not mission_tasks.tasks_in(removed_xml):
        return ""

    root = mission_tasks.parse(removed_xml)
    if root is None:
        return ""

    before_root = mission_tasks.parse(pruned_xml)
    already_violating, already_dangling = (
        _findings(before_root, ontology)
        if before_root is not None
        else (frozenset(), frozenset())
    )
    _repair(root, ontology, already_violating, already_dangling)

    if not any(True for _ in ontology.actions_in(root)):
        return ""

    return etree.tostring(root, encoding="unicode")


def _findings(root, ontology) -> "Tuple[frozenset, frozenset]":
    """``(violating names, dangling names)`` -- the same two findings
    ``ontology.violations``/``ontology.dangling`` report, keyed by each
    offending element's own ``name`` instead of rendered to a sentence.

    A name rather than the element itself, because the caller wants to ask
    "was this already a finding before the edit" against a *different* parsed
    copy of a similar-but-not-identical document, where no element identity
    survives from one parse to the other. Every leaf in a mission this fleet
    flies is named -- ``task_for_node`` already depends on that -- so this
    reuses the identity the rest of the package already relies on rather than
    inventing a second one.
    """
    violating = set()
    dangling_names = set()
    resolved = []
    state = ontology.State()
    for element in ontology.actions_in(root):
        step, state = ontology.advance(state, element)
        resolved.append((step, element))
        if step.violation:
            violating.add(element.get("name") or "")
    for index, (step, element) in enumerate(resolved):
        if step.proposition in (ontology.OBJECTIVE, ontology.ACHIEVEMENT):
            continue
        established = [f for f in step.establishes if f.kind not in ontology.LATCHING]
        if established and not _used_later(established, resolved[index + 1 :]):
            dangling_names.add(element.get("name") or "")
    return frozenset(violating), frozenset(dangling_names)


def _repair(
    root, ontology, already_violating: frozenset, already_dangling: frozenset
) -> None:
    """Remove, in place, whatever ``ontology.violations``/``ontology.
    dangling`` newly report on ``root`` -- not what they reported already.

    Neither function hands back the element it is talking about -- they
    return sentences, for a planner with judgement to read -- so this walks
    the same way they do (``ontology.State`` folded forward over ``ontology.
    actions_in``) and acts on the first *new* offender it finds, then starts
    the walk over: removing one leaf can change which facts anything later in
    the plan still needs, so a second offender found against a state computed
    before the first removal is a stale answer. Bounded by construction --
    every pass removes exactly one leaf from a finite plan -- so this always
    terminates.
    """
    while True:
        resolved = []
        state = ontology.State()
        for element in ontology.actions_in(root):
            step, state = ontology.advance(state, element)
            resolved.append((step, element))

        violator = next(
            (
                element
                for step, element in resolved
                if step.violation
                and (element.get("name") or "") not in already_violating
            ),
            None,
        )
        if violator is not None:
            _remove_and_prune(violator)
            continue

        stray = _first_new_dangling_establisher(resolved, ontology, already_dangling)
        if stray is not None:
            _remove_and_prune(stray)
            continue

        return


def _first_new_dangling_establisher(resolved, ontology, already_dangling: frozenset):
    """The first step, in plan order, whose non-latching fact nothing later
    needs and which was not *already* dangling before this edit.

    The work itself is never a candidate: an ``OBJECTIVE`` or ``ACHIEVEMENT``
    step is what the unit was *for*, not a leftover, exactly as
    ``ontology.dangling`` excludes them. A latching fact (``sampled_tree`` and
    friends) is permanent by definition, so "nothing later needs it" says
    nothing about whether it was worth establishing.
    """
    for index, (step, element) in enumerate(resolved):
        if step.proposition in (ontology.OBJECTIVE, ontology.ACHIEVEMENT):
            continue
        if (element.get("name") or "") in already_dangling:
            continue
        established = [f for f in step.establishes if f.kind not in ontology.LATCHING]
        if not established:
            continue
        if not _used_later(established, resolved[index + 1 :]):
            return element
    return None


def _used_later(established, rest) -> bool:
    """Whether anything in ``rest`` needs one of ``established`` before some
    later step retracts it. The same rule ``ontology._used_later`` applies,
    reproduced here rather than imported: this module answers only to the
    ontology's public vocabulary (``State``, ``advance``, ``actions_in``,
    ``Step``, ``Fact`` kinds), the same boundary ``mission_tasks`` and
    ``ontology`` already keep from each other (see ``orchard.py``'s docstring
    for why two small readings of the same idea are kept apart rather than
    shared) -- and the rule is six lines.
    """
    for step, _ in rest:
        if any(need.met_by(fact) for need in step.needs for fact in established):
            return True
        if any(fact.kind in step.clears for fact in established):
            return False
    return False


def _remove_and_prune(element) -> None:
    """Cut ``element`` out of the tree, then remove any control-node ancestor
    that removal left with nothing to run.

    Mirrors ``mission_tasks._prune_empty`` rather than importing it: that
    function is called from one particular removal site with its own
    bookkeeping around it, and reimplementing its ten lines here keeps this
    module's only dependency on ``mission_tasks`` the two public functions
    ``replan`` already calls. The XSD requires a control node to have at
    least one child, so leaving an emptied one behind would hand the caller's
    own ``xsd.validate`` a document built to fail it.
    """
    parent = element.getparent()
    if parent is None:
        return
    parent.remove(element)
    node = parent
    while node is not None and node.tag != "BehaviorTree":
        above = node.getparent()
        if above is None or above.tag == "BehaviorTree":
            return
        if any(isinstance(child.tag, str) for child in node):
            return
        above.remove(node)
        node = above


# ---------------------------------------------------------------------------
# 5. Viability budget -- arbiter_node._compute_viability_budget
# ---------------------------------------------------------------------------

#: What fraction of a mission's objective trees this policy will let an
#: arbiter accept as dropped before it aborts the mission instead. A policy
#: choice, stated as one, not a value fit to any mission this fleet has run:
#: it says "up to one tree in five may go undone without the mission being
#: considered a failure", which is a legible, round number rather than a
#: tuned one -- the same status ``DEFAULT_VIABILITY_BUDGET = 2`` already has
#: as the language-model branch's own fallback when its call fails.
VIABILITY_FRACTION = 0.2


def viability_budget(n_trees: int) -> int:
    """A viability budget as a fixed fraction of the mission's size.

    Deliberately a function of ``n_trees`` alone. The language-model branch
    is handed the mission's free text on the theory that a mission which says
    "sample every tree, this is for a compliance audit" tolerates fewer
    drops than one that says "spot-check the orchard" -- a real distinction,
    and one this policy cannot see, because reading which is which requires
    reading the sentence. What is left without it is size: a bigger mission
    can lose the same *fraction* of its objectives and still have delivered
    most of what it set out to do, so the budget scales with ``n_trees``
    rather than being the same flat number for a two-tree run and a
    two-hundred-tree one.

    Returns the fraction rounded, with a floor of one tree: a mission that
    permits *zero* drops has no viability budget at all, which is a stricter
    policy than "abort on the first justified failure" was ever meant to
    express here. The caller's own ``max(1, min(n, n_trees))`` clamp still
    applies on top of this, unchanged, exactly as it does to the
    language-model branch's answer -- this function does not duplicate it.
    """
    return max(1, round(VIABILITY_FRACTION * n_trees))


# ---------------------------------------------------------------------------
# 6. Permanence -- arbiter_node._check_objective_preserved
# ---------------------------------------------------------------------------

_PERMANENCE_MODES = frozenset({"keywords", "llm"})

#: A second, narrower switch than ``AGENT_POLICY``, and deliberately not a
#: value ``AGENT_POLICY`` itself can take. For the other five seams, "llm" is
#: the default because a model call was already the shipped behaviour at
#: those five points before this study existed -- ``AGENT_POLICY`` unset
#: reproduces what the fleet has always done, and "deterministic" is the new
#: arm being measured against it. Permanence is not a fifth instance of that
#: shape wearing a sixth number: before this seam, ``_check_objective_preserved``
#: answered Gate 1 with the keyword table below, unconditionally, for every
#: robot that has ever run this code -- there was no arm, because there was
#: no choice to make. Wiring ``AGENT_POLICY=llm`` (or its default) to a model
#: call here would not add a second arm to an existing choice, it would
#: create a choice at a point this study exists to measure, not to edit. An
#: ablation earns the right to swap out reasoning a system already had; it
#: does not earn the right to give the system reasoning it never had, merely
#: because the harness that swaps out the other five also happens to run
#: past this line. So permanence gets its own selector, defaulted to the one
#: behaviour every configuration of this fleet has ever produced here, and
#: reachable only by naming it explicitly:
#:
#:     AGENT_PERMANENCE    "keywords" (default) or "llm"
#:
#: Read once at import, validated the same way ``AGENT_POLICY`` is above and
#: for the same reason: a typo that silently fell back to "keywords" would be
#: the harmless direction to fail in, but a typo that silently fell back to
#: "llm" would be exactly the regression this switch exists to make
#: impossible -- an ablation run quietly sending fault text to a model on a
#: seam its own author believed was still the keyword table.
AGENT_PERMANENCE = os.environ.get("AGENT_PERMANENCE", "keywords").strip().lower()
if AGENT_PERMANENCE not in _PERMANENCE_MODES:
    raise RuntimeError(
        f"AGENT_PERMANENCE={AGENT_PERMANENCE!r} is not one of "
        f"{sorted(_PERMANENCE_MODES)}"
    )


def permanence_uses_model() -> bool:
    """Whether the permanence seam's model arm is opted into.

    Deliberately not phrased in terms of ``deterministic()``: that function
    answers for the other five seams, where ``AGENT_POLICY`` is the whole
    story, and this one must not be a sixth reader of it. Returns ``True``
    only when ``AGENT_PERMANENCE`` was explicitly set to ``"llm"`` -- under
    every value of ``AGENT_POLICY``, including its default, leaving
    ``AGENT_PERMANENCE`` unset means this returns ``False`` and the model is
    never reached for this question. That is the property the regression
    this switch exists to prevent would have violated: a caller that checked
    ``deterministic()`` here would take the model branch by default, because
    the default *of that other switch* is "llm" -- correct for the five
    seams it governs, wrong for this one, which it was never supposed to
    govern at all.
    """
    return AGENT_PERMANENCE == "llm"


#: Moved here unchanged from arbiter_node.py, where it used to be the only
#: policy that existed for this question -- not a fallback for one, the
#: whole of it, for every arm, because nothing before this seam distinguished
#: "deterministic" from "llm" here at all. See ``permanence``'s own docstring
#: for why this table is allowed to open the fault text when nothing else in
#: this module is.
PERMANENT_KEYWORDS = ("permanent", "removed", "unavailable", "does not exist")


def permanence(reason: str) -> bool:
    """Whether the failure described by ``reason`` is permanent -- final
    enough that dropping the objective it blocked is a justified loss rather
    than one the mission gave up on too early.

    This is the one function in this module that breaks the rule the module
    docstring states for the other five: it opens the fault text. That is not
    an oversight, it is the finding this seam exists to report. Gate 1 of
    ``_check_objective_preserved`` needs an answer to "will this ever succeed
    if retried", and nothing *structured* carries that -- not the retry
    count, not the peer registry, not the mission ontology. A tree that no
    longer exists and a tree a robot merely failed to reach today produce the
    identical structured state: one fewer objective in the candidate plan.
    The only place the distinction is written down at all is the sentence a
    fault handler chose to describe what happened, which makes this seam
    different in kind from the other five, not just in subject: there, "do
    not read the free text" was the honest closed-form answer; here, refusing
    to read it would not produce a narrower safe action, it would just be
    wrong about which trees are gone for good.

    So the deterministic arm reads the fault text the same narrow way the
    system already did before this seam existed: a substring search for one
    of four words -- ``PERMANENT_KEYWORDS``, moved here unchanged -- over the
    reason lowercased first, exactly as ``_check_objective_preserved`` has
    always lowercased it before matching. That case-folding is preserved
    here rather than left to the caller, so a caller that forgets to lower
    its own string first still gets the answer today's code has always given
    a mixed-case fault ("Tree Removed" matches, the same as "tree removed"
    always has). It is a general-purpose reader in the sense that it does not
    know which scenario wrote the sentence -- only that it recognises these
    four words -- but it is emphatically not a *good* reader: "the spot is
    permanently empty" and "no tree at the mapped location; the spot is bare
    soil" describe the same fact and only one of them matches. That gap is
    exactly what the language-model branch is being measured against.

    An empty ``reason`` (no failure latched yet, or one whose event carried
    no ``reason`` field) matches no keyword and returns ``False`` -- the same
    "not shown to be permanent" default the substring search always gave an
    empty string, and the safe direction for an ablation to fail in: an
    unjustified drop is only ever a fixable rejection (Gate 1), never the
    abort Gate 2 can reach, so understating permanence costs a retry, never
    a mission.
    """
    return any(kw in reason.lower() for kw in PERMANENT_KEYWORDS)
