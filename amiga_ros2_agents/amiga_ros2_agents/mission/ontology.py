"""What the actions mean: what each one needs, and what it leaves true.

A mission is not a bag of independent leaves. Sampling tree 60 is *enter aisle
6, reach tree 60, sample it* -- three actions in a chain, where the last one is
the only part anybody names. Every piece of this system that reasons about a
plan needs that chain, and until this module existed each of them had its own
partial copy of it:

    promela._Emitter._at            a SampleLeaf samples the tree the walk last
                                    moved to
    arbiter._check_no_orphan_sample the same rule, scoped to one Sequence
    mission_tasks._split_by_objective   a unit is a movement plus the work after

None of the three knew that reaching a tree expects entering its aisle, so no
unit ever contained its aisle move -- and a task handed to another robot
arrived without the part that gets the robot there. That is the gap this closes.

**Two strengths of precondition, and the distinction is load-bearing.**

    required   a plan that violates it is refused. Exactly one today:
               SampleLeaf needs at_tree, which *is* the arbiter's orphan check.
    expected   a plan may skip it and still be correct. MoveToTreeID expects
               in_aisle(aisle_of(t)) -- navigation will route there regardless,
               and plans this fleet already flies do exactly that.

Conflating them would start rejecting missions the planner has always emitted.
Keeping them apart is what lets the same table serve verification (which must
only reject what is genuinely wrong) and planning (which wants the whole chain).

**The table is closed against the schema.** One row per element of the XSD's
``<xs:group name="ActionGroup">``; ``covers`` checks that against
``promela.action_pool`` so a new action in the schema is a loud test failure
rather than a leaf that silently means nothing. The vocabulary a row is written
in is small on purpose -- five fact kinds -- because a predicate nobody can
establish is a predicate that only ever blocks a plan.

**Position is exclusive; achievement latches.** A robot is at one place and in
one aisle at a time, so a move retracts where it used to be. Having sampled a
tree is permanent. That asymmetry is the same one ``promela`` encodes in its
macros and its latches, and it is why a state here is a set of facts rather
than a list of things that happened.

No ROS, no lxml beyond element inspection, so this is testable against real
mission files with nothing running.
"""

import copy
from dataclasses import dataclass
from typing import Dict, FrozenSet, Iterator, List, Optional, Tuple

from amiga_ros2_comms.codec import CAPABILITY_BY_ELEMENT

# --------------------------------------------------------------------------
# The vocabulary
# --------------------------------------------------------------------------

#: The robot has approached tree ``t`` and is positioned to work on it.
AT_TREE = "at_tree"
#: The robot is on the row waypoint beside tree ``t`` but has not approached it
#: -- ``approach_tree="false"``. A place, not an objective, and specifically not
#: somewhere a SampleLeaf may fire.
AT_WAYPOINT = "at_waypoint"
#: The robot is at a lat/lon it was sent to.
AT_GPS = "at_gps"
#: The robot has entered aisle ``a`` and is driving down it.
IN_AISLE = "in_aisle"
#: Tree ``t`` has been sampled. Latches.
SAMPLED_TREE = "sampled_tree"
#: The arm is at a commanded pose. Latches.
ARM_POSITIONED = "arm_positioned"

#: Facts that stay true once true. Having sampled a tree survives driving away
#: from it; being at the tree does not.
LATCHING = frozenset({SAMPLED_TREE, ARM_POSITIONED})

#: Where the robot is. One at a time: establishing any of these retracts the
#: others, which is what makes "sample where you last moved to" mean something.
WHERE = frozenset({AT_TREE, AT_WAYPOINT, AT_GPS})
#: Which lane the robot is in. Also one at a time.
AISLE = frozenset({IN_AISLE})
#: What driving somewhere invalidates: both of the above.
TRAVELS = WHERE | AISLE

#: Precondition strengths. See the module docstring -- the difference decides
#: whether an unmet need is a rejection or a suggestion.
REQUIRED = "required"
EXPECTED = "expected"

#: How an action shows up in the verification model's vocabulary. Named here
#: rather than in ``promela`` so the emitter is a lookup with no tag literals
#: in it, which is what stopped the two drifting apart in the first place.
OBJECTIVE = "objective"  # somewhere the mission wanted the robot to be
ACHIEVEMENT = "achievement"  # something the mission wanted done, latching
DONE = "done"  # ran; nothing else to say about it
SILENT = "silent"  # transit, deliberately not a proposition


#: The table in words, for the mission planner's prompt. A language model
#: writing a plan needs the same relationships this module enforces, and the two
#: drifting apart is how you get a planner that emits plans the gate rejects.
#: Prose rather than generated from the rows because the *reason* is the useful
#: part, and a generator would only ever produce the predicate.
#: ``test_ontology`` holds this to the table: a required precondition with no
#: sentence here is a rule the planner is never told about.
RULES = (
    'A <SampleLeaf> must be preceded by a <MoveToTreeID approach_tree="true">. '
    "It samples whatever the robot is in front of and carries no location of "
    "its own, so this one is REQUIRED -- a plan that breaks it is rejected.",
    "A <MoveToTreeID> should be preceded by a <MoveToAisleHead> naming that "
    "tree's aisle. Navigation reaches the tree either way, so this is EXPECTED "
    "rather than required, but a plan that skips it crosses the orchard instead "
    "of driving down the lane.",
    "A <MoveToAisleHead> ends being at a tree. After one, the robot is in the "
    "aisle and has to approach a tree again before it can sample.",
    "Sampling a tree is therefore three actions, not one: enter the aisle, "
    "approach the tree, sample it.",
)


class OntologyError(ValueError):
    """The table cannot describe this element. Carries a usable reason."""


@dataclass(frozen=True)
class Fact:
    """One thing that is true after some action ran.

    ``arg`` is the thing it is true *of* -- a tree id, an aisle id -- as the
    string the XML carried, so a fact compares equal to one built from the
    attribute it came from without either side deciding on a numeric type.
    """

    kind: str
    arg: str = ""

    def __str__(self) -> str:
        return f"{self.kind}({self.arg})" if self.arg else self.kind


@dataclass(frozen=True)
class Need:
    """A precondition: a fact kind, optionally a binding, and how hard it is.

    An empty ``arg`` means any binding will do -- ``SampleLeaf`` needs the robot
    to be at *a* tree, and which one is what the sample is then *of*.
    """

    kind: str
    strength: str = REQUIRED
    arg: str = ""

    def met_by(self, fact: Fact) -> bool:
        """Whether ``fact`` satisfies this need."""
        return fact.kind == self.kind and (not self.arg or fact.arg == self.arg)

    def __str__(self) -> str:
        return f"{self.kind}({self.arg})" if self.arg else f"{self.kind}(?)"

    @property
    def phrase(self) -> str:
        """The need in words, for a rejection a planner has to act on.

        The reason string goes back to the model that wrote the plan, so it
        names the element that would fix it rather than the predicate that
        failed -- a planner cannot emit an ``at_tree``.
        """
        return _PHRASE.get(self.kind, str(self))


#: What each need looks like as a thing a plan can contain.
_PHRASE = {
    AT_TREE: 'a preceding <MoveToTreeID approach_tree="true">',
    AT_WAYPOINT: "a preceding <MoveToTreeID>",
    AT_GPS: "a preceding <MoveToGPSLocation> or <ApproachGPSWaypoint>",
    IN_AISLE: "a preceding <MoveToAisleHead>",
    ARM_POSITIONED: "a preceding <MoveArmToPosition>",
}


@dataclass(frozen=True)
class Step:
    """One action, resolved against the state it would run in.

    Resolved, because half of what an action means depends on where the robot
    already is: ``SampleLeaf`` establishes ``sampled_tree(60)`` only in a state
    where the robot is at tree 60, and establishes nothing at all in a state
    where it is nowhere -- which is the same thing as the plan being wrong.
    """

    tag: str
    name: str
    needs: Tuple[Need, ...]
    establishes: Tuple[Fact, ...]
    clears: FrozenSet[str]
    proposition: str
    #: Needs the state did not satisfy, both strengths, in table order.
    unmet: Tuple[Need, ...] = ()
    #: Set when the element itself is malformed -- a MoveToTreeID with no id.
    #: Distinct from an unmet need: nothing earlier in the plan could fix it.
    defect: str = ""

    @property
    def violation(self) -> str:
        """Why this step must be refused, or "" if it need not be."""
        if self.defect:
            return self.defect
        for need in self.unmet:
            if need.strength == REQUIRED:
                return (
                    f"<{self.tag}> '{self.name}' requires {need.phrase}, and "
                    f"nothing before it in the plan establishes that"
                )
        return ""

    @property
    def missing(self) -> Tuple[Need, ...]:
        """The *expected* needs the state did not meet.

        What a planner could usefully be told, and what ``synthesize`` fills in
        when it rebuilds a task from an announcement.
        """
        return tuple(n for n in self.unmet if n.strength == EXPECTED)


@dataclass(frozen=True)
class State:
    """What is true at a point in a plan.

    Immutable, so a ``Fallback``'s branches can each be walked from the same
    starting point and then merged, which is the only place in a conforming
    tree where control actually diverges.
    """

    facts: FrozenSet[Fact] = frozenset()

    def holding(self, kind: str) -> Optional[Fact]:
        """The fact of ``kind`` that holds here, or None.

        At most one for the exclusive kinds, which is every kind that answers a
        "where is the robot" question; for the latching kinds this returns an
        arbitrary one and no caller asks.
        """
        return next((f for f in self.facts if f.kind == kind), None)

    def meets(self, need: Need) -> bool:
        """Whether some fact here satisfies ``need``."""
        return any(need.met_by(fact) for fact in self.facts)

    def after(self, step: "Step") -> "State":
        """The state ``step`` leaves behind."""
        kept = frozenset(f for f in self.facts if f.kind not in step.clears)
        return State(kept | frozenset(step.establishes))

    def merge(self, other: "State") -> "State":
        """What both states agree on.

        Intersection rather than union: after a ``Fallback`` the robot is
        wherever the branch that ran left it, and a fact only one branch
        establishes is not something the plan can rely on.
        """
        return State(self.facts & other.facts)


# --------------------------------------------------------------------------
# The table
# --------------------------------------------------------------------------


def _move_to_tree(element, orchard, state) -> dict:
    """MoveToTreeID: the objective, or the transit that leads to one.

    ``approach_tree`` is the whole difference. True means the robot stops at the
    tree and can work there; false means it drives past on the row waypoint.
    The arbiter's ``_objective_tree_ids`` draws the line in exactly this place,
    and the two agreeing is what stops a plan satisfying "sample tree 60" by
    driving past it.
    """
    tree = (element.get("id") or "").strip()
    if not tree:
        return {"defect": f"<MoveToTreeID> '{_name(element)}' has no id"}
    approach = (element.get("approach_tree") or "").strip().lower() == "true"
    aisle = orchard.aisle_of(tree) if orchard is not None else None

    here = Fact(AT_TREE if approach else AT_WAYPOINT, tree)
    establishes = (here,) if aisle is None else (here, Fact(IN_AISLE, str(aisle)))
    return {
        # Expected, not required: navigation routes to a tree from wherever the
        # robot is, so a plan that omits the aisle move is slower, not wrong.
        # Every plan in examples/ omits it. What the expectation buys is the
        # *closure* -- the aisle move belongs to this tree's task -- and a
        # prerequisite worth emitting when the task is rebuilt elsewhere.
        "needs": (Need(IN_AISLE, EXPECTED, "" if aisle is None else str(aisle)),),
        "establishes": establishes,
        "clears": TRAVELS,
        "proposition": OBJECTIVE if approach else SILENT,
    }


def _move_to_aisle_head(element, orchard, state) -> dict:
    """MoveToAisleHead: the prerequisite the rest of the chain hangs off."""
    aisle = (element.get("id") or "").strip()
    if not aisle:
        return {"defect": f"<MoveToAisleHead> '{_name(element)}' has no id"}
    return {
        "establishes": (Fact(IN_AISLE, aisle),),
        # Driving to the head of an aisle is driving away from wherever the
        # robot was standing, so it ends being at a tree.
        "clears": TRAVELS,
        "proposition": DONE,
    }


def _sample_leaf(element, orchard, state) -> dict:
    """SampleLeaf: the only required precondition in the table.

    It carries no location of its own -- it samples whatever the robot is in
    front of -- so in a state that does not say where that is, it establishes
    nothing and the plan is refused. That refusal is the arbiter's orphan
    check, moved here so the verifier and the gate cannot disagree about it.
    """
    at = state.holding(AT_TREE) if state is not None else None
    return {
        "needs": (Need(AT_TREE, REQUIRED),),
        "establishes": () if at is None else (Fact(SAMPLED_TREE, at.arg),),
        "proposition": ACHIEVEMENT,
    }


def _gps(element, orchard, state) -> dict:
    """MoveToGPSLocation / ApproachGPSWaypoint: a place named absolutely."""
    lat, lon = element.get("latitude"), element.get("longitude")
    if lat is None or lon is None:
        return {"defect": f"<{element.tag}> '{_name(element)}' has no coordinates"}
    return {
        "establishes": (Fact(AT_GPS, f"{lat},{lon}"),),
        "clears": TRAVELS,
        "proposition": DONE,
    }


def _arm(element, orchard, state) -> dict:
    """MoveArmToPosition: the manipulator moves, the base does not."""
    return {"establishes": (Fact(ARM_POSITIONED),), "proposition": DONE}


def _local(element, orchard, state) -> dict:
    """MoveToRelativeLocation / OrientRobotHeading: adjustments, not travel.

    They clear nothing. An offset from the robot's own pose and a heading
    change are how a plan lines *up* at a tree, so treating them as departures
    would make ``approach, nudge, sample`` an orphaned sample -- which is a
    reasonable plan the fleet would then refuse.
    """
    return {"proposition": DONE}


def _wait(element, orchard, state) -> dict:
    """Wait: time passes and the robot does not move.

    Clears nothing, on purpose. The pose it already had is the pose it still
    has, so "approach the tree, wait for someone to walk past, sample it" reads
    as one continuous approach rather than an approach, a gap, and an orphaned
    sample the fleet would refuse.
    """
    return {"proposition": DONE}


def _follow(element, orchard, state) -> dict:
    """FollowPerson: unbounded movement, so nowhere is known afterwards."""
    return {"clears": TRAVELS, "proposition": DONE}


#: One row per element of the XSD's ActionGroup. ``covers`` holds this to the
#: schema; adding an action there without adding it here is a test failure, not
#: a leaf that quietly means nothing.
TABLE: Dict[str, object] = {
    "MoveToTreeID": _move_to_tree,
    "MoveToAisleHead": _move_to_aisle_head,
    "MoveToGPSLocation": _gps,
    "ApproachGPSWaypoint": _gps,
    "MoveToRelativeLocation": _local,
    "OrientRobotHeading": _local,
    "FollowPerson": _follow,
    "MoveArmToPosition": _arm,
    "SampleLeaf": _sample_leaf,
    "Wait": _wait,
}

#: Every action this module can talk about.
ACTIONS = frozenset(TABLE)


def covers(pool) -> Tuple[bool, str]:
    """Whether the table describes exactly the schema's action vocabulary.

    Both directions matter. A schema action with no row is a leaf the planner
    may emit and nothing here understands; a row with no schema action is a
    rule about something that cannot appear in a plan, which will be believed
    anyway by whoever reads the table next.
    """
    declared = set(pool)
    missing = sorted(declared - ACTIONS)
    extra = sorted(ACTIONS - declared)
    if not missing and not extra:
        return True, ""
    parts = []
    if missing:
        parts.append(f"in the schema but not in the table: {missing}")
    if extra:
        parts.append(f"in the table but not in the schema: {extra}")
    return False, "; ".join(parts)


# --------------------------------------------------------------------------
# Walking a plan
# --------------------------------------------------------------------------


def is_action(element) -> bool:
    """Whether ``element`` is an action leaf rather than control or a comment."""
    tag = getattr(element, "tag", None)
    return isinstance(tag, str) and tag in CAPABILITY_BY_ELEMENT


def advance(state: State, element, orchard=None) -> Tuple[Step, State]:
    """Resolve one action against ``state`` and report the state it leaves.

    Never raises on an unmet precondition -- the caller decides what to do with
    ``step.violation``, because the verifier refuses, the planner is told, and
    the closure just makes a note. Raises only when handed something that is
    not an action at all, which is a bug in the caller.
    """
    tag = getattr(element, "tag", None)
    rule = TABLE.get(tag)
    if rule is None:
        raise OntologyError(f"<{tag}> is not an action this ontology describes")

    row = rule(element, orchard, state)
    needs = tuple(row.get("needs", ()))
    step = Step(
        tag=tag,
        name=_name(element),
        needs=needs,
        establishes=tuple(row.get("establishes", ())),
        clears=frozenset(row.get("clears", ())),
        proposition=row.get("proposition", DONE),
        unmet=tuple(n for n in needs if not state.meets(n)),
        defect=row.get("defect", ""),
    )
    if step.defect:
        return step, state
    return step, state.after(step)


def actions_in(root) -> Iterator:
    """Every action leaf under ``root``, in document order.

    Document order is execution order for a conforming tree: the XSD has every
    condition node commented out, so ``Sequence`` and the decorators run their
    children straight through. A ``Fallback``'s branches come out one after
    another instead of as alternatives, which over-approximates what the robot
    can rely on -- deliberately, since this order is what ``violations`` uses
    and an over-approximation there means fewer refusals rather than more.
    """
    for element in root.iter():
        if is_action(element):
            yield element


def violations(root, orchard=None) -> List[str]:
    """Every required precondition the plan breaks, in plan order.

    The plan-wide answer to the question the arbiter used to ask one Sequence
    at a time. That scope was the bug: a plan that moves to a tree inside one
    ``RetryUntilSuccessful`` and samples it inside the next -- which is how the
    mission planner writes a retried sample, and how two of the three plans
    this fleet is being tested with are written -- has no ``MoveToTreeID``
    among the ``SampleLeaf``'s siblings at all, and was refused for it.
    """
    state = State()
    out: List[str] = []
    for element in actions_in(root):
        step, state = advance(state, element, orchard)
        if step.violation:
            out.append(step.violation)
    return out


def dangling(root, orchard=None) -> List[str]:
    """What an edit left behind or took away. Neither is a reason to refuse.

    Shedding a task is not a local change. Handing tree 60 to a peer removes
    the drive into aisle 6 along with it -- but the drive back *out* of aisle 6
    now leads nowhere, and if two trees had shared that aisle move, the one
    still on this robot would have lost its way in. Both are visible from the
    same walk, in the same terms, and neither can be seen from the XML alone:

    * a step that establishes something nothing after it ever needs
    * a step whose expected prerequisite the plan no longer contains

    Reported as sentences rather than as structure because the consumer is the
    robot's own mission planner, which is a language model being told what
    changed and asked to write the plan again. A finding is a reason to think,
    not a verdict -- a mission that ends by driving out of its last aisle is
    perfectly sensible, and only the planner can tell that from a leftover.
    """
    steps: List[Step] = []
    state = State()
    for element in actions_in(root):
        step, state = advance(state, element, orchard)
        steps.append(step)

    out: List[str] = []
    for index, step in enumerate(steps):
        if step.proposition in (OBJECTIVE, ACHIEVEMENT):
            continue  # the work itself is never a leftover
        established = [f for f in step.establishes if f.kind not in LATCHING]
        if not established:
            continue
        if not _used_later(established, steps[index + 1 :]):
            facts = ", ".join(str(fact) for fact in established)
            out.append(
                f"<{step.tag}> '{step.name}' establishes {facts}, and nothing "
                f"after it in the plan needs that"
            )

    for step in steps:
        for need in step.missing:
            out.append(
                f"<{step.tag}> '{step.name}' expects {need.phrase}, which the "
                f"plan does not contain"
            )
    return out


def prune_completed(root, completed, orchard=None):
    """A copy of ``root`` with each already-sampled tree's objective removed.

    ``completed`` is tree ids (str or int) the robot has actually sampled this
    session -- see the SUCCESS half of ``/bt/status_change``, the only place
    in the stack that observes a leaf finishing. The executor rebuilds its
    tree from scratch and ticks from the root on every new plan (there is no
    resume point), so a replan that leaves a finished tree's steps in place
    drives the robot back through them. This is the deterministic half of
    fixing that: what is mechanical -- "this already happened, stop asking
    for it" -- is decided here, not left to the model's edit budget the way
    ``dangling``'s findings are.

    For each completed tree, removes the smallest ancestor that isolates its
    ``MoveToTreeID``/``SampleLeaf`` pair (climbing through a wrapping
    ``RetryUntilSuccessful`` the way the planner's own plans are shaped, but
    stopping the moment an ancestor holds any other tree's actions too), then
    drops any ``MoveToAisleHead`` whose aisle no *remaining* tree still needs
    -- never one a still-pending tree shares it with.

    Trees ``completed`` names that the plan does not contain, or that have no
    orchard entry, are silently ignored: nothing here is a violation, only a
    removal of what is provably already done.
    """
    completed_ids = {str(t).strip() for t in completed}
    if not completed_ids:
        return root

    doc = copy.deepcopy(root)
    objectives = _objective_bindings(doc, orchard)

    to_remove = set()
    aisles_still_needed = set()
    for move, sample, tree_id in objectives:
        if tree_id in completed_ids:
            # Both halves, not just the move. When the pair is wrapped in a
            # RetryUntilSuccessful of its own -- how the planner writes an
            # objective -- both climbs reach that wrapper and the set collapses
            # them to one removal, exactly as before. When it is NOT wrapped
            # they climb nowhere and return themselves, so both leaves go.
            #
            # That second shape is the one a task won at auction has: the graft
            # puts the aisle move, the approach, the sample and whatever the
            # planner added into one flat Sequence, so no ancestor holds the
            # pair alone. Asking only about the move then deleted only the
            # move, and the orphaned SampleLeaf failed the arbiter's own
            # precondition check -- "requires a preceding <MoveToTreeID
            # approach_tree='true'>" -- which rejected the whole plan. Live,
            # that cost amiga2 a task it had just won: it had sampled tree 20
            # from an earlier auction, won tree 26, and could not graft it
            # because pruning tree 20 left its sample behind.
            to_remove.add(_isolated_ancestor(move, {move, sample}))
            to_remove.add(_isolated_ancestor(sample, {move, sample}))
        else:
            aisle = orchard.aisle_of(tree_id) if orchard is not None else None
            if aisle is not None:
                aisles_still_needed.add(str(aisle))

    for node in to_remove:
        parent = node.getparent()
        if parent is not None:
            parent.remove(node)

    # 2. An aisle head only outlives the trees it was for if one of them
    #    hasn't happened yet.
    for element in list(doc.iter("MoveToAisleHead")):
        if (element.get("id") or "").strip() not in aisles_still_needed:
            parent = element.getparent()
            if parent is not None:
                parent.remove(element)

    # 3. Whatever the two steps above emptied goes with its contents.
    _drop_emptied_controls(doc)

    return doc


#: Control nodes, matching the XSD's ControlGroup. Spelled out here rather
#: than imported from ``mission_tasks``: that module builds on this one.
CONTROLS = frozenset(
    {"Sequence", "ReactiveSequence", "Fallback", "RetryUntilSuccessful"}
)


def _drop_emptied_controls(root) -> None:
    """Remove control nodes that pruning left holding nothing.

    The XSD requires every control node to have at least one child, so a
    wrapper emptied by pruning makes the entire plan invalid -- and the plan
    the planner shows the model is the *pruned* one, so the model is handed
    XML that cannot validate and its edit inherits the fault. That is the
    whole of ``LLM edit failed XSD validation -- Element 'Sequence': Missing
    child element(s)``, which fired on every absorb in a live run and cost the
    winner the one replan that would have re-derived the ``Wait`` the rebuild
    dropped.

    The path there: grafting a won task wraps the original leaves in a bare
    ``<Sequence>`` so the two stay separable, and pruning the robot's finished
    trees out of that wrapper leaves ``<Sequence/>`` behind.

    Deepest first, so a wrapper whose only child was itself emptied goes too.
    The ``BehaviorTree``'s own root control node always stays: it is what an
    absorbed task gets appended to, and a childless ``BehaviorTree`` is no
    better than an empty ``Sequence``.
    """
    protected = {child for tree in root.iter("BehaviorTree") for child in tree}
    for element in reversed(list(root.iter())):
        if element.tag in CONTROLS and len(element) == 0 and element not in protected:
            parent = element.getparent()
            if parent is not None:
                parent.remove(element)


def sample_leaf_trees(root, orchard=None) -> Dict[str, str]:
    """Every ``SampleLeaf``'s ``name`` -> the tree id it binds to.

    BT.CPP reports a leaf's *name* on status-change, not a tree id -- that
    attribute lives on the ``MoveToTreeID`` ahead of it, which is this
    schema's concept, not BT.CPP's. This is how a SUCCESS event's ``node``
    field (e.g. ``"sample_tree60"``) becomes a tree id for a completed-
    objectives ledger, using the same binding ``prune_completed`` removes by.
    """
    return {
        (sample.get("name") or ""): tree_id
        for _, sample, tree_id in _objective_bindings(root, orchard)
    }


def _objective_bindings(root, orchard=None):
    """Every ``(MoveToTreeID, SampleLeaf, tree_id)`` triple in the plan.

    The binding ``advance`` computes -- which tree a sample lands on is a
    function of where the state says the robot is, not of anything on the
    ``SampleLeaf`` element itself -- kept here instead of discarded, for
    ``prune_completed`` and ``sample_leaf_trees`` to share.
    """
    state = State()
    last_move_for: Dict[str, object] = {}
    out = []
    for element in actions_in(root):
        if (
            element.tag == "MoveToTreeID"
            and (element.get("approach_tree") or "").strip().lower() == "true"
        ):
            last_move_for[(element.get("id") or "").strip()] = element
        step, state = advance(state, element, orchard)
        if element.tag == "SampleLeaf":
            sampled = next(
                (f for f in step.establishes if f.kind == SAMPLED_TREE), None
            )
            if sampled is not None and sampled.arg in last_move_for:
                out.append((last_move_for[sampled.arg], element, sampled.arg))
    return out


def _isolated_ancestor(element, ours: set):
    """The highest ancestor of ``element`` whose action leaves are all in ``ours``.

    A bare pair of siblings climbs no further than themselves; a pair wrapped
    in ``RetryUntilSuccessful<Sequence>`` -- how every plan this fleet flies
    writes an objective -- climbs to that wrapper, so removing it does not
    leave an empty retry shell behind.
    """
    node = element
    parent = node.getparent()
    while parent is not None:
        descendants = {e for e in parent.iter() if is_action(e)}
        if descendants <= ours:
            node = parent
            parent = parent.getparent()
        else:
            break
    return node


def _used_later(facts, rest) -> bool:
    """Whether anything in ``rest`` needs one of ``facts`` before it is retracted.

    The scan stops at the step that clears them rather than running to the end
    of the plan: a second entry into the same aisle makes the first one's fact
    unreachable, so a later action needing it is being served by the second and
    says nothing about the first.
    """
    for step in rest:
        if any(need.met_by(fact) for need in step.needs for fact in facts):
            return True
        if any(fact.kind in step.clears for fact in facts):
            return False
    return False


def _name(element) -> str:
    return element.get("name") or "<unnamed>"
