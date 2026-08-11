"""Mission XML <-> coordination tasks. The one place that knows both.

The behaviour tree is the mission, so a task the fleet can trade has to be a
piece of behaviour-tree XML. This module is the translation, and it lives in the
agents package because the agents are what own the mission: the planner writes
it, the arbiter gates it, triage reads it. The coordinator deliberately never
parses XML -- it learns about tasks over the radio and over
``/coordination/infeasible``, in the flat vocabulary the codec defines.

**What one task is.** A subtree, not a leaf. In ``examples/sample_leafs.xml``::

    <MoveToTreeID name="Visit_Tree_60" id="60" approach_tree="true"/>
    <SampleLeaf   name="Sample_Leaves_Tree_60"/>

``SampleLeaf`` alone cannot be delegated: it has no place, it samples wherever
the robot is standing. The unit is the two together, and that is not an
invention here -- the arbiter's ``_check_no_orphan_sample`` already refuses a
plan where they come apart.

**Where the rules come from.** Each one is a restatement of something the
existing pipeline already believes:

    unit      the failing leaf's nearest enclosing control node, which is the
              scope the arbiter's orphan check already reasons in
    target    the objective, defined exactly as the arbiter's
              ``_objective_tree_ids``: a MoveToTreeID with approach_tree="true"
    identity  a ``__task_<n>`` suffix on the subtree's ``name`` if the plan
              declares one, else a hash of that name -- never the tree ``id``,
              which is a location and collides (tree 2 is visited twice in that
              same file). See TASK_ID_SEPARATOR for why it lives in the name.

**What is deliberately undelegable.** A subtree whose only movement is
``MoveToRelativeLocation`` gets ``Target.none()``. An offset from *this* robot's
pose names nowhere another robot can be sent, and the coordinator refuses to
announce it rather than announcing a place that means something different to
every listener.

No ROS. lxml and the codec vocabulary, nothing else, so this is testable against
the real files in ``examples/`` with no node running.
"""

import re
import zlib
from copy import deepcopy
from dataclasses import dataclass, field
from typing import List, Optional

from lxml import etree

from amiga_ros2_comms.codec import (
    CAPABILITY_BY_ELEMENT,
    PRIORITY_MAX,
    TASK_ID_MAX,
    Capability,
    Target,
    TargetKind,
    cap_mask,
)

from . import ontology

#: Control nodes that can delimit a task. The XSD's ControlFlowGroup, less
#: nothing -- any of them may carry the task identity attributes.
CONTROL_TAGS = frozenset(
    {"Sequence", "ReactiveSequence", "Fallback", "RetryUntilSuccessful"}
)

#: Actions that establish where the work happens, most specific first. Order is
#: the point: a subtree that both approaches a tree and passes an aisle head is
#: about the tree, and the traverse is how it gets there.
TARGET_ACTIONS = (
    "MoveToTreeID",
    "MoveToAisleHead",
    "ApproachGPSWaypoint",
    "MoveToGPSLocation",
)

#: Every action that moves the robot. A new unit of work begins at one of
#: these, once the current unit already has somewhere to be. OrientRobotHeading
#: and MoveToRelativeLocation are movements without being objectives: they say
#: how to move, not where to.
MOVEMENT_ACTIONS = frozenset(
    set(TARGET_ACTIONS) | {"MoveToRelativeLocation", "OrientRobotHeading"}
)

#: Default priority for a task the plan does not rank. Mid-range rather than 0,
#: so work discovered later can be ranked either side of it.
DEFAULT_PRIORITY = 100

#: How a plan declares a task's fleet id: a suffix on the control node's
#: ``name``, so ``<Sequence name="Tree60Seq__task_5">`` is task 5.
#:
#: **Not an attribute of its own, and that is not a style choice.** BT.CPP
#: reserves exactly ``ID``, ``name`` and the eight pre/post-condition attributes
#: (``_skipIf``, ``_while``, ``_onSuccess`` ...); *every* other attribute on a
#: registered node is parsed as a **port**, and a port the node did not declare
#: is a hard error:
#:
#:     a port with name [task_id] is found in the XML (<Sequence>, line 4) but
#:     not in the providedPorts() of its registered node type
#:
#: So a plan carrying ``task_id="5"`` does not load at all -- and neither would
#: a mission a winner absorbed, which is the failure that matters, because it
#: lands after the auction has already succeeded. ``name`` is the only field
#: that is both ours to write and safe to write.
TASK_ID_SEPARATOR = "__task_"

#: The label is optional so that the bare ``task_<n>`` the arbiter already gives
#: an absorbed task (``arbiter_node._apply_task_edit``) declares its id too.
_DECLARED_ID = re.compile(r"^(?:(?P<label>.+)__)?task_(?P<id>\d+)$")


@dataclass(frozen=True)
class MissionTask:
    """One delegable unit of a mission, in the coordinator's vocabulary.

    Everything here fits in a TASK_ANNOUNCE. ``xml`` is the subtree itself,
    serialised, which is what makes a won task executable: the winner grafts it
    into its own plan rather than trying to reconstruct it from the wire fields.
    """

    task_id: int
    name: str
    #: Mask of every Capability the subtree's action leaves use.
    capabilities: int
    target: Target
    priority: int = DEFAULT_PRIORITY
    #: The subtree, serialised. Empty when the task was rebuilt from the wire
    #: by a robot that has not been sent the XML.
    xml: str = ""

    @property
    def delegable(self) -> bool:
        """Whether another robot could be sent to do this."""
        return self.target.placed

    def as_payload(self) -> dict:
        """The flat form that crosses ``/coordination/infeasible``.

        Flat because the coordinator's ``Task`` is flat, and because a
        coordinator that received nested structure would be a coordinator that
        had to understand it.
        """
        return {
            "task_id": int(self.task_id),
            "name": self.name,
            "capabilities": int(self.capabilities),
            "target_kind": int(self.target.kind),
            "target_a": int(self.target.a),
            "target_b": int(self.target.b),
            "priority": int(self.priority),
        }


# --------------------------------------------------------------------------
# Reading a mission
# --------------------------------------------------------------------------


def parse(mission_xml) -> Optional[etree._Element]:
    """Parse a mission, returning None rather than raising on bad XML.

    None because every caller here is downstream of something that already
    failed -- a fault report, a partial plan -- and adding an exception to that
    situation replaces a diagnosable "could not resolve the task" with a
    traceback about the wrong thing.
    """
    if not mission_xml:
        return None
    if isinstance(mission_xml, bytes):
        data = mission_xml
    elif isinstance(mission_xml, str):
        data = mission_xml.encode()
    else:
        return mission_xml
    try:
        return etree.fromstring(data)
    except etree.XMLSyntaxError:
        return None


def tasks_in(mission_xml) -> "List[MissionTask]":
    """Every delegable unit in a mission, in plan order.

    The outermost control nodes that contain work, not every control node: a
    ``RetryUntilSuccessful`` wrapping a ``Sequence`` is one task with a retry
    policy, not two nested ones. Nesting deeper would let the same work be
    announced twice at different granularities.
    """
    root = parse(mission_xml)
    if root is None:
        return []
    return [task for task, _ in _located(root)]


def _located(root):
    """(task, elements) for every unit, so an edit knows what to cut out.

    Removal has to work on the same elements extraction produced, which is why
    this is the one traversal and ``tasks_in`` is a projection of it. A second
    traversal that re-derived the units would agree with this one right up
    until they did not, and the symptom would be a shed task that quietly
    stayed in the plan.
    """
    for tree in root.iter("BehaviorTree"):
        for node in tree:
            yield from _located_under(node)


def _located_under(node):
    """Split one control node into (task, elements) pairs.

    A top-level ``Sequence`` holding a whole mission is not one task -- shedding
    it would shed the mission. Two ways it comes apart:

    * **Structurally**, when its children are themselves control nodes. Each is
      a unit; ``reactive_fallbacks.xml`` is written this way, one
      ``RetryUntilSuccessful`` per tree.
    * **By objective**, when it is a flat run of action leaves.
      ``sample_leafs.xml`` is written this way, and it is the common case,
      because that is what the mission planner emits.

    * **By prerequisite**, when leaves and controls sit side by side. This used
      to be taken whole, on the grounds that deciding where a loose leaf
      belongs relative to a subtree was a planning judgement with no basis --
      and it is the shape the mission planner now emits, so a whole mission
      read back as one undelegable task. ``mission.ontology`` supplies the
      basis: see ``_split_by_prerequisite``.

    A node whose ``name`` declares a task id is one unit however it is
    written. A plan that numbered something has said where its boundaries are.
    """
    if node.tag not in CONTROL_TAGS:
        return

    children = [child for child in node if isinstance(child.tag, str)]
    controls = [child for child in children if child.tag in CONTROL_TAGS]
    leaves = [child for child in children if child.tag in CAPABILITY_BY_ELEMENT]

    if controls and not leaves:
        for child in controls:
            yield from _located_under(child)
        return

    if leaves and not controls and declared_task_id(node.get("name")) is None:
        groups = _split_by_objective(leaves)
        if len(groups) > 1:
            for group in groups:
                task = _task_of_group(node, group)
                if task is not None:
                    yield task, group
            return

    if leaves and controls and declared_task_id(node.get("name")) is None:
        yield from _split_by_prerequisite(node, children)
        return

    task = task_of(node)
    if task is not None:
        yield task, [node]


def _split_by_prerequisite(node, children):
    """Attach each loose leaf to the unit it serves. The mixed case.

    This is the shape the mission planner now emits, and the one that used to
    defeat every rule here::

        <MoveToAisleHead id="6"/>                     <- loose leaf
        <RetryUntilSuccessful num_attempts="3">       <- control node
          <Sequence><MoveToTreeID id="60" .../><SampleLeaf/></Sequence>
        </RetryUntilSuccessful>
        <MoveToAisleHead id="6"/>                     <- loose leaf

    Leaves and controls side by side, so the node was taken whole -- the entire
    mission read back as one task, and nothing in it could be offered to
    anybody. Splitting it needs a reason to say where a loose leaf belongs, and
    ``mission.ontology`` is that reason: entering aisle 6 is what reaching tree
    60 expects, so the two are one unit and the aisle move goes with the tree
    when the tree is handed away.

    Two directions, and each says something different:

    * **Forward**, when a later unit needs what the leaf establishes. A
      traverse is how the robot reaches the next tree, not something it does
      after finishing the last one -- the same rule ``_split_by_objective``
      already applies to a flat sequence.
    * **Backward**, when nothing ahead needs it. Driving back out to the aisle
      head serves the tree just visited, so it leaves with it.

    A leaf serving nothing in either direction is not dropped: it joins the
    nearest unit, because a task extraction that quietly lost steps would shed
    a task and leave the plan short of actions nobody removed on purpose.
    """
    units: "List[list]" = []  # element lists, in plan order
    pending: list = []  # loose leaves not yet attached

    for child in children:
        if child.tag in CONTROL_TAGS:
            found = [elements for _, elements in _located_under(child)]
            if found:
                claimed, unclaimed = _claim(pending, _needs_of(found[0]))
                # Whatever the first unit did not want belongs to the previous
                # one; with no previous one it is a unit of its own.
                if unclaimed:
                    if units:
                        units[-1].extend(unclaimed)
                    else:
                        units.append(unclaimed)
                found[0] = claimed + found[0]
                units.extend(found)
                pending = []
                continue
        pending.append(child)

    if pending:
        if units:
            units[-1].extend(pending)
        else:
            units.append(pending)

    built = [
        (_task_of_group(node, elements), elements)
        for elements in units
        if _task_of_group(node, elements) is not None
    ]
    for index, (task, elements) in enumerate(built):
        later = [rest for _, rest in built[index + 1 :]]
        shared = _shared_prerequisites(elements, later)
        # The task describes the whole unit; the *cut* leaves behind whatever
        # something still on this robot has no other way to get. Handing tree 10
        # away must not take the aisle move tree 12 was relying on with it.
        yield task, [
            element for element in elements if not any(element is s for s in shared)
        ]


def _shared_prerequisites(elements, later_units) -> list:
    """Leaves in this unit that a later unit depends on and cannot replace.

    A plan that enters an aisle once and works two trees in it has one
    ``MoveToAisleHead`` serving both. It belongs to the first unit -- that is
    where it sits and what it leads to -- but cutting it when that unit is
    handed away would leave the second tree with no way in, in a plan nobody
    edited to say so.

    Only the *first* action of a later unit is asked, because that is the only
    precondition a unit has to get from outside itself; a unit that opens with
    its own aisle move needs nothing from here.
    """
    keep = []
    for element in elements:
        if element.tag not in CAPABILITY_BY_ELEMENT:
            continue
        step, _ = ontology.advance(ontology.State(), element)
        if not step.establishes:
            continue
        for later in later_units:
            needs = _needs_of(later)
            if any(need.met_by(fact) for need in needs for fact in step.establishes):
                keep.append(element)
                break
    return keep


def _needs_of(elements) -> tuple:
    """What the first action of a unit expects to be true before it runs.

    The first action, not all of them: a unit establishes its own later
    preconditions internally -- reaching the tree is what its SampleLeaf needs
    -- and only the way *in* has to come from outside.
    """
    for element in elements:
        for action in ontology.actions_in(element):
            step, _ = ontology.advance(ontology.State(), action)
            return step.needs
    return ()


def _claim(pending, needs) -> "tuple":
    """Split ``pending`` into (what this unit needs, what it does not).

    Searched from the end, so the nearest establisher wins. A plan that leaves
    aisle 10 and then enters aisle 6 has two ``in_aisle`` facts available and
    the second one is the prerequisite; the first is how the robot got out of
    where it was. Without an orchard the ontology cannot say *which* aisle a
    tree wants, and nearest is then the only defensible answer -- which is also
    why no caller here passes one: two readers of the same plan must agree
    about its units, and one of them holding a map the other lacks would make
    the same mission split two different ways.
    """
    keep: set = set()
    for need in needs:
        for index in range(len(pending) - 1, -1, -1):
            step, _ = ontology.advance(ontology.State(), pending[index])
            if any(need.met_by(fact) for fact in step.establishes):
                keep.add(index)
                break
    if not keep:
        return [], list(pending)
    # Everything from the earliest claimed leaf onward comes too: it sits
    # between the prerequisite and the work, so it is part of getting there.
    start = min(keep)
    return list(pending[start:]), list(pending[:start])


def _split_by_objective(leaves) -> "List[list]":
    """Cut a flat run of leaves into units, one per objective.

    A unit is *the approach and the work*: the movement that gets the robot
    somewhere, plus everything it does once there. So a group closes when a new
    movement begins after the current group already has its objective --
    transit moves attach **forward**, to the objective they lead to, because a
    traverse is how the robot reaches the next tree rather than something it
    does after finishing the last one.

    On ``sample_leafs.xml`` that yields exactly what a person would draw:

        [Visit_Tree_10, Sample_Leaves_Tree_10]
        [Exit_To_Top_Headland, Traverse_To_Col_4, Visit_Tree_60, Sample_60]
    """
    groups: "List[list]" = []
    current: list = []
    has_objective = False
    for leaf in leaves:
        moves = leaf.tag in MOVEMENT_ACTIONS
        if moves and has_objective:
            groups.append(current)
            current = []
            has_objective = False
        current.append(leaf)
        if not has_objective and _target_of_action(leaf) is not None:
            has_objective = True
    if current:
        groups.append(current)
    return groups


def _task_of_group(parent, leaves) -> Optional[MissionTask]:
    """One unit of a split sequence, as a subtree of its own.

    The group is rebuilt as a ``Sequence`` rather than as a slice of the
    original, because what leaves here has to be a plan on its own: the winner
    grafts this XML into its mission, and a bare list of leaves is not
    something the schema accepts.

    The name always comes from the group's own leaves, never from ``parent``'s
    descriptive name -- even when this group happens to be the only one and so
    spans everything ``parent`` has. Falling back to the parent's name there
    used to make a task's id depend on how much of the mission was left
    standing around it: shedding a sibling task could reduce a mission to this
    group alone, which reads identically to a mission that only ever had one
    unit, and the surviving task's id would jump to match. A unit's identity
    has to survive its sibling being given away.
    """
    if not leaves:
        return None
    group = etree.Element("Sequence")
    for leaf in leaves:
        group.append(deepcopy(leaf))
    name = _name_of(group)
    group.set("name", name or "task")
    return MissionTask(
        task_id=_task_id_of(group, name),
        name=task_label(name),
        capabilities=_capabilities_of(group),
        target=_target_of(group),
        priority=_priority_of(parent),
        xml=etree.tostring(group, encoding="unicode").strip(),
    )


def task_of(node) -> Optional[MissionTask]:
    """Turn one control node into a task, or None if it contains no actions."""
    capabilities = _capabilities_of(node)
    if not capabilities:
        return None
    name = _name_of(node)
    return MissionTask(
        task_id=_task_id_of(node, name),
        # The label, not the declaration: the id is already a field of its own,
        # and a name that repeated it would put "Tree60Seq__task_5" in every log
        # line and every operator display for the rest of the task's life. The
        # XML keeps the declared form -- that is what makes it readable back.
        name=task_label(name),
        capabilities=capabilities,
        target=_target_of(node),
        priority=_priority_of(node),
        xml=etree.tostring(node, encoding="unicode").strip(),
    )


def task_for_node(mission_xml, node_name: str) -> Optional[MissionTask]:
    """The task containing the behaviour-tree node called ``node_name``.

    This is the bridge the fault report crosses. ``/bt/status_change`` reports
    which *leaf* failed; what the fleet can be offered is the unit that leaf
    belongs to.

    Asked of ``_located`` rather than derived by walking up from the leaf, and
    that is not a detail. A flat mission -- which is what the mission planner
    emits, and what ``examples/sample_leafs.xml`` is -- has one Sequence holding
    every leaf, so walking up lands on the whole mission and then reports its
    *first* unit. A live run of that file resolved a failure at tree 60 to the
    task for tree 10: the same enclosing node, the same answer, the wrong tree.
    Going through the one traversal that also does the splitting is what makes
    the lookup agree with the announcement.
    """
    root = parse(mission_xml)
    if root is None or not node_name:
        return None
    target = None
    for element in root.iter():
        if element.get("name") == node_name:
            target = element
            break
    if target is None:
        return None

    # Held as a live list, not as a set of ids. lxml creates element proxies on
    # demand and only guarantees the same proxy back while a Python reference
    # to it exists -- so a set of ids from a throwaway generator matches
    # nothing, because the proxies it measured have already been collected and
    # the traversal below builds fresh ones for the same nodes.
    ancestors = [target, *target.iterancestors()]
    for task, elements in _located(root):
        if any(any(element is node for node in ancestors) for element in elements):
            return task
    return None


# --------------------------------------------------------------------------
# Editing a mission
# --------------------------------------------------------------------------


def remove_task(mission_xml, task_id: int) -> Optional[str]:
    """The mission without ``task_id``, or None if it is not in there.

    For a task transferred away or dropped. Returns a new document rather than
    mutating, because the caller is usually holding the plan that is currently
    running and must not have it change underneath the tree.

    A unit that came out of a flat sequence is several sibling leaves rather
    than one node, so removal cuts all of them -- taking only the objective
    would leave the sample behind, stranded with nothing to sample.
    """
    root = parse(mission_xml)
    if root is None:
        return None
    for task, elements in _located(root):
        if task.task_id != int(task_id):
            continue
        for element in elements:
            parent = element.getparent()
            if parent is not None:
                parent.remove(element)
                _prune_empty(parent)
        return etree.tostring(root, encoding="unicode")
    return None


def _prune_empty(node) -> None:
    """Drop the control nodes a removal emptied, upward from ``node``.

    A task is often the only thing inside its wrapper -- ``RetryUntilSuccessful``
    around one ``Sequence`` is how the planner writes a retried objective, and it
    is the shape that separates units cleanly in the first place. Taking the task
    out leaves the wrapper with nothing to run: the XSD requires a control node
    to have at least one child, and BT.CPP requires a decorator to have exactly
    one. So without this the *loser's* plan is what breaks, after the transfer
    has already been agreed and the winner is on its way.

    Stops at the behaviour tree, and never climbs into it. A mission whose last
    unit was shed has no valid form under the XSD -- a control node must hold at
    least one child -- so the document that comes out is going to be refused
    whatever this does. What it can decide is *how*: a plan the arbiter reads as
    "this mission no longer does anything", or one with no ``<BehaviorTree>`` in
    it at all, which it could only report as malformed.
    """
    while node is not None and node.tag != "BehaviorTree":
        parent = node.getparent()
        if parent is None or parent.tag == "BehaviorTree":
            return
        if any(isinstance(child.tag, str) for child in node):
            return
        parent.remove(node)
        node = parent


def insert_task(mission_xml, task: MissionTask) -> Optional[str]:
    """The mission with ``task``'s subtree appended to the main tree.

    For a task won at auction. Appended rather than inserted at a computed
    position: choosing *where* in the plan the new work belongs is a planning
    decision, and this module has no business making it. What comes out of here
    is a candidate -- it goes to the arbiter, which validates it against the
    XSD and its own semantic checks exactly as it would any other edit.
    """
    root = parse(mission_xml)
    if root is None or not task.xml:
        return None
    subtree = parse(task.xml)
    if subtree is None:
        return None
    trees = list(root.iter("BehaviorTree"))
    if not trees:
        return None
    container = trees[0]
    # Into the root control node, not beside it: the XSD allows a BehaviorTree
    # exactly one child.
    root_control = next(
        (child for child in container if child.tag in CONTROL_TAGS), None
    )
    target = root_control if root_control is not None else container
    _isolate_leaves(target)
    target.append(subtree)
    return etree.tostring(root, encoding="unicode")


def _isolate_leaves(container) -> None:
    """Wrap a flat run of action leaves so an appended subtree stays visible.

    Without this, appending to a mission written as one ``Sequence`` of leaves
    -- the common form, and the one the mission planner emits -- produces a
    control node with leaves and a subtree side by side. ``_located_under``
    takes a genuinely mixed node *whole*, deliberately, because splitting it
    would mean deciding where a loose leaf belongs relative to a subtree. The
    consequence is that the whole mission reads back as one task and the task
    just inserted cannot be found or removed again.

    Wrapping sidesteps the judgement rather than making it: the leaves that were
    already a unit stay one, in the order they were in, and the container
    becomes all-controls, which is the case that splits cleanly. The wrapper
    carries no ``name``, so it declares no id and the units inside it keep the
    identities they had -- ids are derived from the leaves' own names.
    """
    children = [child for child in container if isinstance(child.tag, str)]
    if not any(child.tag in CAPABILITY_BY_ELEMENT for child in children):
        return
    wrapper = etree.SubElement(container, "Sequence")
    for child in children:
        container.remove(child)
        wrapper.append(child)


# --------------------------------------------------------------------------
# Field extraction
# --------------------------------------------------------------------------


def _capabilities_of(node) -> int:
    """Mask of every action type anywhere under ``node``.

    The whole subtree, not the immediate children: a task wrapped in a
    ``RetryUntilSuccessful`` still needs every action its inner sequence uses.
    """
    return cap_mask(
        *{
            CAPABILITY_BY_ELEMENT[element.tag]
            for element in node.iter()
            if element.tag in CAPABILITY_BY_ELEMENT
        }
    )


def _target_of(node) -> Target:
    """Where the subtree's work happens.

    ``MoveToTreeID`` with ``approach_tree="true"`` is the objective -- that is
    the arbiter's own definition, and using a different one here would mean the
    two disagreed about which trees a mission is actually for. A transit move
    (``approach_tree="false"``) is how the robot gets somewhere, not what it is
    there to do.
    """
    for tag in TARGET_ACTIONS:
        for element in node.iter(tag):
            target = _target_of_action(element)
            if target is not None:
                return target
    return Target.none()


def _target_of_action(element) -> Optional[Target]:
    if element.tag == "MoveToTreeID":
        if (element.get("approach_tree") or "").strip().lower() != "true":
            return None
        return _index_target(element.get("id"), Target.tree)
    if element.tag == "MoveToAisleHead":
        return _index_target(element.get("id"), Target.aisle)
    lat, lon = element.get("latitude"), element.get("longitude")
    if lat is None or lon is None:
        return None
    try:
        return Target.gps(float(lat), float(lon))
    except (TypeError, ValueError):
        # An unparseable or off-globe coordinate is not a place. Treated as no
        # target rather than a guessed one: an announcement naming somewhere
        # invented is worse than one the coordinator refuses to make.
        return None


def _index_target(raw, build) -> Optional[Target]:
    try:
        return build(int(raw))
    except (TypeError, ValueError):
        return None


def _name_of(node) -> str:
    """The subtree's name: its own, else its objective's, else its first leaf's.

    Falling back this way is what makes the feature work against plans written
    before the schema carried ``name`` on a control node -- and the fallback is
    stable, because it is derived from the plan rather than from position.
    """
    own = node.get("name")
    if own:
        return own
    # The objective's name, not merely the first movement's: a unit that
    # traverses two headlands to reach tree 60 is about tree 60, and naming it
    # after the traverse would put the wrong thing in every log line and every
    # operator display for the rest of the task's life.
    for tag in TARGET_ACTIONS:
        for element in node.iter(tag):
            if _target_of_action(element) is not None and element.get("name"):
                return element.get("name")
    for element in node.iter():
        if element.tag in CAPABILITY_BY_ELEMENT and element.get("name"):
            return element.get("name")
    return ""


def _split_declaration(name) -> "tuple":
    """``(label, task_id)`` for a name, with ``task_id`` None when undeclared.

    An out-of-range number is not a declaration: 0 is the wire's "no task" and
    anything above TASK_ID_MAX does not survive a TASK_ANNOUNCE, so the name is
    taken as an ordinary name and the id is derived instead. Refusing the plan
    outright would be worse -- the id is bookkeeping, and a mission is still
    runnable without one.
    """
    match = _DECLARED_ID.match(name or "")
    if match is None:
        return (name or ""), None
    value = int(match.group("id"))
    if not 0 < value <= TASK_ID_MAX:
        return (name or ""), None
    return (match.group("label") or ""), value


def declared_task_id(name) -> Optional[int]:
    """The fleet id a control node's ``name`` declares, or None."""
    return _split_declaration(name)[1]


def task_label(name) -> str:
    """A name with its id declaration taken off, for logs and for the wire.

    A bare ``task_5`` has nothing left underneath, so it keeps its whole name --
    an empty one in an operator display would be worse than a redundant one.
    """
    return _split_declaration(name)[0] or (name or "")


def name_with_task_id(label, task_id: int) -> str:
    """``label`` carrying ``task_id``, ready to put on a ``name`` attribute.

    The inverse of ``task_label``, and the thing that closes the round trip: a
    subtree rebuilt from an announcement has to read back as the same task the
    announcement named, or the winner cannot later drop or re-offer it.
    """
    base = _ident(_split_declaration(_ident(label))[0])
    return f"{base}{TASK_ID_SEPARATOR}{int(task_id)}" if base else f"task_{task_id}"


def _task_id_of(node, name: str) -> int:
    """The subtree's fleet-unique id.

    The id declared in the node's own ``name`` when the plan declares one --
    its *own*, never ``_name_of``'s fallback, which reaches down into the
    leaves and would let a leaf happening to be called ``task_3`` rename the
    unit around it. Otherwise a hash of the *names* of the subtree's leaves --
    of the plan's own words, so the same work in the same plan always gets the
    same id, and two robots reading the same mission agree without talking
    about it.

    Never the tree ``id``. That is a location: it repeats whenever a plan visits
    a tree twice, and it is absent entirely from work like ``SampleLeaf``, so a
    fleet that used it would collide on the first and drop the second.
    """
    declared = declared_task_id(node.get("name"))
    if declared is not None:
        return declared

    material = name or " ".join(
        element.get("name") or element.tag
        for element in node.iter()
        if element.tag in CAPABILITY_BY_ELEMENT
    )
    if not material:
        return 0
    # crc32 folded into the wire's range. 0 is TASK_NONE, so it is mapped off:
    # a task whose id means "no task" would be announced and never matched.
    return (zlib.crc32(material.encode()) % TASK_ID_MAX) + 1


def _priority_of(node) -> int:
    raw = node.get("priority")
    if raw is None:
        return DEFAULT_PRIORITY
    try:
        value = int(raw)
    except ValueError:
        return DEFAULT_PRIORITY
    return value if 0 <= value <= PRIORITY_MAX else DEFAULT_PRIORITY


def capability_names(mask: int) -> "List[str]":
    """Element names for a mask, for logs and prompts."""
    from amiga_ros2_comms.codec import XML_ELEMENT, capabilities_in

    return [
        XML_ELEMENT.get(Capability(c), f"CAP_{int(c)}") for c in capabilities_in(mask)
    ]


# --------------------------------------------------------------------------
# Rebuilding a task the radio described
# --------------------------------------------------------------------------


def action_names(xsd_path: str) -> "dict":
    """Element name -> its ``action_name``, read out of the schema.

    Every action type fixes this attribute, because it is the ROS action server
    the leaf dispatches to. Read rather than restated: a wrong ``action_name``
    produces a plan that validates and then fails at run time against a server
    that does not exist, which is the worst place to find out.
    """
    doc = etree.parse(xsd_path)
    ns = {"xs": "http://www.w3.org/2001/XMLSchema"}
    out = {}
    for ctype in doc.findall(".//xs:complexType[@name]", namespaces=ns):
        attr = ctype.find("xs:attribute[@name='action_name']", namespaces=ns)
        if attr is None or attr.get("fixed") is None:
            continue
        # "moveToTreeIDType" -> "MoveToTreeID"
        element = ctype.get("name")[:-4]
        out[element[0].upper() + element[1:]] = attr.get("fixed")
    return out


#: What ``synthesize`` can put back into a plan from the wire fields alone.
#: Everything else in the vocabulary needs a parameter the announcement does not
#: carry -- an arm pose, an offset, a heading -- and inventing one would be
#: worse than saying so.
REBUILDABLE = frozenset(
    {
        Capability.MOVE_TO_AISLE_HEAD,
        Capability.MOVE_TO_TREE_ID,
        Capability.SAMPLE_LEAF,
    }
)


@dataclass(frozen=True)
class Rebuilt:
    """A task reconstructed from an announcement, and what got left out."""

    xml: str
    #: Element names the announcement asked for and this could not rebuild, in
    #: schema order. Not an error: the floor is still runnable work, and naming
    #: what is missing is what lets a planner fill it in.
    dropped: "List[str]" = field(default_factory=list)


def action_grammar(xsd_path: str) -> "List[dict]":
    """The action vocabulary with its attributes, as ``{tag, attrs}`` rows.

    For the mission planner's prompt, which until now carried a hand-written
    two-element list -- ``MoveToTreeID`` and ``SampleLeaf``. ``MoveToAisleHead``
    was not in it, so the replanner could not emit an aisle move even when the
    plan it was editing was full of them, and every replan quietly flattened the
    prerequisite chain. That is exactly the drift ``promela.action_pool``'s
    docstring was written about, in a second place.

    Read out of the schema for the same reason ``action_names`` is: a list that
    cannot drift cannot describe a grammar the validator will then reject.
    """
    doc = etree.parse(xsd_path)
    ns = {"xs": "http://www.w3.org/2001/XMLSchema"}
    group = doc.find(".//xs:group[@name='ActionGroup']", namespaces=ns)
    if group is None:
        return []
    rows = []
    for element in group.findall(".//xs:element", namespaces=ns):
        tag = element.get("name")
        ctype = doc.find(
            f".//xs:complexType[@name='{element.get('type')}']", namespaces=ns
        )
        if not tag or ctype is None:
            continue
        rows.append({"tag": tag, "attrs": _attrs_of(ctype, doc, ns)})
    return rows


def _attrs_of(ctype, doc, ns) -> str:
    """One complexType's attributes, in the form a prompt can be handed."""
    out = []
    for attr in ctype.findall("xs:attribute", namespaces=ns):
        name = attr.get("name")
        if attr.get("fixed") is not None:
            out.append(f'{name}="{attr.get("fixed")}"')
            continue
        kind = (attr.get("type") or "string").replace("xs:", "")
        values = doc.findall(
            f".//xs:simpleType[@name='{kind}']//xs:enumeration", namespaces=ns
        )
        if values:
            kind = "|".join(v.get("value") for v in values)
        optional = "" if attr.get("use") == "required" else ", optional"
        out.append(f"{name} ({kind}{optional})")
    return ", ".join(out)


def synthesize(task: MissionTask, xsd_path: str, orchard=None) -> Optional[Rebuilt]:
    """Rebuild an executable subtree from the wire fields alone.

    The winner of an auction has the announcement, not the XML: TASK_ANNOUNCE
    carries a capability mask and a target in 7 bytes and the subtree does not
    fit. What comes back is a **floor** -- the objective plus the prerequisite
    chain ``mission.ontology`` says it expects -- built from the *winner's* own
    map rather than from whatever the announcer's plan happened to contain.
    That is what "announce the goal, let the winner plan it" means in practice:
    the loser's route out of its own aisle is not the winner's route in.

    So ``sample tree 60`` with an orchard becomes::

        <MoveToAisleHead id="6"/>          <- the expected prerequisite
        <MoveToTreeID id="60" approach_tree="true"/>
        <SampleLeaf/>

    and without one, the same thing minus the aisle move -- which still runs,
    because the expectation is soft and navigation routes there regardless.

    Returns None only when the *objective* cannot be built: a GPS target the
    announcement rounded, or a mask with no ``MoveToTreeID`` in it. A capability
    outside ``REBUILDABLE`` is reported in ``dropped`` instead, because refusing
    the whole task over an arm pose would make every unit that contains one
    untradeable -- and those are exactly the complex units worth trading.

    Prefer ``task.xml`` when it is populated; this is the fallback, and what it
    builds is a candidate like any other -- it goes to the arbiter, and then to
    the robot's own planner to be elaborated.
    """
    from amiga_ros2_comms.codec import XML_ELEMENT, capabilities_in

    if task.target.kind != TargetKind.TREE:
        return None

    caps = {Capability(c) for c in capabilities_in(task.capabilities)}
    if Capability.MOVE_TO_TREE_ID not in caps:
        return None

    names = action_names(xsd_path)
    tree_id = int(task.target.a)
    safe = _ident(task.name) or f"task_{int(task.task_id)}"
    aisle = orchard.aisle_of(tree_id) if orchard is not None else None

    root = etree.Element("Sequence")
    # The declared form here and the plain label on the leaves below: the unit
    # is what carries the identity, and repeating it on every action would make
    # the names longer without making anything findable.
    root.set("name", name_with_task_id(safe, task.task_id))

    if aisle is not None:
        head = etree.SubElement(root, "MoveToAisleHead")
        head.set("name", f"{safe}_aisle_{aisle}")
        head.set("action_name", names.get("MoveToAisleHead", "move_to_aisle_head"))
        head.set("id", str(aisle))

    move = etree.SubElement(root, "MoveToTreeID")
    move.set("name", f"{safe}_visit_{tree_id}")
    move.set("action_name", names.get("MoveToTreeID", "follow_tree_id_waypoint"))
    move.set("id", str(tree_id))
    # An objective, not a transit move: this task was announced because work has
    # to happen *at* that tree. approach_tree="false" would satisfy the schema
    # and quietly stop the plan establishing at_tree_<id>, so the mission it was
    # absorbed into would then fail verification for a reason nobody could see.
    move.set("approach_tree", "true")

    if Capability.SAMPLE_LEAF in caps:
        sample = etree.SubElement(root, "SampleLeaf")
        sample.set("name", f"{safe}_sample_{tree_id}")
        sample.set("action_name", names.get("SampleLeaf", "segment_leaves"))

    dropped = [XML_ELEMENT[cap] for cap in sorted(caps - REBUILDABLE)]
    # The aisle move was asked for and could not be placed: an unknown aisle is
    # a step the winner's planner has to supply, not one nobody wanted.
    if Capability.MOVE_TO_AISLE_HEAD in caps and aisle is None:
        dropped.append(XML_ELEMENT[Capability.MOVE_TO_AISLE_HEAD])
    return Rebuilt(etree.tostring(root, encoding="unicode"), dropped)


def _ident(text: str) -> str:
    """Reduce a name to the XSD's taskNameType ([A-Za-z0-9_]+)."""
    return re.sub(r"[^A-Za-z0-9_]", "_", text or "").strip("_")
