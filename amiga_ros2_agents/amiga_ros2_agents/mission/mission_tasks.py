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
    identity  the ``task_id`` attribute if the plan carries one, else a hash of
              the subtree's name -- never the tree ``id``, which is a location
              and collides (tree 2 is visited twice in that same file)

**What is deliberately undelegable.** A subtree whose only movement is
``MoveToRelativeLocation`` gets ``Target.none()``. An offset from *this* robot's
pose names nowhere another robot can be sent, and the coordinator refuses to
announce it rather than announcing a place that means something different to
every listener.

No ROS. lxml and the codec vocabulary, nothing else, so this is testable against
the real files in ``examples/`` with no node running.
"""

import zlib
from copy import deepcopy
from dataclasses import dataclass
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

    A control node whose children are *actually mixed* -- leaves and controls
    side by side -- is taken whole. Splitting it would mean deciding where a
    loose leaf belongs relative to a subtree, and that is a planning judgement
    this module has no basis to make.
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

    if leaves and not controls and node.get("task_id") is None:
        groups = _split_by_objective(leaves)
        if len(groups) > 1:
            for group in groups:
                task = _task_of_group(node, group)
                if task is not None:
                    yield task, group
            return

    task = task_of(node)
    if task is not None:
        yield task, [node]


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
    """
    if not leaves:
        return None
    group = etree.Element("Sequence")
    for leaf in leaves:
        group.append(deepcopy(leaf))
    name = _name_of(group)
    if parent.get("name") and len(leaves) == len(list(parent)):
        name = parent.get("name")
    group.set("name", name or "task")
    return MissionTask(
        task_id=_task_id_of(group, name),
        name=name,
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
        name=name,
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
        return etree.tostring(root, encoding="unicode")
    return None


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
    carries no ``name`` and no ``task_id``, so the units inside it keep the
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


def _task_id_of(node, name: str) -> int:
    """The subtree's fleet-unique id.

    The ``task_id`` attribute when the plan declares one. Otherwise a hash of
    the *names* of the subtree's leaves -- of the plan's own words, so the same
    work in the same plan always gets the same id, and two robots reading the
    same mission agree without talking about it.

    Never the tree ``id``. That is a location: it repeats whenever a plan visits
    a tree twice, and it is absent entirely from work like ``SampleLeaf``, so a
    fleet that used it would collide on the first and drop the second.
    """
    declared = node.get("task_id")
    if declared is not None:
        try:
            value = int(declared)
        except ValueError:
            value = 0
        if 0 < value <= TASK_ID_MAX:
            return value

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


def synthesize(task: MissionTask, xsd_path: str) -> Optional[str]:
    """Rebuild an executable subtree from the wire fields alone.

    The winner of an auction has the announcement, not the XML: TASK_ANNOUNCE
    carries a capability mask and a target in 7 bytes and the subtree does not
    fit. For the shapes this fleet actually trades that is enough to reconstruct
    the work, because the target says where and the mask says what.

    Returns None when it is not enough, which is the honest answer rather than a
    guess. A GPS-targeted task needs two coordinates the announcement rounds,
    and ``MoveToRelativeLocation`` names an offset from *someone else's* pose --
    ``Target.none()`` already refuses to announce those, so arriving here with
    one means something upstream is wrong.

    Prefer ``task.xml`` when it is populated; this is the fallback, and the
    subtree it builds is a candidate like any other -- it goes to the arbiter,
    which validates it against the XSD and verifies it against the mission's
    formula before anything runs it.
    """
    from amiga_ros2_comms.codec import capabilities_in

    if task.target.kind != TargetKind.TREE:
        return None

    caps = {Capability(c) for c in capabilities_in(task.capabilities)}
    if Capability.MOVE_TO_TREE_ID not in caps:
        return None
    unsupported = caps - {Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF}
    if unsupported:
        return None

    names = action_names(xsd_path)
    tree_id = int(task.target.a)
    safe = _ident(task.name) or f"task_{int(task.task_id)}"

    root = etree.Element("Sequence")
    root.set("name", safe)
    root.set("task_id", str(int(task.task_id)))

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

    return etree.tostring(root, encoding="unicode")


def _ident(text: str) -> str:
    """Reduce a name to the XSD's taskNameType ([A-Za-z0-9_]+)."""
    import re

    return re.sub(r"[^A-Za-z0-9_]", "_", text or "").strip("_")
