"""Rebuilding a won task from its announcement.

The winner of an auction has 7 bytes of TASK_ANNOUNCE, not a subtree. This is
where those bytes become executable behaviour-tree XML again, so the round trip
-- extract a task from a real mission, reduce it to what the radio carries,
rebuild it, graft it back, validate against the schema the runner uses -- is the
evidence that a transferred task is something the winning robot can actually do.

The naming matters as much as the structure: the rebuilt subtree has to
establish the same propositions the original did, or the mission it is absorbed
into fails verification for a reason nobody can see.
"""

import os
import sys

import pytest
from lxml import etree

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_agents.mission import mission_tasks, ontology, orchard  # noqa: E402
from amiga_ros2_agents.verification import promela  # noqa: E402
from amiga_ros2_comms.codec import Capability, Target, cap_mask  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
EXAMPLES = os.path.join(REPO, "amiga_ros2_behavior_tree", "examples")
SCHEMA = os.path.join(REPO, "amiga_ros2_behavior_tree", "schemas", "amiga_btcpp.xsd")


@pytest.fixture(scope="module")
def schema():
    with open(SCHEMA, "rb") as handle:
        return etree.XMLSchema(etree.parse(handle))


def announced(task_id=7, tree=35, caps=(Capability.MOVE_TO_TREE_ID,)):
    """A task as it survives the radio: fields only, no XML."""
    return mission_tasks.MissionTask(
        task_id=task_id,
        name=f"task_{task_id}",
        capabilities=cap_mask(*caps),
        target=Target.tree(tree),
    )


SAMPLING = (Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF)


def test_action_names_come_from_the_schema():
    """Fixed in the XSD because it is the ROS action server the leaf calls.

    A wrong one validates and then fails at run time against a server that does
    not exist, which is the worst place to find out.
    """
    names = mission_tasks.action_names(SCHEMA)
    assert names["MoveToTreeID"] == "follow_tree_id_waypoint"
    assert names["SampleLeaf"] == "segment_leaves"


def test_rebuilt_task_validates_against_the_schema(schema):
    """Grafted into a plan, the result is something bt_runner would accept."""
    rebuilt = mission_tasks.synthesize(announced(caps=SAMPLING), SCHEMA)
    assert rebuilt is not None
    xml = rebuilt.xml

    with open(os.path.join(EXAMPLES, "sample_leafs.xml")) as handle:
        host = handle.read()
    merged = mission_tasks.insert_task(
        host,
        mission_tasks.MissionTask(**{**_fields(announced(caps=SAMPLING)), "xml": xml}),
    )
    assert merged is not None
    assert schema.validate(etree.fromstring(merged.encode())), schema.error_log


def test_rebuilt_task_establishes_its_propositions():
    """The point of rebuilding it: the mission's formula can see the work.

    A subtree that navigated to the tree without ``approach_tree="true"`` would
    satisfy the schema and quietly establish nothing, so the absorbing robot's
    mission would fail verification with no visible cause.
    """
    xml = mission_tasks.synthesize(announced(tree=35, caps=SAMPLING), SCHEMA).xml
    wrapped = (
        '<root BTCPP_format="4" schema_location="s.xsd"><Mission>m</Mission>'
        f'<BehaviorTree ID="T">{xml}</BehaviorTree></root>'
    )
    model = promela.compile_mission(wrapped, SCHEMA)
    assert "at_tree_35" in model.defined_aps
    assert "sampled_tree_35" in model.defined_aps


def test_a_rebuilt_task_reads_back_as_the_task_it_was(schema):
    """The round trip that makes an absorbed task droppable again.

    A winner is granted task N, rebuilds it and grafts it in. If the grafted
    subtree read back under a *derived* id, the mission would hold work the
    robot could no longer name: ``remove_task(N)`` would find nothing, so the
    task could never be transferred on or given up, and the arbiter would refuse
    the edit with "task N is not in the active plan".

    The id survives in the ``name``, which is the only attribute BT.CPP will
    let a control node carry -- see ``mission_tasks.TASK_ID_SEPARATOR``.
    """
    task = announced(task_id=4210, tree=35, caps=SAMPLING)
    rebuilt = mission_tasks.synthesize(task, SCHEMA).xml
    assert "task_id=" not in rebuilt

    with open(os.path.join(EXAMPLES, "sample_leafs.xml")) as handle:
        host = handle.read()
    merged = mission_tasks.insert_task(
        host, mission_tasks.MissionTask(**{**_fields(task), "xml": rebuilt})
    )
    assert schema.validate(etree.fromstring(merged.encode())), schema.error_log
    assert 4210 in [found.task_id for found in mission_tasks.tasks_in(merged)]
    assert mission_tasks.remove_task(merged, 4210) is not None


def test_a_named_task_keeps_its_name_and_its_id(schema):
    """Both, because both are read: the name by a person, the id by the fleet."""
    task = mission_tasks.MissionTask(
        task_id=77,
        name="Visit_Tree_60",
        capabilities=cap_mask(*SAMPLING),
        target=Target.tree(60),
    )
    rebuilt = mission_tasks.synthesize(task, SCHEMA).xml
    root = etree.fromstring(rebuilt.encode())
    assert root.get("name") == "Visit_Tree_60__task_77"
    assert mission_tasks.declared_task_id(root.get("name")) == 77
    # The leaves keep the plain label: the unit carries the identity, and
    # repeating it on every action only makes the names longer.
    assert root[0].get("name") == "Visit_Tree_60_visit_60"


def test_a_rebuilt_task_carries_the_way_in(schema):
    """The point of the whole exercise: the winner gets the chain, not a leaf.

    Seven bytes of TASK_ANNOUNCE say *sample tree 60*. Sampling tree 60 is
    three actions -- enter aisle 6, approach the tree, sample it -- and the
    announcement cannot carry the first one, because which aisle serves tree 60
    is a fact about the orchard rather than about the task.

    So the winner supplies it from its **own** map. Not from the announcer's
    plan: the loser's route out of the aisle it was already in is not the
    winner's route in, which is what "announce the goal, let the winner plan
    it" means once it stops being a slogan.
    """
    rebuilt = mission_tasks.synthesize(
        announced(tree=60, caps=SAMPLING), SCHEMA, orchard=_orchard()
    )
    tags = [el.tag for el in etree.fromstring(rebuilt.xml.encode())]
    assert tags == ["MoveToAisleHead", "MoveToTreeID", "SampleLeaf"]
    assert not rebuilt.dropped

    aisle = etree.fromstring(rebuilt.xml.encode())[0]
    assert aisle.get("id") == "6", "tree 60 is in column 6 of the real orchard"
    assert aisle.get("action_name") == "move_to_aisle_head"


def test_without_a_map_the_task_is_still_runnable_and_says_what_is_missing():
    """The prerequisite is expected, not required, so its absence is not fatal.

    A robot that has not been handed the orchard can still do the work --
    navigation routes to a tree from wherever it is. What it cannot do is
    choose the lane, so that step is reported rather than guessed, and the
    planner is the thing that fills it in.
    """
    task = announced(
        tree=60,
        caps=(
            Capability.MOVE_TO_TREE_ID,
            Capability.MOVE_TO_AISLE_HEAD,
            Capability.SAMPLE_LEAF,
        ),
    )
    rebuilt = mission_tasks.synthesize(task, SCHEMA)

    tags = [el.tag for el in etree.fromstring(rebuilt.xml.encode())]
    assert tags == ["MoveToTreeID", "SampleLeaf"]
    assert rebuilt.dropped == ["MoveToAisleHead"]


def test_the_rebuilt_chain_breaks_no_precondition(schema):
    """Checked with the gate's own rule, not by reading the tags off."""
    rebuilt = mission_tasks.synthesize(
        announced(tree=60, caps=SAMPLING), SCHEMA, orchard=_orchard()
    )
    assert ontology.violations(etree.fromstring(rebuilt.xml.encode())) == []


def test_move_only_task_samples_nothing():
    xml = mission_tasks.synthesize(announced(tree=12), SCHEMA).xml
    assert "SampleLeaf" not in xml
    assert 'id="12"' in xml


def test_round_trip_from_a_real_mission(schema):
    """A task pulled out of a real plan, reduced to the wire, and rebuilt."""
    with open(os.path.join(EXAMPLES, "sample_leafs.xml")) as handle:
        original = handle.read()
    tasks = [t for t in mission_tasks.tasks_in(original) if t.delegable]
    assert tasks, "sample_leafs.xml should contain delegable work"

    task = tasks[0]
    stripped = mission_tasks.MissionTask(**{**_fields(task), "xml": ""})
    rebuilt = mission_tasks.synthesize(stripped, SCHEMA).xml
    assert rebuilt is not None
    assert schema.validate(
        etree.fromstring(
            (
                '<root BTCPP_format="4" schema_location="s.xsd">'
                "<Mission>m</Mission>"
                f'<BehaviorTree ID="T">{rebuilt}</BehaviorTree></root>'
            ).encode()
        )
    ), schema.error_log


# ---------------------------------------------------------------------------
# Refusals — an honest None beats a guess
# ---------------------------------------------------------------------------


def test_unplaced_task_is_refused():
    """SampleLeaf alone names nowhere; Target.none() already refuses to announce."""
    task = mission_tasks.MissionTask(
        task_id=3,
        name="t",
        capabilities=cap_mask(Capability.SAMPLE_LEAF),
        target=Target.none(),
    )
    assert mission_tasks.synthesize(task, SCHEMA) is None


def test_gps_task_is_refused():
    """The announcement rounds the coordinates; rebuilding would invent a place."""
    task = mission_tasks.MissionTask(
        task_id=4,
        name="t",
        capabilities=cap_mask(Capability.MOVE_TO_GPS_LOCATION),
        target=Target.gps(37.419321, -122.084401),
    )
    assert mission_tasks.synthesize(task, SCHEMA) is None


def test_an_unrebuildable_step_is_dropped_and_named_rather_than_refused():
    """The objective is buildable; one step in it is not. That is not a refusal.

    Refusing the whole task over a step the announcement has no parameters for
    -- an arm pose, a person to follow -- makes every unit containing one
    *untradeable*, and those are exactly the units worth trading. The floor
    still runs, and naming what is missing is what lets the robot's own planner
    put it back.
    """
    task = announced(caps=(Capability.MOVE_TO_TREE_ID, Capability.FOLLOW_PERSON))
    rebuilt = mission_tasks.synthesize(task, SCHEMA)

    assert rebuilt is not None
    assert rebuilt.dropped == ["FollowPerson"]
    assert "FollowPerson" not in rebuilt.xml
    assert 'id="35"' in rebuilt.xml


def _orchard():
    """The real orchard, from the second frame of a real mission binary."""
    import struct

    with open(os.path.join(EXAMPLES, "aisle_sample_10_60.bin"), "rb") as handle:
        data = handle.read()
    offset, frames = 0, []
    while offset + 4 <= len(data):
        (length,) = struct.unpack(">I", data[offset : offset + 4])
        offset += 4
        frames.append(data[offset : offset + length])
        offset += length
    return orchard.parse(frames[1].decode())


def _fields(task) -> dict:
    return {
        "task_id": task.task_id,
        "name": task.name,
        "capabilities": task.capabilities,
        "target": task.target,
        "priority": task.priority,
    }
