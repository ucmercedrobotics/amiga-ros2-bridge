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

from amiga_ros2_agents.mission import mission_tasks  # noqa: E402
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
    xml = mission_tasks.synthesize(announced(caps=SAMPLING), SCHEMA)
    assert xml is not None

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
    xml = mission_tasks.synthesize(announced(tree=35, caps=SAMPLING), SCHEMA)
    wrapped = (
        '<root BTCPP_format="4" schema_location="s.xsd"><Mission>m</Mission>'
        f'<BehaviorTree ID="T">{xml}</BehaviorTree></root>'
    )
    model = promela.compile_mission(wrapped, SCHEMA)
    assert "at_tree_35" in model.defined_aps
    assert "sampled_tree_35" in model.defined_aps


def test_move_only_task_samples_nothing():
    xml = mission_tasks.synthesize(announced(tree=12), SCHEMA)
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
    rebuilt = mission_tasks.synthesize(stripped, SCHEMA)
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


def test_unsupported_capability_is_refused():
    task = announced(caps=(Capability.MOVE_TO_TREE_ID, Capability.FOLLOW_PERSON))
    assert mission_tasks.synthesize(task, SCHEMA) is None


def _fields(task) -> dict:
    return {
        "task_id": task.task_id,
        "name": task.name,
        "capabilities": task.capabilities,
        "target": task.target,
        "priority": task.priority,
    }
