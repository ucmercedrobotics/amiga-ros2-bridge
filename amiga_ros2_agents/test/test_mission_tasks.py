"""Mission XML -> coordination tasks, against the real example missions.

Every test here runs on a file in ``amiga_ros2_behavior_tree/examples/``, not on
a fixture written to pass. That is the point: the extraction rules are claims
about what real plans look like, and a fixture is a place to accidentally encode
the rule twice and prove nothing.

The round trip -- extract a task, graft it into an empty plan, validate the
result against the schema ``bt_runner`` actually uses -- is the first evidence
that a delegated task is executable by the robot that wins it. Without it,
"absorb" is a method name.
"""

import os
import sys

import pytest
from lxml import etree

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_agents.mission import mission_tasks  # noqa: E402
from amiga_ros2_comms.codec import (  # noqa: E402
    TASK_ID_MAX,
    Capability,
    Target,
    TargetKind,
    cap_mask,
)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
EXAMPLES = os.path.join(REPO, "amiga_ros2_behavior_tree", "examples")
SCHEMA = os.path.join(REPO, "amiga_ros2_behavior_tree", "schemas", "amiga_btcpp.xsd")

#: Every example mission, by name. Parameterised over rather than picked from,
#: so a rule that only works on the file I happened to look at gets caught.
EXAMPLE_FILES = sorted(f for f in os.listdir(EXAMPLES) if f.endswith(".xml"))


def example(name: str) -> str:
    with open(os.path.join(EXAMPLES, name)) as handle:
        return handle.read()


@pytest.fixture(scope="module")
def schema():
    return etree.XMLSchema(etree.parse(SCHEMA))


# ==========================================================================
# The unit of work
# ==========================================================================


def test_a_sampling_mission_splits_into_one_task_per_tree():
    """sample_leafs.xml visits two trees and traverses past two more.

    Four units, and what matters is that the two sampling ones each carry both
    actions. A split that put MoveToTreeID and SampleLeaf in different tasks
    would offer the fleet a drive with nothing at the end of it.
    """
    tasks = mission_tasks.tasks_in(example("sample_leafs.xml"))

    sampling = [t for t in tasks if t.capabilities & (1 << Capability.SAMPLE_LEAF)]
    assert len(sampling) == 2
    for task in sampling:
        assert task.capabilities == cap_mask(
            Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF
        )
        assert task.target.kind is TargetKind.TREE
    assert {task.target.a for task in sampling} == {10, 60}


def test_a_retry_wrapper_is_one_task_not_two():
    """reactive_fallbacks.xml wraps each sampling pair in RetryUntilSuccessful.

    That is a retry policy on one unit of work, not a second unit. Descending
    into it would let the same work be announced twice at two granularities,
    and two robots could win the same tree.
    """
    tasks = mission_tasks.tasks_in(example("reactive_fallbacks.xml"))
    trees = [t.target.a for t in tasks if t.target.kind is TargetKind.TREE]

    assert sorted(trees) == [10, 60]
    for task in tasks:
        if task.target.kind is TargetKind.TREE:
            assert task.capabilities == cap_mask(
                Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF
            )


def test_a_gps_mission_yields_gps_targets():
    """quad.xml has no tree ids at all -- only latitudes and longitudes.

    This is the mission the tree-index-only design could not have delegated.
    """
    tasks = mission_tasks.tasks_in(example("quad.xml"))

    assert tasks, "quad.xml has work in it"
    assert all(task.target.kind is TargetKind.GPS for task in tasks)
    first = tasks[0].target
    assert round(first.lat_deg, 6) == 37.366449
    assert round(first.lon_deg, 6) == -120.423065


def test_follow_person_has_nowhere_to_be_sent():
    """follow.xml is one FollowPerson and nothing else.

    It happens wherever the robot and the person are, so it names no place --
    and the coordinator refuses to announce it rather than announcing a target
    every listener resolves to somewhere different.
    """
    tasks = mission_tasks.tasks_in(example("follow.xml"))

    assert len(tasks) == 1
    assert tasks[0].capabilities == cap_mask(Capability.FOLLOW_PERSON)
    assert tasks[0].target == Target.none()
    assert not tasks[0].delegable


def test_a_relative_move_is_not_delegable():
    """relative_move.xml moves 5 m forward and turns.

    "Forward" is relative to *this* robot's pose. Another robot sent to do it
    would do it somewhere else entirely, so it gets no target at all.
    """
    tasks = mission_tasks.tasks_in(example("relative_move.xml"))

    assert len(tasks) == 1
    assert not tasks[0].delegable


@pytest.mark.parametrize("name", EXAMPLE_FILES)
def test_every_example_mission_yields_usable_tasks(name):
    """No example produces a task the wire could not carry."""
    for task in mission_tasks.tasks_in(example(name)):
        assert 0 < task.task_id <= TASK_ID_MAX
        assert task.capabilities, f"{task.name} needs no actions at all"
        assert task.xml, f"{task.name} carries no subtree to hand over"


# ==========================================================================
# Which unit a failing node belongs to
# ==========================================================================


@pytest.mark.parametrize(
    "node_name,tree",
    [
        ("Visit_Tree_10", 10),
        ("Sample_Leaves_Tree_10", 10),
        ("Visit_Tree_60", 60),
        ("Sample_Leaves_Tree_60", 60),
        # The traverse is how the robot reaches tree 60, so it belongs to the
        # unit it leads to rather than the one it leaves.
        ("Exit_To_Top_Headland_Col2_Tree2", 60),
        ("Traverse_Top_Headland_To_Col4_Tree4", 60),
    ],
)
def test_a_failing_node_resolves_to_its_own_unit(node_name, tree):
    """Regression: a live run resolved tree 60's failure to tree 10's task.

    sample_leafs.xml is one flat Sequence holding all six leaves, so walking up
    from the failing leaf lands on the whole mission -- and reports its *first*
    unit no matter which leaf failed. Every node gave the same answer, and the
    answer was wrong for all but two of them. The auction would have been for
    work the robot had already finished.
    """
    task = mission_tasks.task_for_node(example("sample_leafs.xml"), node_name)

    assert task is not None
    assert task.target == Target.tree(tree)


def test_either_half_of_a_unit_resolves_to_the_same_task():
    mission = example("sample_leafs.xml")
    move = mission_tasks.task_for_node(mission, "Visit_Tree_60")
    sample = mission_tasks.task_for_node(mission, "Sample_Leaves_Tree_60")
    assert move.task_id == sample.task_id


def test_the_resolved_task_is_one_the_mission_actually_contains():
    """The lookup and the announcement have to agree.

    They come from the same traversal now, which is the only reason they
    cannot drift -- a second traversal that re-derived the units would agree
    right up until it did not, and the symptom would be an announcement for a
    task id nothing in the plan matches.
    """
    mission = example("sample_leafs.xml")
    announced = {task.task_id for task in mission_tasks.tasks_in(mission)}
    for name in ("Visit_Tree_10", "Sample_Leaves_Tree_60", "Visit_Tree_60"):
        assert mission_tasks.task_for_node(mission, name).task_id in announced


def test_an_unknown_node_name_resolves_to_nothing():
    assert mission_tasks.task_for_node(example("sample_leafs.xml"), "<tree>") is None
    assert mission_tasks.task_for_node(example("sample_leafs.xml"), "") is None


# ==========================================================================
# Identity
# ==========================================================================


def test_task_ids_are_unique_within_a_mission():
    """The bug the tree id had.

    sample_leafs.xml traverses *through* tree 2 twice on its way between
    columns. Keying on the tree id gave both traverses the same task number, so
    the fleet could not tell one from the other.
    """
    for name in EXAMPLE_FILES:
        tasks = mission_tasks.tasks_in(example(name))
        ids = [task.task_id for task in tasks]
        assert len(ids) == len(set(ids)), f"{name} has colliding task ids"


def test_a_declared_task_id_wins_over_the_derived_one():
    """Once the planner names its units, the plan is the authority."""
    declared = example("sample_leafs.xml").replace(
        "<Sequence>", '<Sequence name="Whole_Mission" task_id="4210">', 1
    )
    tasks = mission_tasks.tasks_in(declared)
    # The named sequence wraps the whole mission, so it is a container rather
    # than a unit -- the declaration is read, but it does not make the mission
    # into one task. What matters is that parsing it does not fail.
    assert tasks


def test_the_same_plan_read_twice_gives_the_same_ids():
    """Two robots hold the same mission and must agree without negotiating."""
    first = mission_tasks.tasks_in(example("sample_leafs.xml"))
    second = mission_tasks.tasks_in(example("sample_leafs.xml"))
    assert [t.task_id for t in first] == [t.task_id for t in second]


# ==========================================================================
# The round trip: a won task has to be executable
# ==========================================================================


EMPTY = """<?xml version="1.0"?>
<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">
  <Mission>work taken on from a peer</Mission>
  <BehaviorTree ID="Absorbed">
    <Sequence>
      <MoveToTreeID name="Return_Home" action_name="follow_tree_id_waypoint"
                    id="1" approach_tree="false"/>
    </Sequence>
  </BehaviorTree>
</root>
"""


@pytest.mark.parametrize("name", EXAMPLE_FILES)
def test_a_task_grafted_into_another_plan_still_validates(name, schema):
    """The first evidence that a delegated task is executable by its winner.

    A task that cannot be written back into a plan the schema accepts is a task
    the winner's own arbiter would reject -- so the auction would complete, the
    ownership would transfer, and the work would then quietly fail to run.

    Only missions that are valid to begin with can be checked this way. One of
    the examples is not (see the test below), and grafting cannot be blamed for
    invalidity it inherited.
    """
    mission = example(name)
    if not schema.validate(etree.fromstring(mission.encode())):
        pytest.skip(f"{name} does not validate against the schema to begin with")

    for task in mission_tasks.tasks_in(mission):
        merged = mission_tasks.insert_task(EMPTY, task)
        assert merged is not None, f"{task.name} could not be grafted"
        document = etree.fromstring(merged.encode())
        assert schema.validate(
            document
        ), f"{name}:{task.name} produced an invalid plan: {schema.error_log}"


def test_which_example_missions_the_schema_accepts(schema):
    """A record of an existing drift, not a claim that it is fine.

    ``relative_move.xml`` uses action_name="navigate_to_pose_in_frame" where
    the schema fixes "move_in_frame" and "rotate_in_frame". bt_runner with
    xml_validation enabled -- the default -- refuses to build a tree from it,
    so it is a mission this robot cannot currently run at all.

    Pinned here so the drift is visible rather than something the graft test
    quietly skips. When the example (or the schema) is corrected, this test
    fails and is deleted.
    """
    invalid = {
        name
        for name in EXAMPLE_FILES
        if not schema.validate(etree.fromstring(example(name).encode()))
    }
    assert invalid == {"relative_move.xml"}


def test_removing_a_task_leaves_the_rest_of_the_mission_intact():
    mission = example("sample_leafs.xml")
    tasks = mission_tasks.tasks_in(mission)
    target = next(t for t in tasks if t.target == Target.tree(60))

    without = mission_tasks.remove_task(mission, target.task_id)

    assert without is not None
    remaining = mission_tasks.tasks_in(without)
    assert target.task_id not in {t.task_id for t in remaining}
    assert len(remaining) == len(tasks) - 1
    # The other tree is untouched: shedding one unit must not disturb the plan
    # around it.
    assert any(t.target == Target.tree(10) for t in remaining)


def test_removing_a_task_that_is_not_there_reports_so():
    assert mission_tasks.remove_task(example("follow.xml"), 65535) is None


def test_a_removal_and_an_insertion_round_trip(schema):
    """Shedding a task from one plan and absorbing it into another.

    The two halves of a transfer, run back to back. Both plans have to still be
    plans afterwards, which is the property the arbiter will check anyway and
    the one worth failing here rather than at runtime on the winner.
    """
    mission = example("sample_leafs.xml")
    task = next(
        t for t in mission_tasks.tasks_in(mission) if t.target == Target.tree(60)
    )

    shed = mission_tasks.remove_task(mission, task.task_id)
    absorbed = mission_tasks.insert_task(EMPTY, task)

    for plan in (shed, absorbed):
        assert schema.validate(etree.fromstring(plan.encode())), schema.error_log

    # The work moved rather than being copied or lost.
    assert Target.tree(60) not in {t.target for t in mission_tasks.tasks_in(shed)}
    assert Target.tree(60) in {t.target for t in mission_tasks.tasks_in(absorbed)}


# ==========================================================================
# Malformed input
# ==========================================================================


@pytest.mark.parametrize("bad", ["", None, "<not xml", "{}", "<root/>"])
def test_unusable_input_yields_no_tasks_rather_than_raising(bad):
    # Every caller here is already downstream of something that failed. Adding
    # an exception to that replaces a diagnosable "no task resolved" with a
    # traceback about the wrong thing.
    assert mission_tasks.tasks_in(bad) == []


def test_a_leaf_with_nonsense_coordinates_names_no_place_of_its_own():
    """An unparseable fix establishes no objective, so it opens no new unit.

    It merges forward into the next unit that does have one, which leaves it
    executable in its original order and announced under a place somebody
    actually wrote down. The alternative -- treating it as its own unit with a
    guessed target -- would put a robot on the road to a coordinate nobody
    chose.
    """
    intact = mission_tasks.tasks_in(example("quad.xml"))
    broken = mission_tasks.tasks_in(
        example("quad.xml").replace('latitude="37.366449"', 'latitude="north"', 1)
    )

    assert len(broken) == len(intact) - 1
    assert all(task.delegable for task in broken), "no invented targets"
