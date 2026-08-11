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


#: The three plans the upstream mission planner wrote for the three-robot run,
#: and the tree each of their two units is about. These replaced the deleted
#: aisle_sample_*.xml fixtures; the XML here is extracted straight out of the
#: matching sample_*.bin, which is what the planner actually emits.
PLANNER_PLANS = {
    "sample_20_64.xml": [20, 64],
    "sample_22_66.xml": [22, 66],
    "sample_24_68.xml": [24, 68],
}


@pytest.mark.parametrize("name,trees", sorted(PLANNER_PLANS.items()))
def test_a_planner_written_mission_splits_into_one_task_per_tree(name, trees):
    """The shape that used to defeat every rule here.

    These plans put the aisle move *outside* the retry wrapper that holds the
    tree move and the sample, so the mission's top Sequence has leaves and
    control nodes side by side. That was taken whole -- the entire mission read
    back as one task, so nothing in it could be offered to anybody, and the
    fleet had nothing to auction on the very plans it was going to be run with.
    """
    tasks = mission_tasks.tasks_in(example(name))

    assert [task.target.a for task in tasks] == trees
    for task in tasks:
        assert task.delegable
        assert task.capabilities == cap_mask(
            Capability.MOVE_TO_AISLE_HEAD,
            Capability.MOVE_TO_TREE_ID,
            Capability.SAMPLE_LEAF,
        ), "the aisle move is part of the unit, so it is part of what is announced"


@pytest.mark.parametrize("name,trees", sorted(PLANNER_PLANS.items()))
def test_shedding_a_tree_takes_the_way_in_with_it(name, trees, schema):
    """Handing tree 60 away removes the drive into tree 60's aisle.

    Not a tidiness point. What is left has to be a plan the loser can keep
    running: valid against the schema, still holding the work it did not give
    up, and still readable as the tasks it has -- otherwise the transfer
    succeeds on the radio and the robot is left with a mission it cannot load.
    """
    mission = example(name)
    tasks = mission_tasks.tasks_in(mission)
    kept, given = tasks[0], tasks[1]

    without = mission_tasks.remove_task(mission, given.task_id)
    assert without is not None

    document = etree.fromstring(without.encode())
    assert schema.validate(document), schema.error_log
    assert [t.task_id for t in mission_tasks.tasks_in(without)] == [kept.task_id]

    aisle_ids = {el.get("id") for el in document.iter("MoveToAisleHead")}
    tree_ids = {el.get("id") for el in document.iter("MoveToTreeID")}
    assert str(given.target.a) not in tree_ids
    assert str(_aisle_in(given.xml)) not in aisle_ids


def test_a_prerequisite_two_tasks_share_is_not_cut_with_either_of_them():
    """One aisle move, two trees in that aisle. Give one away; the other stays.

    The unit that *holds* the shared step is still the one that carries it --
    it sits there and it leads there -- but cutting it out would leave the
    second tree with no way in, in a plan nobody edited to say so. So the task
    describes the whole unit and the removal leaves the shared step behind.
    """
    plan = (
        '<root BTCPP_format="4" schema_location="s.xsd"><Mission>m</Mission>'
        '<BehaviorTree ID="T"><Sequence>'
        '<MoveToAisleHead name="into_6" action_name="move_to_aisle_head" id="6"/>'
        '<RetryUntilSuccessful name="r60" num_attempts="3"><Sequence name="s60">'
        '<MoveToTreeID name="v60" action_name="follow_tree_id_waypoint" id="60" '
        'approach_tree="true"/>'
        '<SampleLeaf name="p60" action_name="segment_leaves"/>'
        "</Sequence></RetryUntilSuccessful>"
        '<RetryUntilSuccessful name="r61" num_attempts="3"><Sequence name="s61">'
        '<MoveToTreeID name="v61" action_name="follow_tree_id_waypoint" id="61" '
        'approach_tree="true"/>'
        '<SampleLeaf name="p61" action_name="segment_leaves"/>'
        "</Sequence></RetryUntilSuccessful>"
        "</Sequence></BehaviorTree></root>"
    )
    tasks = mission_tasks.tasks_in(plan)
    assert [task.target.a for task in tasks] == [60, 61]
    # The first unit is *about* the aisle move -- it is announced with it.
    assert tasks[0].capabilities & (1 << Capability.MOVE_TO_AISLE_HEAD)

    without = mission_tasks.remove_task(plan, tasks[0].task_id)
    document = etree.fromstring(without.encode())
    assert [el.get("id") for el in document.iter("MoveToAisleHead")] == ["6"]
    assert [el.get("id") for el in document.iter("MoveToTreeID")] == ["61"]


def _aisle_in(subtree_xml: str):
    head = etree.fromstring(subtree_xml.encode()).find("MoveToAisleHead")
    return head.get("id") if head is not None else None


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
    """Once the planner names its units, the plan is the authority.

    Declaring an id also says *this is one unit*: the sequence is no longer cut
    up by objective, because a plan that numbered something has said where its
    boundaries are, and guessing different ones would give the fleet two names
    for the same work.
    """
    declared = example("sample_leafs.xml").replace(
        "<Sequence>", '<Sequence name="Whole_Mission__task_4210">', 1
    )
    tasks = mission_tasks.tasks_in(declared)
    assert [task.task_id for task in tasks] == [4210]
    assert tasks[0].name == "Whole_Mission"


def test_the_id_lives_in_the_name_because_an_attribute_would_not_load():
    """The constraint this encoding exists for.

    BT.CPP parses every attribute that is not ``ID``, ``name`` or a
    pre/post-condition as a *port*, and refuses to build a tree that declares
    one the node type does not provide. So ``task_id="5"`` on a ``<Sequence>``
    is not a slower path or a lossy one, it is a mission that does not run --
    and the worst place for that is a subtree a winner just absorbed, where it
    lands after the auction has already succeeded.
    """
    attribute = example("sample_leafs.xml").replace(
        "<Sequence>", '<Sequence name="Whole_Mission" task_id="4210">', 1
    )
    tasks = mission_tasks.tasks_in(attribute)
    assert 4210 not in [task.task_id for task in tasks]

    rebuilt = mission_tasks.name_with_task_id("Whole_Mission", 4210)
    assert "task_id" not in rebuilt
    assert mission_tasks.declared_task_id(rebuilt) == 4210
    assert mission_tasks.task_label(rebuilt) == "Whole_Mission"


def test_an_unlabelled_declaration_is_the_form_the_arbiter_already_writes():
    """``task_<n>`` is what an absorbed task is named; it declares its id too."""
    assert mission_tasks.declared_task_id("task_77") == 77
    assert mission_tasks.name_with_task_id("task_77", 77) == "task_77"
    # Nothing underneath to fall back to, so the whole name stands rather than
    # leaving an operator display blank.
    assert mission_tasks.task_label("task_77") == "task_77"


def test_a_number_outside_the_wire_range_is_a_name_and_not_a_declaration():
    """0 is the wire's "no task" and 70000 does not survive a TASK_ANNOUNCE."""
    for name in ("Stage__task_0", "Stage__task_70000"):
        assert mission_tasks.declared_task_id(name) is None
        assert mission_tasks.task_label(name) == name


def test_a_leaf_named_like_a_declaration_does_not_rename_its_unit():
    """The id is read from the unit's own name, never from ``_name_of``'s fallback.

    ``_name_of`` reaches down into the leaves when a control node is unnamed,
    which is the common shape. If that fallback could declare, a planner
    numbering its *steps* would be silently renumbering the fleet's tasks.
    """
    plan = (
        '<root BTCPP_format="4" schema_location="s.xsd"><Mission>m</Mission>'
        '<BehaviorTree ID="T"><Sequence>'
        '<MoveToTreeID name="task_9" action_name="follow_tree_id_waypoint" '
        'id="10" approach_tree="true"/>'
        '<SampleLeaf name="s10" action_name="segment_leaves"/>'
        "</Sequence></BehaviorTree></root>"
    )
    assert [task.task_id for task in mission_tasks.tasks_in(plan)] != [9]


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


def test_shedding_a_retried_task_does_not_leave_its_wrapper_behind(schema):
    """The loser's plan has to still load, and an empty decorator does not.

    ``RetryUntilSuccessful`` around a single ``Sequence`` is how a retried
    objective is written, and it is the shape that separates units cleanly --
    so it is the shape a shed most often cuts out of. Leaving the wrapper
    childless breaks the plan of the robot that gave the work away, after the
    transfer has already been agreed.
    """
    mission = (
        '<root BTCPP_format="4" schema_location="s.xsd"><Mission>m</Mission>'
        '<BehaviorTree ID="T"><Sequence name="Both">'
        '<RetryUntilSuccessful name="RetryTree10" num_attempts="3">'
        '<Sequence name="Tree10Seq">'
        '<MoveToTreeID name="V10" action_name="follow_tree_id_waypoint" '
        'id="10" approach_tree="true"/>'
        '<SampleLeaf name="S10" action_name="segment_leaves"/>'
        "</Sequence></RetryUntilSuccessful>"
        '<RetryUntilSuccessful name="RetryTree60" num_attempts="3">'
        '<Sequence name="Tree60Seq">'
        '<MoveToTreeID name="V60" action_name="follow_tree_id_waypoint" '
        'id="60" approach_tree="true"/>'
        '<SampleLeaf name="S60" action_name="segment_leaves"/>'
        "</Sequence></RetryUntilSuccessful>"
        "</Sequence></BehaviorTree></root>"
    )
    task = next(
        t for t in mission_tasks.tasks_in(mission) if t.target == Target.tree(60)
    )

    without = mission_tasks.remove_task(mission, task.task_id)

    assert without is not None
    doc = etree.fromstring(without.encode())
    assert [node.get("name") for node in doc.iter("RetryUntilSuccessful")] == [
        "RetryTree10"
    ]
    assert schema.validate(doc), schema.error_log


def test_shedding_the_last_task_still_leaves_a_document_with_a_tree_in_it():
    """A mission with nothing left has no valid form; it can still be readable.

    The XSD requires a control node to hold at least one child, so a plan whose
    last unit was shed will be refused whichever way this goes. What is worth
    pinning is which refusal: the arbiter can read an empty ``<BehaviorTree>``
    as "this mission no longer does anything". A document the pruning walked all
    the way out of has no tree at all, and it could only call that malformed.
    """
    mission = (
        '<root BTCPP_format="4" schema_location="s.xsd"><Mission>m</Mission>'
        '<BehaviorTree ID="T"><Sequence name="Only">'
        '<MoveToTreeID name="V10" action_name="follow_tree_id_waypoint" '
        'id="10" approach_tree="true"/>'
        "</Sequence></BehaviorTree></root>"
    )
    task = mission_tasks.tasks_in(mission)[0]

    without = mission_tasks.remove_task(mission, task.task_id)

    assert without is not None
    doc = etree.fromstring(without.encode())
    assert doc.find("BehaviorTree") is not None
    assert not mission_tasks.tasks_in(without)


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
