"""A task crosses robots, and the arbiter verifies what that made.

The coordinator's entry into the gate, exercised through the real node: the
service handler, the edit it applies to the plan it holds, and the verdict.
Everything is real except the model call -- the formula is injected, because an
LLM in a test makes the result a coin flip, and SPIN is what is actually being
asked the question anyway.

This covers the path nothing else does. ``test_ltl_gate`` checks the decision
given a candidate; this checks that a task described only by the fields the
radio carries becomes a candidate at all, which is the step between winning an
auction and having a mission you can run.
"""

import os
import sys

import pytest
from lxml import etree

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_interfaces.srv import VerifyReplan  # noqa: E402
from amiga_ros2_agents.replanning.arbiter_node import ArbiterNode  # noqa: E402
from amiga_ros2_agents.verification import ltl, ltl_gate, verify  # noqa: E402
from amiga_ros2_comms.codec import (  # noqa: E402
    Capability,
    TargetKind,
    cap_mask,
)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCHEMA = os.path.join(REPO, "amiga_ros2_behavior_tree", "schemas", "amiga_btcpp.xsd")
EXAMPLES = os.path.join(REPO, "amiga_ros2_behavior_tree", "examples")

needs_spin = pytest.mark.skipif(not verify.spin_available(), reason="spin not on PATH")

#: The real mission, not a fixture. A hand-written one-line plan makes every
#: edit "100% of lines changed" and trips the arbiter's edit-size limit, which
#: says more about the fixture than about the code -- real plans are formatted.
with open(os.path.join(EXAMPLES, "sample_leafs.xml")) as _handle:
    ACTIVE = _handle.read()

MISSION = "sample leaves from trees 10 and 60"


@pytest.fixture
def arbiter(request):
    """A real ArbiterNode holding ACTIVE, with the model call stubbed."""
    node = ArbiterNode()
    formula = getattr(request, "param", None) or "<>sampled_tree_10"

    node._ltl_gate = ltl_gate.LtlGate(
        SCHEMA, generate=lambda text: ltl.Formula(formula)
    )
    node.active_mission_xml = ACTIVE
    node.original_objectives = {"10", "60"}
    node.max_droppable = 2
    yield node
    node.destroy_node()


def request_for(
    task_id=4210,
    tree=35,
    caps=(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF),
    removing=False,
    task_xml="",
):
    message = VerifyReplan.Request()
    message.removing = removing
    message.task_id = task_id
    message.required_capabilities = cap_mask(*caps)
    message.target_kind = int(TargetKind.TREE)
    message.target_a = tree
    message.target_b = 0
    message.priority = 100
    message.task_xml = task_xml
    return message


def call(node, message):
    return node._on_verify_replan(message, VerifyReplan.Response())


# ---------------------------------------------------------------------------
# Winning a task
# ---------------------------------------------------------------------------


@needs_spin
def test_absorbed_task_is_grafted_and_committed(arbiter):
    """The auction-win path, end to end inside the arbiter."""
    response = call(arbiter, request_for(tree=35))
    assert response.accepted, response.reason

    doc = etree.fromstring(arbiter.active_mission_xml.encode())
    ids = {
        mv.get("id")
        for mv in doc.iter("MoveToTreeID")
        if mv.get("approach_tree") == "true"
    }
    assert ids == {"10", "60", "35"}


@needs_spin
def test_absorbing_extends_the_mission_text(arbiter):
    """The specification has to grow with the work, or the plan overshoots it.

    Written from the wire fields by the arbiter, never by a model -- which is
    what stops the thing being checked from widening what it is checked against.
    """
    call(arbiter, request_for(tree=35))
    doc = etree.fromstring(arbiter.active_mission_xml.encode())
    assert doc.findtext("Mission") == f"{MISSION}; sample leaves from tree 35"


@needs_spin
def test_move_only_task_extends_the_text_differently(arbiter):
    call(arbiter, request_for(tree=7, caps=(Capability.MOVE_TO_TREE_ID,)))
    doc = etree.fromstring(arbiter.active_mission_xml.encode())
    assert doc.findtext("Mission") == f"{MISSION}; visit tree 7"


@needs_spin
def test_permission_to_extend_does_not_outlive_the_call(arbiter):
    """The planner must not inherit the coordinator's licence to edit the text.

    Without this, one absorbed task would leave the door open for whatever the
    planner published next.
    """
    call(arbiter, request_for(tree=35))
    forged = arbiter.active_mission_xml.replace(
        f"{MISSION}; sample leaves from tree 35", "do whatever you like"
    )
    accepted, _, reason = arbiter._decide(forged)
    assert not accepted
    assert "may not be rewritten" in reason


@needs_spin
def test_supplied_subtree_is_used_verbatim(arbiter):
    """A task our own planner shed carries its XML; it is not re-synthesised."""
    subtree = (
        '<Sequence name="Handmade" task_id="99">'
        '<MoveToTreeID name="V22" action_name="follow_tree_id_waypoint" id="22" '
        'approach_tree="true"/>'
        '<SampleLeaf name="S22" action_name="segment_leaves"/>'
        "</Sequence>"
    )
    response = call(arbiter, request_for(task_id=99, tree=22, task_xml=subtree))
    assert response.accepted, response.reason
    assert "Handmade" in arbiter.active_mission_xml


# ---------------------------------------------------------------------------
# Losing a task
# ---------------------------------------------------------------------------


@needs_spin
@pytest.mark.parametrize("arbiter", ["<>sampled_tree_10"], indirect=True)
def test_transferred_task_is_removed(arbiter):
    """Tree 60 leaves; the mission text is unchanged, so the formula is too."""
    task_id = _task_id_of(ACTIVE, "Visit_Tree_60")
    response = call(arbiter, request_for(task_id=task_id, removing=True))
    assert response.accepted, response.reason

    doc = etree.fromstring(arbiter.active_mission_xml.encode())
    assert doc.findtext("Mission") == MISSION
    ids = {
        mv.get("id")
        for mv in doc.iter("MoveToTreeID")
        if mv.get("approach_tree") == "true"
    }
    assert "60" not in ids


@needs_spin
@pytest.mark.parametrize(
    "arbiter", ["<>sampled_tree_10 && <>sampled_tree_60"], indirect=True
)
def test_removing_work_the_mission_still_wants_is_refused(arbiter):
    """The whole point: a plan cannot drop an objective and still verify.

    Shedding tree 60 does not licence rewriting the mission, so the formula
    still asks for it and the shortened plan no longer establishes it.
    """
    task_id = _task_id_of(ACTIVE, "Visit_Tree_60")
    response = call(arbiter, request_for(task_id=task_id, removing=True))
    assert not response.accepted
    assert "sampled_tree_60" in response.reason


@needs_spin
@pytest.mark.parametrize("arbiter", ["<>sampled_tree_10"], indirect=True)
def test_transfer_justifies_the_drop_it_causes(arbiter):
    """A transferred objective is not an abandoned one.

    Gate 1 of ``_check_objective_preserved`` cannot tell the two apart from the
    XML, so without the coordinator vouching for it every successful transfer
    would be rejected as an unjustified drop and no auction would ever commit.
    The justification is stronger than the fault reports it normally takes: a
    peer acknowledged the GRANT, which is what made the task theirs.
    """
    task_id = _task_id_of(ACTIVE, "Visit_Tree_60")
    assert call(arbiter, request_for(task_id=task_id, removing=True)).accepted
    assert "60" in arbiter.justified_drops


@needs_spin
@pytest.mark.parametrize(
    "arbiter", ["<>sampled_tree_10 && <>sampled_tree_60"], indirect=True
)
def test_a_rejected_transfer_leaves_no_justification_behind(arbiter):
    """Nothing was handed over, so nothing is excused.

    Otherwise a transfer that failed verification would still buy the planner a
    free pass to drop that objective later.
    """
    task_id = _task_id_of(ACTIVE, "Visit_Tree_60")
    assert not call(arbiter, request_for(task_id=task_id, removing=True)).accepted
    assert "60" not in arbiter.justified_drops


def test_unknown_task_removal_is_refused(arbiter):
    response = call(arbiter, request_for(task_id=31337, removing=True))
    assert not response.accepted
    assert "not in the active plan" in response.reason


# ---------------------------------------------------------------------------
# Refusals
# ---------------------------------------------------------------------------


def test_unrebuildable_task_is_refused(arbiter):
    """A GPS target the announcement rounded cannot be turned back into a leaf."""
    message = request_for()
    message.target_kind = int(TargetKind.GPS)
    response = call(arbiter, message)
    assert not response.accepted
    assert "cannot be rebuilt" in response.reason


def test_no_active_mission_accepts_without_claiming_verification():
    """A task must not be stranded over a plan we have not been handed."""
    node = ArbiterNode()
    try:
        node.active_mission_xml = None
        response = call(node, request_for())
        assert response.accepted
        assert not response.verified
    finally:
        node.destroy_node()


@needs_spin
def test_violating_absorption_is_rejected_with_a_counterexample(arbiter):
    """A formula the extended plan cannot satisfy comes back refuted."""
    arbiter._ltl_gate = ltl_gate.LtlGate(
        SCHEMA,
        generate=lambda text: ltl.Formula("<>(at_tree_35 && <>at_tree_10)"),
    )
    response = call(arbiter, request_for(tree=35))
    assert not response.accepted
    assert response.verified
    assert "violation" in response.reason.lower()


def _task_id_of(mission_xml: str, node_name: str) -> int:
    from amiga_ros2_agents.mission import mission_tasks

    task = mission_tasks.task_for_node(mission_xml, node_name)
    assert task is not None, f"no task around {node_name}"
    return task.task_id
