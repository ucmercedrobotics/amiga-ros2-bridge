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

import json
import os
import struct
import sys

import pytest
from lxml import etree

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_interfaces.srv import VerifyReplan  # noqa: E402
from amiga_ros2_agents.mission import orchard  # noqa: E402
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
    note="",
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
    message.note = note
    return message


def replan_requests(node):
    """Capture what the arbiter asks its planner for. Returns a live list."""
    captured = []
    node.replan_request_pub.publish = lambda msg: captured.append(json.loads(msg.data))
    return captured


def real_orchard():
    """The tree -> aisle map out of a real mission binary."""
    with open(os.path.join(EXAMPLES, "sample_20_64.bin"), "rb") as handle:
        data = handle.read()
    offset, frames = 0, []
    while offset + 4 <= len(data):
        (length,) = struct.unpack(">I", data[offset : offset + 4])
        offset += 4
        frames.append(data[offset : offset + length])
        offset += length
    return orchard.parse(frames[1].decode())


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
        '<Sequence name="Handmade__task_99">'
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
@pytest.mark.parametrize("arbiter", ["<>sampled_tree_10"], indirect=True)
def test_a_transferred_tree_does_not_spend_the_viability_budget(arbiter):
    """The abort that ended a live mission over work a peer was already doing.

    amiga1's camera was dead, so both its trees went to auction. Robot 2 took
    tree 20 and robot 3 took tree 64, and both sampled them. But amiga1's own
    budget said one tree may go unsampled, and the second transfer made two
    trees "dropped" from its plan -- so its arbiter aborted the mission while
    robot 2 was standing at tree 20 with the arm out.

    The budget bounds work that will NOT HAPPEN. Transferred work happens; it
    happens somewhere else. So it comes out of the outstanding set the same way
    finished work does, and only genuine abandonment is charged for.
    """
    arbiter.max_droppable = 0  # not one tree of this mission may go unsampled

    task_id = _task_id_of(ACTIVE, "Visit_Tree_60")
    assert call(arbiter, request_for(task_id=task_id, removing=True)).accepted
    assert arbiter.transferred_objectives == {"60"}

    # The plan the robot runs from here holds tree 10 and nothing else, which
    # against a budget of zero is only acceptable if 60 stopped being ours.
    ok, reason, abort = arbiter._check_objective_preserved(
        etree.fromstring(arbiter.active_mission_xml.encode("utf-8"))
    )
    assert ok, reason
    assert not abort


def test_abandoning_a_tree_still_spends_it(arbiter):
    """The other half, or the fix would just disable the gate.

    The same shortened plan as above, arrived at without a peer taking
    anything -- tree 60 is simply gone. That is the case the budget exists for,
    and it must still abort.
    """
    from amiga_ros2_agents.mission import mission_tasks

    arbiter.max_droppable = 0
    arbiter.justified_drops = {"60"}  # gate 1 satisfied, so gate 2 is what answers

    abandoned = mission_tasks.remove_task(ACTIVE, _task_id_of(ACTIVE, "Visit_Tree_60"))
    ok, reason, abort = arbiter._check_objective_preserved(
        etree.fromstring(abandoned.encode("utf-8"))
    )

    assert not ok
    assert abort, "an abandoned objective over budget must still end the mission"
    assert "viability budget" in reason


@needs_spin
@pytest.mark.parametrize(
    "arbiter", ["<>sampled_tree_10 && <>sampled_tree_60"], indirect=True
)
def test_a_rejected_transfer_leaves_no_transfer_credit_behind(arbiter):
    """A transfer that failed verification hands nothing over, so it buys
    nothing -- neither a justified drop nor room in the budget."""
    task_id = _task_id_of(ACTIVE, "Visit_Tree_60")
    assert not call(arbiter, request_for(task_id=task_id, removing=True)).accepted
    assert arbiter.transferred_objectives == set()


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


# ---------------------------------------------------------------------------
# Handing the question on: what the deterministic half cannot decide
# ---------------------------------------------------------------------------


@needs_spin
def test_absorbing_a_task_asks_this_robots_planner_to_replan(arbiter):
    """The committed edit is structural, and nothing here pretends otherwise.

    ``synthesize`` appends the objective and its prerequisites to the end of the
    plan, because appending is the only placement this layer can justify. Where
    the work actually belongs in the run is a planning judgement, so the arbiter
    commits a plan that *runs* and then asks the thing that can make that
    judgement -- as a request, never as a plan, which is what keeps the model
    outside the decision that publishes /mission/xml.
    """
    asked = replan_requests(arbiter)
    assert call(arbiter, request_for(tree=35)).accepted

    assert len(asked) == 1
    assert asked[0]["cause"] == "task_absorbed"
    assert asked[0]["task_id"] == 4210
    assert asked[0]["target"] == {"kind": "tree", "a": 35, "b": 0}


@needs_spin
def test_the_peers_note_is_what_the_planner_is_given(arbiter):
    """The one thing in a transfer that no amount of structure could derive.

    Every other field is something the fleet computed. This is a sentence about
    the world that only the robot which raised the task knew, carried across a
    radio that fits 328 bytes of it -- and it is useless anywhere but here,
    because a planner is the only thing in this system that can act on prose.
    """
    asked = replan_requests(arbiter)
    note = "north end of column 6 is flooded; enter from the south"
    assert call(arbiter, request_for(tree=35, note=note)).accepted
    assert asked[0]["note"] == note


@needs_spin
def test_a_rebuilt_task_names_the_step_it_could_not_supply(arbiter):
    """Without an orchard the winner cannot choose the lane, and says so.

    ``dropped`` is the handover from the deterministic half to the planner: it
    is not an error, it is the list of things seven bytes of radio could not
    carry and only this robot can put back.
    """
    asked = replan_requests(arbiter)
    assert call(
        arbiter,
        request_for(
            tree=35,
            caps=(
                Capability.MOVE_TO_AISLE_HEAD,
                Capability.MOVE_TO_TREE_ID,
                Capability.SAMPLE_LEAF,
            ),
        ),
    ).accepted
    assert asked[0]["dropped"] == ["MoveToAisleHead"]


@needs_spin
def test_with_the_orchard_the_winner_builds_the_way_in_itself(arbiter):
    """And then has nothing to report as missing."""
    arbiter.orchard = real_orchard()
    asked = replan_requests(arbiter)
    assert call(
        arbiter,
        request_for(
            tree=60,
            caps=(
                Capability.MOVE_TO_AISLE_HEAD,
                Capability.MOVE_TO_TREE_ID,
                Capability.SAMPLE_LEAF,
            ),
        ),
    ).accepted

    assert asked[0]["dropped"] == []
    document = etree.fromstring(arbiter.active_mission_xml.encode())
    assert "4" in {el.get("id") for el in document.iter("MoveToAisleHead")}


def test_shedding_a_task_reports_what_it_left_behind():
    """The user's opening ask: after a task is given away, replan anyway.

    Shedding is not a local edit. Tree 64 leaves and the drive back out of its
    aisle stays -- a step that now establishes something nothing in the plan
    needs. That is invisible in the XML and obvious in the ontology's terms, so
    the planner is told in those terms rather than handed a diff and asked to
    notice.
    """
    node = ArbiterNode()
    try:
        with open(os.path.join(EXAMPLES, "sample_20_64.xml")) as handle:
            node.active_mission_xml = handle.read()
        node.orchard = real_orchard()
        node.ltl_verification = False  # the finding is the subject, not the gate
        asked = replan_requests(node)

        task_id = _task_id_of(node.active_mission_xml, "ApproachTree64")
        assert call(node, request_for(task_id=task_id, removing=True)).accepted

        assert asked[0]["cause"] == "task_transferred"
        findings = " ".join(asked[0]["findings"])
        assert "ExitRow2" in findings
        assert "in_aisle(2)" in findings
        assert "EnterRow2" not in findings, "the way in to work we kept is not dead"
    finally:
        node.destroy_node()


# ---------------------------------------------------------------------------
# The gate, switched off
# ---------------------------------------------------------------------------


def unverified_arbiter():
    node = ArbiterNode()
    node.ltl_verification = False
    node.active_mission_xml = ACTIVE
    return node


def test_with_verification_off_a_task_is_absorbed_without_spin_or_a_model():
    """What the flag is for: bringing the coordination loop up end to end.

    No formula, no SPIN, no viability budget -- the ``_ltl_gate`` here would
    raise if it were consulted, which is how this test knows it was not.
    """
    node = unverified_arbiter()
    try:
        node._ltl_gate = None
        response = call(node, request_for(tree=35))
        assert response.accepted, response.reason
        ids = {
            mv.get("id")
            for mv in etree.fromstring(node.active_mission_xml.encode()).iter(
                "MoveToTreeID"
            )
            if mv.get("approach_tree") == "true"
        }
        assert ids == {"10", "60", "35"}
    finally:
        node.destroy_node()


def test_an_unverified_accept_never_claims_to_have_been_verified():
    """An accept with the gate down must not read like one that passed.

    This is the whole cost of the flag, and the reason it is not a default: the
    claim "this plan satisfies the mission" is exactly what was skipped, so
    saying it anyway would make the flag a way of faking the result.
    """
    node = unverified_arbiter()
    try:
        node._ltl_gate = None
        response = call(node, request_for(tree=35))
        assert response.accepted
        assert not response.verified
        assert node.get_status()["ltl_unverified"] == 1
        assert node.get_status()["ltl_checked"] == 0
    finally:
        node.destroy_node()


def without_trees(*trees: str) -> str:
    """ACTIVE with those trees no longer objectives. Structurally a fine plan.

    Which is the point: nothing in checks 1-3 has any objection to it. Dropping
    the work is only visible to the check that knows what the mission asked for.
    """
    plan = ACTIVE
    for tree in trees:
        plan = plan.replace(
            f'      <MoveToTreeID name="Visit_Tree_{tree}" '
            f'action_name="follow_tree_id_waypoint" id="{tree}" '
            f'approach_tree="true"/>\n',
            "",
        ).replace(
            f'      <SampleLeaf name="Sample_Leaves_Tree_{tree}" '
            f'action_name="segment_leaves"/>\n',
            "",
        )
    assert plan != ACTIVE, f"nothing was removed for {trees}"
    return plan


def test_verification_off_still_refuses_a_plan_that_abandons_the_mission():
    """The check that ends the local loop does not ride with the formal gate.

    This is the split. Turning SPIN off is a statement about how strong a claim
    the arbiter is making; it is not permission for the planner to make a
    failing plan pass by deleting the work. Before the flags were separated
    this candidate was accepted, the planner never exhausted anything, and the
    fault it was papering over never reached the coordinator.
    """
    node = unverified_arbiter()
    try:
        node._ltl_gate = None  # would raise if the formal gate were consulted
        node.original_objectives = {"10", "60"}
        node.max_droppable = 2

        accepted, reason, abort = node._evaluate(without_trees("60"))
        assert not accepted
        assert not abort, "one unjustified drop is fixable — retry, do not abort"
        assert "unjustified drop" in reason
        assert "60" in reason
    finally:
        node.destroy_node()


def test_verification_off_still_aborts_when_too_little_of_the_mission_survives():
    """And the abort survives too, which is the signal triage escalates on.

    ``/mission/abort`` is the one message that ends local recovery. With the
    two flags fused, a run with SPIN off had no way to produce it at all, so
    nothing was ever handed to the fleet.
    """
    node = unverified_arbiter()
    try:
        node._ltl_gate = None
        node.original_objectives = {"10", "60"}
        node.justified_drops = {"10"}  # already conceded earlier this mission
        node.max_droppable = 1
        node._last_failure_reason = "tree removed from the orchard"  # permits 1 new

        # Gate 1 passes: only tree 60 is newly dropped, and a permanent failure
        # allows one. Gate 2 does not: that makes two gone in total, against a
        # budget of one.
        accepted, reason, abort = node._evaluate(without_trees("10", "60"))
        assert not accepted
        assert abort, "2 drops against a budget of 1 is not something to retry"
        assert "no longer viable" in reason
    finally:
        node.destroy_node()


def test_objective_gating_off_is_the_only_way_to_skip_the_objective_check():
    """The escape hatch still exists — it is just its own flag now."""
    node = unverified_arbiter()
    try:
        node._ltl_gate = None
        node.objective_gating = False
        node.original_objectives = {"10", "60"}
        node.max_droppable = 0

        accepted, reason, abort = node._evaluate(without_trees("10", "60"))
        assert accepted, reason
        assert not abort
    finally:
        node.destroy_node()


def test_verification_off_still_refuses_a_plan_that_cannot_run():
    """The three checks that survive are the ones about running, not meaning.

    A sample with nowhere to sample is not a question about whether the plan
    still matches the mission -- it is a plan that puts the arm out in the
    middle of an aisle. Turning off verification must not turn that off.
    """
    node = unverified_arbiter()
    try:
        node._ltl_gate = None
        orphan = ACTIVE.replace(
            '<SampleLeaf name="Sample_Leaves_Tree_10" action_name="segment_leaves"/>',
            '<SampleLeaf name="Sample_Leaves_Tree_10" action_name="segment_leaves"/>'
            '<MoveToAisleHead name="Leave" action_name="move_to_aisle_head" id="4"/>'
            '<SampleLeaf name="Sample_Nowhere" action_name="segment_leaves"/>',
        )
        accepted, _, reason = node._decide(orphan)
        assert not accepted
        assert "Sample_Nowhere" in reason
    finally:
        node.destroy_node()


#: A plan for the absorbed task and nothing else -- the shape a robot that has
#: finished its own mission and won one at auction actually produces, because
#: the planner prunes completed work before the model ever sees it.
ABSORBED_ONLY = (
    '<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">'
    "<Mission>sample leaves from trees 10 and 60</Mission>"
    '<BehaviorTree ID="Main"><Sequence name="task_43808">'
    '<MoveToAisleHead name="task_43808_aisle" action_name="move_to_aisle_head" id="2"/>'
    '<MoveToTreeID name="task_43808_visit" action_name="follow_tree_id_waypoint"'
    ' id="20" approach_tree="true"/>'
    '<SampleLeaf name="task_43808_sample" action_name="segment_leaves"/>'
    "</Sequence></BehaviorTree></root>"
)


def test_finished_objectives_are_not_drops(arbiter):
    """The rejection that cost a live auction its replan.

    amiga3 sampled trees 24 and 68, won tree 20 from a peer with a broken
    camera, and its planner -- correctly -- sent back a plan holding only the
    absorbed task, because finished work is pruned before the model sees it
    (the executor re-ticks from the root, so anything left in the plan is
    driven again). The arbiter compared that against the objectives captured at
    mission start, saw both trees missing, and rejected the edit as an
    unjustified drop of the two trees the robot had just finished.

    A finished objective is not an abandoned one. Nothing about the candidate
    changes here -- only whether the arbiter was told the work had happened.
    """
    doc = etree.fromstring(ABSORBED_ONLY.encode("utf-8"))

    ok, reason, abort = arbiter._check_objective_preserved(doc)
    assert not ok
    assert "unjustified drop" in reason
    assert not abort, "a fixable rejection, never an abort"

    arbiter.completed_objectives = {"10", "60"}
    ok, reason, abort = arbiter._check_objective_preserved(doc)
    assert ok, reason


def test_absorbing_work_does_not_re_run_finished_trees(arbiter):
    """The three minutes of driving that a live run spent going nowhere.

    amiga2 sampled tree 58, sampled tree 64, then won tree 20 at auction. The
    plan it was handed still opened with tree 58 and tree 64, so it re-entered
    its own aisle, drove to 58, sampled it again, drove to 64, sampled it again,
    and only then set off for the tree it had actually won.

    Nothing was wrong with the ledger -- the planner had already logged both
    trees as complete. The arbiter simply grafted the new task onto the plan as
    written, and bt_runner ticks every published plan from the root.
    """
    arbiter.orchard = real_orchard()
    arbiter.completed_objectives = {"10"}

    edited, _clause, _dropped = arbiter._apply_task_edit(
        request_for(task_id=43808, tree=20), ACTIVE
    )

    approached = {
        element.get("id")
        for element in etree.fromstring(edited.encode("utf-8")).iter("MoveToTreeID")
        if (element.get("approach_tree") or "").lower() == "true"
    }
    assert "10" not in approached, "already sampled -- driving there again is waste"
    assert {"60", "20"} <= approached, "pending work and the won task both survive"


def test_absorbing_work_keeps_a_plan_whose_trees_are_all_pending(arbiter):
    """The pruning only ever removes what is provably done. With an empty
    ledger the plan reaching insert_task is the plan as written, so a robot
    that has finished nothing absorbs work exactly as it did before."""
    arbiter.orchard = real_orchard()

    edited, _clause, _dropped = arbiter._apply_task_edit(
        request_for(task_id=43808, tree=20), ACTIVE
    )

    approached = {
        element.get("id")
        for element in etree.fromstring(edited.encode("utf-8")).iter("MoveToTreeID")
        if (element.get("approach_tree") or "").lower() == "true"
    }
    assert {"10", "60", "20"} <= approached


def _tree_status(status):
    from std_msgs.msg import String as _String

    return _String(data=json.dumps({"node": "<tree>", "status": status, "reason": ""}))


def _published(node):
    """Everything the arbiter puts on /mission/xml. Returns a live list."""
    sent = []
    node.mission_pub.publish = lambda msg: sent.append(msg.data)
    return sent


def test_a_plan_accepted_mid_mission_waits_for_the_mission_to_end(arbiter):
    """bt_runner cannot take a plan while it is running one.

    Its tick loop only exits when the tree returns, so a plan published in the
    middle of a mission does not start any sooner -- it sits in a single
    pending slot, going stale, until the executor comes back for it. Holding it
    here costs nothing for exactly that reason, and buys the chance to prune it
    against what finished in the meantime.
    """
    sent = _published(arbiter)
    arbiter._executor_busy = True

    arbiter._deliver("<root>held</root>")
    assert sent == [], "published into a mission that cannot adopt it"
    assert arbiter._held_plan == "<root>held</root>"

    arbiter._on_bt_status(_tree_status("SUCCESS"))
    assert sent == ["<root>held</root>"], "not released when the mission ended"
    assert arbiter._held_plan is None


def test_a_held_plan_is_pruned_against_what_finished_while_it_waited(arbiter):
    """The re-sampling this exists to stop.

    amiga3's last plan was written at 09:44:08, when tree 26 was genuinely
    still to do. Tree 26 was sampled at 09:45:39, and that same plan was
    adopted in the same instant -- so the robot drove back and sampled it
    again. The plan was correct when written and stale by the time it ran, and
    nothing looked at it in between.
    """
    arbiter.orchard = real_orchard()
    arbiter._executor_busy = True
    sent = _published(arbiter)

    arbiter._deliver(ACTIVE)
    arbiter.completed_objectives = {"10"}  # finished while the plan waited
    arbiter._on_bt_status(_tree_status("SUCCESS"))

    assert len(sent) == 1
    approached = {
        el.get("id")
        for el in etree.fromstring(sent[0].encode("utf-8")).iter("MoveToTreeID")
        if (el.get("approach_tree") or "").lower() == "true"
    }
    assert "10" not in approached, "a finished tree survived into the running plan"
    assert "60" in approached, "pending work must not go with it"


def test_a_failed_mission_releases_the_hold_too(arbiter):
    """FAILURE ends a mission exactly as SUCCESS does, and the executor comes
    back for a plan either way. Only SUCCESS releasing it would strand every
    plan written for a robot whose tree failed -- which is most of them."""
    sent = _published(arbiter)
    arbiter._executor_busy = True
    arbiter._deliver("<root>held</root>")

    arbiter._on_bt_status(_tree_status("FAILURE"))
    assert sent == ["<root>held</root>"]


def test_an_idle_robot_is_handed_its_plan_straight_away(arbiter):
    """Nothing is deferred that does not need to be. With no mission running
    there is nothing to wait for, and waiting would be the one way this could
    leave a robot idle holding work."""
    sent = _published(arbiter)
    arbiter._executor_busy = False

    arbiter._deliver("<root>now</root>")
    assert sent == ["<root>now</root>"]
    assert arbiter._held_plan is None


def test_a_hold_nobody_ends_fails_open(arbiter):
    """The mission-end signal is one message from one node, and nothing here
    can prove it arrives. Timing out publishes the plan into the pending slot
    it would have gone to anyway, so the failure mode is the behaviour this
    replaced rather than a robot waiting forever."""
    from amiga_ros2_agents.replanning import arbiter_node as mod

    sent = _published(arbiter)
    arbiter._executor_busy = True
    arbiter._deliver("<root>held</root>")

    arbiter._release_stale_hold()
    assert sent == [], "released before the timeout"

    arbiter._held_since -= mod.HOLD_TIMEOUT_SEC + 1
    arbiter._release_stale_hold()
    assert sent == ["<root>held</root>"]


def test_completion_ledger_is_read_off_bt_status(arbiter):
    """Populated from the SUCCESS half of /bt/status_change, by the same
    binding the planner uses -- so the two agree by construction. A SampleLeaf
    reports its own name; the tree id lives on the MoveToTreeID ahead of it."""
    import json as _json

    from std_msgs.msg import String as _String

    def _status(node, status):
        return _String(data=_json.dumps({"node": node, "status": status, "reason": ""}))

    arbiter._on_bt_status(_status("Sample_Leaves_Tree_10", "SUCCESS"))
    assert arbiter.completed_objectives == {"10"}

    # A failure must not land in the ledger -- it is the reason channel.
    arbiter._on_bt_status(_status("Sample_Leaves_Tree_60", "FAILURE"))
    assert arbiter.completed_objectives == {"10"}

    arbiter._on_bt_status(_status("Sample_Leaves_Tree_60", "SUCCESS"))
    assert arbiter.completed_objectives == {"10", "60"}


def _task_id_of(mission_xml: str, node_name: str) -> int:
    from amiga_ros2_agents.mission import mission_tasks

    task = mission_tasks.task_for_node(mission_xml, node_name)
    assert task is not None, f"no task around {node_name}"
    return task.task_id
