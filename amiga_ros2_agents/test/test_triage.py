"""Tests for the triage agent — the first of the two reasoning points.

What is worth testing about a node whose interesting behaviour is a language
model's? Everything around it. The model's answer is not deterministic, but
these are, and each one is a way the pipeline could quietly do the wrong thing:

    * the escalation trigger, which is deterministic on purpose and must fire
      on a give-up and stay silent otherwise;
    * the task lookup, which is the bridge between a failing BT node and the
      unit of work the fleet can actually be offered -- a subtree, not a leaf;
    * the decision parser, which is the guard keeping free text out of a state
      machine -- the constraint the whole design rests on.

The model is replaced by a function returning a fixed string. That is not a
weaker test than calling a real one: what is under test is what happens to the
reply, and a real model would only make the input less controlled.
"""

import json

import pytest

from amiga_ros2_agents.coordination import triage_node as tn
from amiga_ros2_comms.codec import Capability, Target, TargetKind, cap_mask


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(tn.llm, "MODEL", "test-model")
    created = tn.TriageNode()
    yield created
    created.destroy_node()


def reply(**fields) -> str:
    return json.dumps(fields)


#: examples/sample_leafs.xml, trimmed to two units of work. Deliberately the
#: real file's shape: two tree visits with a sample at each, and a transit move
#: through tree 2 in between -- which is what makes tree ids unusable as task
#: ids, since tree 2 is passed and never worked on.
MISSION_XML = """<?xml version="1.0"?>
<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">
  <Mission>sample leaves from trees 10 and 60</Mission>
  <BehaviorTree ID="Sample_Leaves_Trees_10_60">
    <Sequence>
      <Sequence name="Sample_Tree_10">
        <MoveToTreeID name="Visit_Tree_10" action_name="follow_tree_id_waypoint"
                      id="10" approach_tree="true"/>
        <SampleLeaf name="Sample_Leaves_Tree_10" action_name="segment_leaves"/>
      </Sequence>
      <Sequence name="Traverse_To_Column_4">
        <MoveToTreeID name="Exit_To_Top_Headland" action_name="follow_tree_id_waypoint"
                      id="2" approach_tree="false"/>
      </Sequence>
      <Sequence name="Sample_Tree_60">
        <MoveToTreeID name="Visit_Tree_60" action_name="follow_tree_id_waypoint"
                      id="60" approach_tree="true"/>
        <SampleLeaf name="Sample_Leaves_Tree_60" action_name="segment_leaves"/>
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>
"""


# ==========================================================================
# The decision parser: the guard on the closed union
# ==========================================================================


def test_a_well_formed_re_delegate_is_accepted(node):
    decision = node._parse_decision(
        reply(
            action="re_delegate",
            reason_code="battery_low",
            fallback="hold",
            rationale="9% battery and two idle peers",
        )
    )
    assert decision["action"] == "re_delegate"
    assert decision["reason_code"] == tn.REASON_CODES["battery_low"]
    assert decision["fallback"] == "hold"
    # Only meaningful for drop_task, and blanked so a reader cannot mistake a
    # default for a decision the agent made.
    assert decision["disposition"] == ""
    assert "battery" in decision["rationale"]


def test_a_reply_wrapped_in_prose_is_still_parsed(node):
    # Models add a sentence either side of the object more often than not, and
    # refusing those would mean refusing most well-reasoned answers.
    decision = node._parse_decision(
        "Looking at the logs, I think:\n"
        '{"action": "drop_task", "reason_code": "unreachable", '
        '"disposition": "drop", "rationale": "the tree is gone"}\n'
        "Hope that helps!"
    )
    assert decision["action"] == "drop_task"
    assert decision["disposition"] == "drop"
    assert decision["fallback"] == ""


def test_a_reply_in_a_code_fence_is_parsed(node):
    decision = node._parse_decision(
        '```json\n{"action": "re_delegate", "reason_code": "task_failed", '
        '"fallback": "request_human", "rationale": "arm fault"}\n```'
    )
    assert decision["fallback"] == "request_human"


def test_free_text_is_refused_rather_than_interpreted(node):
    # The whole point of the schema. A sentence must not become an action.
    with pytest.raises(ValueError, match="no JSON object"):
        node._parse_decision(
            "I would re-delegate this task to another robot, since the battery "
            "is low and peers are available."
        )


def test_an_action_outside_the_schema_is_refused(node):
    with pytest.raises(ValueError, match="not one of"):
        node._parse_decision(reply(action="add_constraint", reason_code="unspecified"))


def test_an_invented_reason_code_is_refused(node):
    with pytest.raises(ValueError, match="reason_code"):
        node._parse_decision(reply(action="re_delegate", reason_code="cosmic_rays"))


def test_a_re_delegate_with_a_nonsense_fallback_is_refused(node):
    # Not silently defaulted to HOLD: the fallback decides what happens to work
    # nobody bid on, and inventing it is a decision made by no one.
    with pytest.raises(ValueError, match="fallback"):
        node._parse_decision(
            reply(action="re_delegate", reason_code="task_failed", fallback="retry")
        )


def test_a_drop_task_with_a_nonsense_disposition_is_refused(node):
    with pytest.raises(ValueError, match="disposition"):
        node._parse_decision(
            reply(action="drop_task", reason_code="task_failed", disposition="delete")
        )


def test_add_task_fields_survive_the_parse(node):
    decision = node._parse_decision(
        reply(
            action="add_task",
            reason_code="operator_request",
            task_id=77,
            actions=["MoveToTreeID", "SampleLeaf"],
            target={"kind": "tree", "index": 47},
            priority=200,
            rationale="tree 47 was passed unsampled",
        )
    )
    assert decision["task_id"] == 77
    assert decision["capabilities"] == cap_mask(
        Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF
    )
    assert decision["target"] == Target.tree(47)
    assert decision["priority"] == 200


def test_an_action_the_mission_schema_does_not_have_is_refused(node):
    """Refused, not dropped.

    Silently ignoring an unknown action would build a mask describing *less*
    work than the model asked for, and the task would then be announced as
    feasible for robots that cannot do it -- with the announcement looking
    entirely ordinary.
    """
    with pytest.raises(ValueError, match="not in the mission schema"):
        node._parse_decision(
            reply(
                action="add_task",
                reason_code="unspecified",
                task_id=77,
                actions=["MoveToTreeID", "SprayTree"],
            )
        )


def test_a_gps_target_survives_the_parse(node):
    decision = node._parse_decision(
        reply(
            action="add_task",
            reason_code="unspecified",
            task_id=78,
            actions=["MoveToGPSLocation"],
            target={
                "kind": "gps",
                "latitude": 37.366449,
                "longitude": -120.423065,
            },
        )
    )
    assert decision["target"] == Target.gps(37.366449, -120.423065)


@pytest.mark.parametrize(
    "target",
    [
        {"kind": "orchard", "index": 3},  # not a kind
        {"kind": "tree"},  # no index
        {"kind": "gps", "latitude": 37.4},  # half a fix
        {"kind": "gps", "latitude": 200.0, "longitude": 0.0},  # off the globe
        "tree 47",  # not an object
    ],
)
def test_a_target_that_is_not_a_place_is_refused(node, target):
    # Same reasoning as the actions. A place quietly coerced to "here" is work
    # announced somewhere every listener resolves differently.
    with pytest.raises(ValueError):
        node._parse_decision(
            reply(
                action="add_task",
                reason_code="unspecified",
                task_id=77,
                actions=["SampleLeaf"],
                target=target,
            )
        )


def test_out_of_range_task_fields_read_as_absent(node):
    # They are only used for add_task, and the caller substitutes the request's
    # own values when they come back 0. A model omitting them on a re_delegate
    # -- which is most of the time -- must not fail the parse.
    decision = node._parse_decision(
        reply(action="add_task", reason_code="unspecified", task_id=99999, priority=-4)
    )
    assert decision["task_id"] == 0
    assert decision["priority"] == 0


def test_a_list_of_actions_is_refused(node):
    # Which one? Taking the first is a decision the agent did not make.
    with pytest.raises(ValueError, match="list of them"):
        node._parse_decision(
            '[{"action": "re_delegate", "reason_code": "task_failed"}, '
            '{"action": "drop_task", "reason_code": "task_failed"}]'
        )


# ==========================================================================
# The note: free text that rides with an announcement
# ==========================================================================


def test_a_note_on_a_re_delegate_survives_the_parse(node):
    decision = node._parse_decision(
        reply(
            action="re_delegate",
            reason_code="task_failed",
            fallback="hold",
            note="north end of row 7 is flooded; approach from the south",
        )
    )
    assert decision["note"].startswith("north end of row 7 is flooded")


def test_a_note_on_anything_but_a_re_delegate_is_dropped(node):
    # There is no announcement for it to ride on, so it would be text
    # addressed to nobody -- and a drop_task that carried one would look, in
    # the counters, like a note that was sent.
    decision = node._parse_decision(
        reply(
            action="drop_task",
            reason_code="task_failed",
            disposition="drop",
            note="the tree was felled last season",
        )
    )
    assert decision["note"] == ""


def test_an_absent_note_is_the_normal_case_and_parses_to_empty(node):
    decision = node._parse_decision(
        reply(action="re_delegate", reason_code="task_failed", fallback="hold")
    )
    assert decision["note"] == ""


def test_a_note_too_long_for_the_radio_is_refused_rather_than_cut(node):
    # Refusing fails the whole interpretation, and the coordinator then leaves
    # the task alone. That is louder than an announcement that quietly lost
    # half its context -- and where to cut a sentence is a decision with
    # meaning that neither this parser nor the codec is able to make.
    with pytest.raises(ValueError, match="Say less"):
        node._parse_decision(
            reply(
                action="re_delegate",
                reason_code="task_failed",
                fallback="hold",
                note="x" * (tn.NOTE_MAX_BYTES + 1),
            )
        )


def test_the_note_budget_comes_from_the_radio_and_not_from_a_constant(node):
    # 328 bytes at the 50-byte payload default: eight fragments of 41. If the
    # payload budget moves, this moves with it -- the prompt is rendered from
    # the same number, so the model is asked for what will actually fit.
    assert tn.NOTE_MAX_BYTES == 8 * 41


def test_add_task_does_not_inherit_the_failed_tasks_id(node, monkeypatch):
    # An add_task that silently reuses the failing task id turns "I found other
    # work" into "re-add the thing that just failed", and nothing downstream
    # can tell the two apart.
    monkeypatch.setattr(
        tn.llm,
        "complete",
        lambda system, user, **kw: reply(
            action="add_task", reason_code="operator_request", rationale="no id given"
        ),
    )
    response = node._on_interpret(_request(task_id=60), _response())

    assert response.ok is True
    assert response.action == "add_task"
    assert response.task_id == 0, "must not echo the failed task"


# ==========================================================================
# The task bridge: a failing BT node -> the unit of work it belongs to
# ==========================================================================


def test_a_failing_move_resolves_to_the_whole_sampling_subtree(node):
    """The failing leaf is the move; the task is the move *and* the sample.

    Offering the fleet only the leaf that failed would hand a peer a drive to
    tree 60 with nothing to do when it arrives, and leave the sample behind on
    a robot that has already given up on getting there.
    """
    task = node._task_for({"node": "Visit_Tree_60"}, MISSION_XML)

    assert task is not None
    assert task.name == "Sample_Tree_60"
    assert task.capabilities == cap_mask(
        Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF
    )
    assert task.target == Target.tree(60)


def test_a_failing_sample_resolves_to_the_same_subtree(node):
    """Either half of the unit resolves to the whole unit.

    SampleLeaf carries no id at all. Under the old lookup this was task 0 --
    "no task attached" -- so a sampling failure could never be delegated by
    anyone, which is exactly the work this fleet exists to move around.
    """
    move = node._task_for({"node": "Visit_Tree_60"}, MISSION_XML)
    sample = node._task_for({"node": "Sample_Leaves_Tree_60"}, MISSION_XML)

    assert sample is not None
    assert sample.task_id == move.task_id
    assert sample.target == Target.tree(60)


def test_two_units_in_one_mission_get_different_ids(node):
    """The bug the tree id had: it is a location, and locations repeat."""
    ten = node._task_for({"node": "Visit_Tree_10"}, MISSION_XML)
    sixty = node._task_for({"node": "Visit_Tree_60"}, MISSION_XML)

    assert ten.task_id != sixty.task_id
    assert ten.target == Target.tree(10)
    assert sixty.target == Target.tree(60)


def test_a_transit_move_is_not_an_objective(node):
    """approach_tree="false" is how the robot gets somewhere, not what it does.

    This is the arbiter's own definition of an objective, reused rather than
    reinvented -- if the two disagreed, the fleet and the plan validator would
    disagree about which trees a mission is actually for.
    """
    transit = node._task_for({"node": "Exit_To_Top_Headland"}, MISSION_XML)

    assert transit is not None
    assert not transit.delegable, "a transit move names nowhere to be sent"


def test_the_task_id_is_stable_across_reads(node):
    """Two robots reading the same plan must agree without talking about it."""
    first = node._task_for({"node": "Visit_Tree_60"}, MISSION_XML)
    second = node._task_for({"node": "Visit_Tree_60"}, MISSION_XML)
    assert first.task_id == second.task_id
    assert 0 < first.task_id <= 0xFFFF


def test_a_fault_with_no_mission_loaded_has_no_task(node):
    assert node._task_for({"node": "Visit_Tree_60"}, None) is None


def test_a_tree_level_fault_has_no_task(node):
    assert node._task_for({"node": "<tree>"}, MISSION_XML) is None


def test_unparseable_mission_xml_does_not_raise(node):
    assert node._task_for({"node": "Visit_Tree_60"}, "<not xml") is None


# ==========================================================================
# Escalation: deterministic, and only on a give-up
# ==========================================================================


def escalations(node, monkeypatch):
    """Capture what would go out on /coordination/infeasible."""
    sent = []
    monkeypatch.setattr(
        node.infeasible_pub, "publish", lambda msg: sent.append(json.loads(msg.data))
    )
    return sent


def test_an_arbiter_abort_escalates(node, monkeypatch):
    sent = escalations(node, monkeypatch)
    node._on_mission(_string(MISSION_XML))
    node._on_fault(_string(json.dumps({"node": "Visit_Tree_60", "timestamp_ms": 0})))

    node._on_abort(_string(json.dumps({"reason": "viability budget exceeded"})))

    assert len(sent) == 1
    assert sent[0]["cause"] == "arbiter_abort"
    assert "viability budget" in sent[0]["detail"]
    # The whole descriptor, not just an id. This node is the only thing that
    # reads the mission XML, so it is the only thing that can say what the
    # failed work actually is -- and a coordinator sent only an id would have
    # to invent both the action set and the place.
    assert sent[0]["name"] == "Sample_Tree_60"
    assert sent[0]["capabilities"] == cap_mask(
        Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF
    )
    assert sent[0]["target_kind"] == int(TargetKind.TREE)
    assert sent[0]["target_a"] == 60
    assert sent[0]["task_id"] > 0


def test_a_planner_give_up_escalates(node, monkeypatch):
    sent = escalations(node, monkeypatch)
    node._on_planner_status(
        _string(json.dumps({"event": "gave_up", "reason": "3 rejections"}))
    )
    assert len(sent) == 1
    assert sent[0]["cause"] == "planner_gave_up"


@pytest.mark.parametrize(
    "cause",
    ["rejection_retries_exhausted", "max_retries_exhausted"],
)
def test_the_payload_the_planner_actually_publishes_escalates(node, monkeypatch, cause):
    """The give-up contract, taken from the producer rather than restated.

    This is the seam the whole pipeline hangs off: a fault this robot cannot fix
    only reaches the fleet because triage recognises the planner saying it has
    stopped. It was broken for exactly this reason -- the planner published
    ``outcome`` and this node read ``event``, so the branch below never ran in
    any configuration, while the test above passed because it wrote the payload
    the consumer wanted instead of the one the producer sends.

    So the message here is produced by ``MissionPlannerNode._report_gave_up``
    itself. If either side renames a key again, this fails; a test that spelled
    the JSON out by hand would not.
    """
    from amiga_ros2_agents.replanning.mission_planner_node import MissionPlannerNode

    published = []
    planner = MissionPlannerNode()
    try:
        monkeypatch.setattr(
            planner.status_pub, "publish", lambda msg: published.append(msg.data)
        )
        planner._report_gave_up(
            cause=cause, node="Visit_Tree_60", reason="the arbiter refused it thrice"
        )
    finally:
        planner.destroy_node()

    assert len(published) == 1, "the planner did not report giving up at all"

    sent = escalations(node, monkeypatch)
    node._on_planner_status(_string(published[0]))

    assert len(sent) == 1, f"triage did not escalate on {published[0]}"
    assert sent[0]["cause"] == "planner_gave_up"
    # The arbiter's own words survive the hop. Without this the escalation --
    # and so the triage prompt that reads it -- says only "gave up".
    assert "the arbiter refused it thrice" in sent[0]["detail"]


def test_a_give_up_is_reported_once_however_often_the_planner_stops(monkeypatch):
    """Every later BT failure re-enters the planner; the give-up is still one.

    Without the dedup each subsequent fault would re-announce an exhausted
    budget, and each one would be a fresh escalation of work the fleet is
    already deciding about.
    """
    from amiga_ros2_agents.replanning.mission_planner_node import MissionPlannerNode

    published = []
    planner = MissionPlannerNode()
    try:
        monkeypatch.setattr(
            planner.status_pub, "publish", lambda msg: published.append(msg.data)
        )
        for _ in range(3):
            planner._report_gave_up(
                cause="rejection_retries_exhausted",
                node="Visit_Tree_60",
                reason="out of budget",
            )
        # Rejection retries are counted per node, so a different node that also
        # runs out is a second, genuine give-up.
        planner._report_gave_up(
            cause="rejection_retries_exhausted",
            node="Visit_Tree_10",
            reason="out of budget",
        )
    finally:
        planner.destroy_node()

    assert len(published) == 2


def test_running_out_of_sessions_escalates_once_not_once_per_failing_node(monkeypatch):
    """The shape a live fleet run actually produced, before this was keyed right.

    MAX_RETRIES counts planning sessions for the whole mission, not attempts at
    one node. And a leaf failure is *always* followed by the tree's own FAILURE
    event -- see test_scenario_bt_fault -- so whichever leaf was last in, the
    planner is re-entered a moment later with node="<tree>". Keying the dedup on
    the node made those two different give-ups: the run escalated the same
    exhausted budget twice, milliseconds apart, and the coordinator opened two
    interpretations of one fault.
    """
    from amiga_ros2_agents.replanning.mission_planner_node import MissionPlannerNode

    published = []
    planner = MissionPlannerNode()
    try:
        monkeypatch.setattr(
            planner.status_pub, "publish", lambda msg: published.append(msg.data)
        )
        for node_name in ("EnterRow2", "<tree>", "ApproachTree20"):
            planner._report_gave_up(
                cause="max_retries_exhausted",
                node=node_name,
                reason="reached MAX_RETRIES (20) planning sessions",
            )
    finally:
        planner.destroy_node()

    assert len(published) == 1


def test_ordinary_planner_progress_does_not_escalate(node, monkeypatch):
    # The planner publishes this topic for terminal outcomes, but the loop is
    # allowed to keep trying. Escalating on every status message would put a
    # task on the radio the robot was still perfectly capable of finishing.
    sent = escalations(node, monkeypatch)
    node._on_planner_status(_string(json.dumps({"event": "candidate_published"})))
    node._on_planner_status(_string(json.dumps({"event": "retrying"})))
    assert sent == []


def test_a_bt_fault_alone_does_not_escalate(node, monkeypatch):
    # A fault starts the *local* loop. Coordination is only for what that loop
    # could not fix, so the tree failing must not reach the radio by itself.
    sent = escalations(node, monkeypatch)
    node._on_fault(_string(json.dumps({"node": "Visit_Tree_60", "timestamp_ms": 0})))
    assert sent == []
    assert node.get_status()["faults_seen"] == 1


def test_a_malformed_fault_payload_is_ignored_not_fatal(node, monkeypatch):
    sent = escalations(node, monkeypatch)
    node._on_fault(_string("not json at all"))
    node._on_fault(_string('"a bare string"'))
    assert node.last_fault is None
    assert sent == []


# ==========================================================================
# The service, end to end with the model replaced
# ==========================================================================


def test_a_good_interpretation_fills_the_response(node, monkeypatch):
    monkeypatch.setattr(
        tn.llm,
        "complete",
        lambda system, user, **kw: reply(
            action="re_delegate",
            reason_code="battery_low",
            fallback="hold",
            rationale="battery at 9%",
        ),
    )
    response = node._on_interpret(_request(task_id=60), _response())

    assert response.ok is True
    assert response.action == "re_delegate"
    assert response.fallback == "hold"
    assert response.reason_code == tn.REASON_CODES["battery_low"]
    # Echoed back from the request, so the response reads as self-contained.
    assert response.task_id == 60
    assert response.model == "test-model"


def test_a_refused_interpretation_says_why_instead_of_raising(node, monkeypatch):
    monkeypatch.setattr(
        tn.llm, "complete", lambda system, user, **kw: "sure, I'd delegate that"
    )
    response = node._on_interpret(_request(task_id=60), _response())

    assert response.ok is False
    assert response.action == ""
    assert "no JSON object" in response.error
    assert node.get_status()["refused"] == 1


def test_a_model_that_raises_does_not_take_the_service_down(node, monkeypatch):
    def explode(system, user, **kw):
        raise RuntimeError("endpoint refused the connection")

    monkeypatch.setattr(tn.llm, "complete", explode)
    response = node._on_interpret(_request(task_id=60), _response())

    assert response.ok is False
    assert "endpoint refused" in response.error


def test_the_prompt_carries_the_evidence_the_node_gathered(node, monkeypatch):
    node._on_mission(_string(MISSION_XML))
    node._on_fault(
        _string(json.dumps({"node": "Visit_Tree_60", "timestamp_ms": 0, "uid": 3}))
    )
    node._on_world_state(_string('{"battery": 9, "row": 4}'))

    seen = {}

    def capture(system, user, **kw):
        seen["system"] = system
        seen["user"] = user
        return reply(action="drop_task", reason_code="unreachable", disposition="drop")

    monkeypatch.setattr(tn.llm, "complete", capture)
    node._on_interpret(_request(task_id=60), _response())

    assert "Visit_Tree_60" in seen["user"]
    assert '"battery": 9' in seen["user"]
    # The peer list is load-bearing: re_delegate is only sensible with someone
    # to delegate to, and the prompt says so.
    assert "Peers alive right now" in seen["user"]
    for action in tn.VALID_ACTIONS:
        assert action in seen["system"]


# --------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------


def _string(data: str):
    from std_msgs.msg import String

    return String(data=data)


def _request(**fields):
    from amiga_interfaces.srv import InterpretAnomaly

    request = InterpretAnomaly.Request()
    for name, value in fields.items():
        setattr(request, name, value)
    return request


def _response():
    from amiga_interfaces.srv import InterpretAnomaly

    return InterpretAnomaly.Response()
