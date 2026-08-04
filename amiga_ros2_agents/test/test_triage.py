"""Tests for the triage agent — the first of the two reasoning points.

What is worth testing about a node whose interesting behaviour is a language
model's? Everything around it. The model's answer is not deterministic, but
these are, and each one is a way the pipeline could quietly do the wrong thing:

    * the escalation trigger, which is deterministic on purpose and must fire
      on a give-up and stay silent otherwise;
    * the task-id lookup, which is the bridge between a BT node name and a
      number small enough to put on a LoRa frame;
    * the decision parser, which is the guard keeping free text out of a state
      machine -- the constraint the whole design rests on.

The model is replaced by a function returning a fixed string. That is not a
weaker test than calling a real one: what is under test is what happens to the
reply, and a real model would only make the input less controlled.
"""

import json

import pytest

from amiga_ros2_agents import triage_node as tn


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(tn.llm, "MODEL", "test-model")
    created = tn.TriageNode()
    yield created
    created.destroy_node()


def reply(**fields) -> str:
    return json.dumps(fields)


MISSION_XML = """<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <MoveToTreeID name="Visit_Tree_60" action_name="follow_tree_id_waypoint"
                    id="60" approach_tree="true"/>
      <SampleLeaf name="Sample_60" action_name="segment_leaves"/>
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
            reason_code="low_battery",
            fallback="hold",
            rationale="9% battery and two idle peers",
        )
    )
    assert decision["action"] == "re_delegate"
    assert decision["reason_code"] == tn.REASON_CODES["low_battery"]
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
            reason_code="operator",
            task_id=77,
            capability=2,
            grid_row=4,
            grid_col=9,
            priority=200,
            rationale="found an unsprayed row",
        )
    )
    assert decision["task_id"] == 77
    assert decision["required_capability"] == 2
    assert (decision["grid_row"], decision["grid_col"]) == (4, 9)
    assert decision["priority"] == 200


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


def test_add_task_does_not_inherit_the_failed_tasks_id(node, monkeypatch):
    # An add_task that silently reuses the failing task id turns "I found other
    # work" into "re-add the thing that just failed", and nothing downstream
    # can tell the two apart.
    monkeypatch.setattr(
        tn.llm,
        "complete",
        lambda system, user, **kw: reply(
            action="add_task", reason_code="operator", rationale="no id given"
        ),
    )
    response = node._on_interpret(_request(task_id=60), _response())

    assert response.ok is True
    assert response.action == "add_task"
    assert response.task_id == 0, "must not echo the failed task"


# ==========================================================================
# The task-id bridge: BT node name -> wire task id
# ==========================================================================


def test_the_failing_nodes_tree_id_becomes_the_task_id(node):
    task_id = node._task_id_for({"node": "Visit_Tree_60"}, MISSION_XML)
    assert task_id == 60


def test_a_node_with_no_id_attribute_has_no_task(node):
    # SampleLeaf carries no id. Reporting 0 is honest; inventing one would have
    # the coordinator announce a task nobody can match to any work.
    assert node._task_id_for({"node": "Sample_60"}, MISSION_XML) == 0


def test_a_fault_with_no_mission_loaded_has_no_task(node):
    assert node._task_id_for({"node": "Visit_Tree_60"}, None) == 0


def test_a_tree_level_fault_has_no_task(node):
    assert node._task_id_for({"node": "<tree>"}, MISSION_XML) == 0


def test_unparseable_mission_xml_does_not_raise(node):
    assert node._task_id_for({"node": "Visit_Tree_60"}, "<not xml") == 0


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
    assert sent[0]["task_id"] == 60
    assert "viability budget" in sent[0]["detail"]


def test_a_planner_give_up_escalates(node, monkeypatch):
    sent = escalations(node, monkeypatch)
    node._on_planner_status(
        _string(json.dumps({"event": "gave_up", "reason": "3 rejections"}))
    )
    assert len(sent) == 1
    assert sent[0]["cause"] == "planner_gave_up"


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
            reason_code="low_battery",
            fallback="hold",
            rationale="battery at 9%",
        ),
    )
    response = node._on_interpret(_request(task_id=60), _response())

    assert response.ok is True
    assert response.action == "re_delegate"
    assert response.fallback == "hold"
    assert response.reason_code == tn.REASON_CODES["low_battery"]
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
