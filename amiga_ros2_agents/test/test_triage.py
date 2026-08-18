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
import time

import pytest

from amiga_ros2_agents.coordination import triage_node as tn
from amiga_ros2_comms.codec import Capability, Target, TargetKind, cap_mask


class _Inline:
    """A Thread that runs its target on ``start()``.

    Routing happens on a background thread so the executor keeps spinning while
    a model is thinking. That is right in the node and useless in a test: a
    verdict that lands after the assertions is a flake, and a thread still
    publishing after ``destroy_node`` is a crash in a daemon nobody reads. The
    test wants the same work, finished before the call returns.
    """

    def __init__(self, target=None, args=(), kwargs=None, daemon=None):
        self._call = lambda: target(*args, **(kwargs or {}))

    def start(self):
        self._call()


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(tn.llm, "MODEL", "test-model")
    monkeypatch.setattr(tn, "Thread", _Inline)
    # Every /bt/status_change FAILURE now costs a routing call, so a test that
    # is about something else would otherwise reach for a real endpoint. The
    # default verdict is the one that changes nothing downstream.
    monkeypatch.setattr(
        tn.llm,
        "complete",
        lambda system, user, **kw: route_reply(
            route="repair", reason_code="unspecified", rationale="default test verdict"
        ),
    )
    created = tn.TriageNode()
    yield created
    created.destroy_node()


def reply(**fields) -> str:
    return json.dumps(fields)


def route_reply(**fields) -> str:
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
# Routing: the judgement that used to be a retry counter
# ==========================================================================


def routes(node, monkeypatch):
    """Capture what would go out on /mission/fault_route."""
    sent = []
    monkeypatch.setattr(
        node.route_pub, "publish", lambda msg: sent.append(json.loads(msg.data))
    )
    return sent


def _model(monkeypatch, *replies):
    """Answer each model call with the next reply, and count the calls."""
    calls = []

    def answer(system, user, **kw):
        calls.append(user)
        return replies[min(len(calls) - 1, len(replies) - 1)]

    monkeypatch.setattr(tn.llm, "complete", answer)
    return calls


FAULT = json.dumps(
    {
        "node": "Sample_Leaves_Tree_60",
        "status": "FAILURE",
        "source": "leaf",
        "timestamp_ms": 0,
    }
)


def test_a_repair_verdict_is_published_and_escalates_nothing(node, monkeypatch):
    published = routes(node, monkeypatch)
    sent = escalations(node, monkeypatch)
    _model(
        monkeypatch,
        route_reply(
            route="repair",
            reason_code="unspecified",
            guidance="the arm was still stowed",
            rationale="a different plan could order this correctly",
        ),
    )
    node._on_mission(_string(MISSION_XML))

    node._on_fault(_string(FAULT))

    assert len(published) == 1
    assert published[0]["route"] == "repair"
    assert published[0]["node"] == "Sample_Leaves_Tree_60"
    assert published[0]["guidance"] == "the arm was still stowed"
    assert sent == [], "a repairable fault must not reach the fleet"


def test_an_escalate_verdict_sheds_the_work_immediately(node, monkeypatch):
    """The whole point of routing first: the plan is still the one that failed.

    This is what a give-up escalation cannot do. By the time MAX_RETRIES runs
    out the planner has replaced /mission/xml several times over, and the node
    named in the fault is no longer in it -- so the escalation carries task_id
    0 and the coordinator has nothing it can announce.
    """
    published = routes(node, monkeypatch)
    sent = escalations(node, monkeypatch)
    _model(
        monkeypatch,
        route_reply(
            route="escalate",
            reason_code="capability_missing",
            rationale="no point cloud on any attempt; the camera is dead",
        ),
    )
    node._on_mission(_string(MISSION_XML))

    node._on_fault(_string(FAULT))

    assert published[0]["route"] == "escalate"
    assert len(sent) == 1
    assert sent[0]["cause"] == "triage_route"
    assert "camera is dead" in sent[0]["detail"]
    # Announceable: the whole descriptor, resolved against the plan that was
    # running when the leaf failed.
    assert sent[0]["task_id"] > 0
    assert sent[0]["name"] == "Sample_Tree_60"
    assert sent[0]["capabilities"] == cap_mask(
        Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF
    )
    assert sent[0]["target_a"] == 60


def test_the_verdict_goes_out_before_the_escalation(node, monkeypatch):
    """Ordering, not decoration.

    The planner stands down on the verdict. If the escalation went first the
    coordinator could commit an edit to the same plan the planner is about to
    open a session on, and the two would be writing over each other.
    """
    order = []
    monkeypatch.setattr(node.route_pub, "publish", lambda msg: order.append("route"))
    monkeypatch.setattr(
        node.infeasible_pub, "publish", lambda msg: order.append("escalation")
    )
    _model(
        monkeypatch,
        route_reply(
            route="escalate", reason_code="capability_missing", rationale="dead"
        ),
    )
    node._on_mission(_string(MISSION_XML))

    node._on_fault(_string(FAULT))

    assert order == ["route", "escalation"]


def test_the_same_leaf_failing_again_costs_no_second_model_call(node, monkeypatch):
    """Retry decorators re-tick a dead leaf several times before the tree ends.

    The verdict does not change between those ticks, and a model call each time
    would spend the mission's budget re-learning it. The cached verdict is
    re-published rather than dropped, so a planner that came up late still gets
    one.
    """
    published = routes(node, monkeypatch)
    calls = _model(
        monkeypatch,
        route_reply(route="repair", reason_code="unspecified", rationale="once"),
    )
    node._on_mission(_string(MISSION_XML))

    for _ in range(3):
        node._on_fault(_string(FAULT))

    assert len(calls) == 1, "routing asked the model more than once about one node"
    assert len(published) == 3, "later ticks got no verdict at all"
    assert {p["route"] for p in published} == {"repair"}


def test_a_different_leaf_is_routed_on_its_own_evidence(node, monkeypatch):
    calls = _model(
        monkeypatch,
        route_reply(route="repair", reason_code="unspecified", rationale="a"),
        route_reply(route="repair", reason_code="unspecified", rationale="b"),
    )
    node._on_mission(_string(MISSION_XML))

    node._on_fault(_string(FAULT))
    node._on_fault(
        _string(
            json.dumps({"node": "Visit_Tree_10", "status": "FAILURE", "source": "leaf"})
        )
    )

    assert len(calls) == 2


def test_a_leaf_reaching_success_is_not_a_fault(node, monkeypatch):
    """/bt/status_change reports both halves, and this used to read both as one.

    The cost was not a wasted model call. ``last_fault`` is what the escalation
    names, so recording a SUCCESS meant escalating the *approach* that worked
    instead of the sample that did not.
    """
    published = routes(node, monkeypatch)
    calls = _model(monkeypatch, route_reply(route="repair", reason_code="unspecified"))

    node._on_fault(
        _string(
            json.dumps({"node": "Visit_Tree_60", "status": "SUCCESS", "source": "leaf"})
        )
    )

    assert calls == []
    assert published == []
    assert node.last_fault is None


def test_the_trees_own_outcome_does_not_displace_the_leaf_that_failed(
    node, monkeypatch
):
    """``<tree>`` arrives after the leaf fault and is not a unit of work.

    Letting it overwrite ``last_fault`` is exactly how an escalation goes out
    with ``task_id: 0``: ``task_for_node`` finds nothing for ``<tree>``, so the
    coordinator is handed an anomaly it can interpret but never announce.
    """
    _model(monkeypatch, route_reply(route="repair", reason_code="unspecified"))
    node._on_mission(_string(MISSION_XML))
    node._on_fault(_string(FAULT))

    node._on_fault(
        _string(json.dumps({"node": "<tree>", "status": "FAILURE", "source": "tree"}))
    )

    assert node.last_fault["node"] == "Sample_Leaves_Tree_60"
    sent = escalations(node, monkeypatch)
    node._on_abort(_string(json.dumps({"reason": "non-viable"})))
    assert sent[0]["task_id"] > 0


@pytest.mark.parametrize(
    "bad_reply",
    [
        "the camera is broken, escalate this",
        '{"route": "retry", "reason_code": "unspecified"}',
        '{"route": "escalate", "reason_code": "made_up_code"}',
        "",
    ],
)
def test_routing_fails_open_to_local_repair(node, monkeypatch, bad_reply):
    """A model that is down, slow or incoherent must not strand a fault.

    Failing closed here would be worse than having no routing at all: the robot
    would sit on a fault it could have planned around because a prompt came
    back malformed. Open means "replan and find out", which is what this node
    was inserted in front of.
    """
    published = routes(node, monkeypatch)
    sent = escalations(node, monkeypatch)
    _model(monkeypatch, bad_reply)

    node._on_fault(_string(FAULT))

    assert published[0]["route"] == "repair"
    assert sent == []


def test_a_model_that_raises_fails_open_too(node, monkeypatch):
    published = routes(node, monkeypatch)

    def explode(system, user, **kw):
        raise RuntimeError("endpoint refused the connection")

    monkeypatch.setattr(tn.llm, "complete", explode)

    node._on_fault(_string(FAULT))

    assert published[0]["route"] == "repair"
    assert "endpoint refused" in published[0]["rationale"]


def test_guidance_is_dropped_on_an_escalate_verdict(node, monkeypatch):
    """There is no local planner left to advise once the work has gone."""
    published = routes(node, monkeypatch)
    _model(
        monkeypatch,
        route_reply(
            route="escalate",
            reason_code="capability_missing",
            guidance="try deploying the arm first",
            rationale="dead camera",
        ),
    )

    node._on_fault(_string(FAULT))

    assert published[0]["guidance"] == ""


def test_the_routing_prompt_carries_the_evidence_and_the_stakes(node, monkeypatch):
    calls = _model(monkeypatch, route_reply(route="repair", reason_code="unspecified"))
    node._on_mission(_string(MISSION_XML))
    node._on_world_state(_string('{"battery": 9, "row": 4}'))

    node._on_fault(_string(FAULT))

    prompt = calls[0]
    assert "Sample_Leaves_Tree_60" in prompt
    assert '"battery": 9' in prompt
    # The unit of work at stake, not just the leaf -- the difference between
    # "give up on this step" and "give this job to somebody else".
    assert "Sample_Tree_60" in prompt
    for route in tn.VALID_ROUTES:
        assert route in node.route_system_prompt


def test_the_planner_and_triage_agree_on_the_topic_name(node):
    """Spelled out on both sides rather than imported. Pin them together."""
    from amiga_ros2_agents.replanning import mission_planner_node as mp

    assert mp.ROUTE_TOPIC == tn.ROUTE_TOPIC


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
    from amiga_ros2_agents.replanning.mission_planner_node import (
        MAX_RETRIES,
        MissionPlannerNode,
    )

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
                reason=f"reached MAX_RETRIES ({MAX_RETRIES}) planning sessions",
            )
    finally:
        planner.destroy_node()

    assert len(published) == 1


def test_shedding_a_task_restores_the_planning_budget(monkeypatch):
    """A robot that sheds a task must be able to fail again, and shed again.

    The shape a live fleet run produced. amiga1's depth camera was broken for
    the whole mission, so tree 20 burned all three planning sessions, escalated,
    and was auctioned to amiga3 -- correctly. Then amiga1 drove to tree 64 and
    hit the identical fault with the budget still reported as spent, and
    _gave_up still holding max_retries_exhausted from the first one. No replan
    was possible and no second escalation could fire, so the one fault it could
    have offered the fleet twice was offered once and tree 64 was abandoned
    without a word.

    MAX_RETRIES bounds work on ONE situation. The transfer ends that situation:
    the coordinator has already committed the edit, so the plan the planner is
    now holding is not the plan the budget was spent on.
    """
    from amiga_ros2_agents.replanning.mission_planner_node import (
        MAX_RETRIES,
        MissionPlannerNode,
    )

    planner = MissionPlannerNode()
    try:
        # No model call: what is under test is the state around the session,
        # and _on_replan_request hands _run_planner to a thread.
        started = []
        monkeypatch.setattr(
            planner, "_run_planner", lambda *a, **kw: started.append((a, kw))
        )
        planner._last_status["sessions"] = MAX_RETRIES
        planner._gave_up.add(("max_retries_exhausted",))
        planner._rejection_retries = MAX_RETRIES

        planner._on_replan_request(
            _string(
                json.dumps(
                    {
                        "cause": "task_transferred",
                        "task_id": 43808,
                        "findings": [],
                    }
                )
            )
        )

        assert planner._last_status["sessions"] == 0
        # Not merely reset: still holding it would silence the give-up that
        # the NEXT exhausted budget has to publish for triage to escalate.
        assert planner._gave_up == set()
        assert planner._rejection_retries == 0
    finally:
        planner.destroy_node()


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


# ==========================================================================
# The other half of the contract: the planner's gate
# ==========================================================================


@pytest.fixture
def planner():
    from amiga_ros2_agents.replanning.mission_planner_node import MissionPlannerNode

    created = MissionPlannerNode()
    yield created
    created.destroy_node()


def sessions(planner, monkeypatch):
    """Capture the planning sessions that actually opened."""
    opened = []
    monkeypatch.setattr(
        planner,
        "_run_planner",
        lambda event, logs, **kw: opened.append((event, kw)),
    )
    return opened


def test_an_escalate_verdict_opens_no_planning_session(planner, monkeypatch):
    """The behaviour the whole reordering exists for.

    Every session opened against a permanently dead leaf makes things worse in
    two ways at once: the edit is usually another retry decorator around the
    dead thing, and the accepted plan replaces the one the escalation needs to
    resolve its failing node against.
    """
    opened = sessions(planner, monkeypatch)
    planner._on_fault_route(
        _string(json.dumps({"node": "Sample_Leaves_Tree_60", "route": "escalate"}))
    )

    planner._repair_if_routed({"node": "Sample_Leaves_Tree_60"}, [])

    assert opened == []


def test_a_repair_verdict_opens_one_and_carries_the_guidance(planner, monkeypatch):
    opened = sessions(planner, monkeypatch)
    planner._on_fault_route(
        _string(
            json.dumps(
                {
                    "node": "Sample_Leaves_Tree_60",
                    "route": "repair",
                    "guidance": "the arm was still stowed",
                }
            )
        )
    )

    planner._repair_if_routed({"node": "Sample_Leaves_Tree_60"}, [])

    assert len(opened) == 1
    assert opened[0][1]["route_guidance"] == "the arm was still stowed"


def test_no_verdict_replans_anyway_rather_than_stranding_the_fault(
    planner, monkeypatch
):
    """The gate fails open, or triage being down stops the robot recovering.

    Failing closed would make a new node a single point of failure for a loop
    that worked without it, which is a strictly worse trade than the wasted
    session the timeout occasionally costs.
    """
    from amiga_ros2_agents.replanning import mission_planner_node as mp

    monkeypatch.setattr(mp, "ROUTE_TIMEOUT_SEC", 0.05)
    opened = sessions(planner, monkeypatch)

    planner._repair_if_routed({"node": "Sample_Leaves_Tree_60"}, [])

    assert len(opened) == 1
    assert opened[0][1]["route_guidance"] == ""


@pytest.mark.parametrize(
    "event",
    [
        {"node": "<tree>", "source": "tree"},
        {"node": "", "source": "tree"},
        {"node": "<tree>"},
    ],
)
def test_the_trees_own_outcome_is_not_waited_on(planner, monkeypatch, event):
    """Triage rules on leaves, so the gate must not block on a whole-tree event.

    Seen live: the leaf was routed and escalated in 11s, then the tree's own
    FAILURE arrived and the planner sat for the full ROUTE_TIMEOUT_SEC waiting
    for a verdict on '<tree>' that triage will never publish -- because a
    verdict is only useful when it names a unit of work the fleet could be
    offered. The wait changed nothing except when the replan started.
    """
    from amiga_ros2_agents.replanning import mission_planner_node as mp

    monkeypatch.setattr(mp, "ROUTE_TIMEOUT_SEC", 30.0)
    opened = sessions(planner, monkeypatch)

    started = time.monotonic()
    planner._repair_if_routed(event, [])

    assert len(opened) == 1
    assert time.monotonic() - started < 1.0, "the gate waited on an unroutable event"


def test_a_verdict_arriving_late_wakes_the_waiting_session(planner, monkeypatch):
    """The wait is a condition, not a poll: the session starts when triage answers.

    Worth pinning because the alternative shapes both fail quietly -- a sleep
    long enough to be safe adds that delay to every repairable fault, and one
    short enough not to would fall through the gate before any model replies.
    """
    import threading

    from amiga_ros2_agents.replanning import mission_planner_node as mp

    monkeypatch.setattr(mp, "ROUTE_TIMEOUT_SEC", 10.0)
    opened = sessions(planner, monkeypatch)

    waiter = threading.Thread(
        target=planner._repair_if_routed,
        args=({"node": "Sample_Leaves_Tree_60"}, []),
        daemon=True,
    )
    waiter.start()
    time.sleep(0.1)
    assert opened == [], "the session started before any verdict arrived"

    planner._on_fault_route(
        _string(json.dumps({"node": "Sample_Leaves_Tree_60", "route": "repair"}))
    )
    waiter.join(timeout=5)

    assert not waiter.is_alive(), "the verdict did not wake the waiting session"
    assert len(opened) == 1


def test_a_verdict_for_a_different_node_does_not_release_the_gate(planner, monkeypatch):
    from amiga_ros2_agents.replanning import mission_planner_node as mp

    monkeypatch.setattr(mp, "ROUTE_TIMEOUT_SEC", 0.05)
    opened = sessions(planner, monkeypatch)
    planner._on_fault_route(
        _string(json.dumps({"node": "Visit_Tree_10", "route": "escalate"}))
    )

    # Times out and replans, rather than adopting another node's verdict and
    # standing down on work nobody ruled on.
    planner._repair_if_routed({"node": "Sample_Leaves_Tree_60"}, [])

    assert len(opened) == 1


def test_a_malformed_verdict_is_ignored_not_fatal(planner, monkeypatch):
    from amiga_ros2_agents.replanning import mission_planner_node as mp

    monkeypatch.setattr(mp, "ROUTE_TIMEOUT_SEC", 0.05)
    opened = sessions(planner, monkeypatch)
    planner._on_fault_route(_string("not json at all"))
    planner._on_fault_route(_string('"a bare string"'))
    planner._on_fault_route(_string(json.dumps({"route": "escalate"})))  # no node

    planner._repair_if_routed({"node": "Sample_Leaves_Tree_60"}, [])

    assert len(opened) == 1


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
