"""Tests for the coordinator's half of the reasoning seam.

The triage agent answers over a ROS service, and ROS IDL has no unions, so the
answer arrives as strings and integers. This is the code that turns those back
into the closed action schema the state machine was tested against -- which
makes it the place where a stringly-typed wire could quietly widen an interface
that everything below it depends on being narrow.

So: every valid answer maps to the right action, and every invalid one raises
rather than being coerced into something plausible. A coordinator that acted on
a half-understood interpretation would shed a task on no evidence, and the fleet
has no way to tell that apart from a decision.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_coordinator.model import Location, Task  # noqa: E402
from amiga_ros2_coordinator.schema import (  # noqa: E402
    AddTask,
    AnomalyContext,
    DropTask,
    LocalDisposition,
    ReDelegate,
)
from amiga_ros2_coordinator.triage_client import (  # noqa: E402
    TriageClient,
    TriageRefused,
    _peers_json,
)

TASK = Task(
    task_id=60,
    required_capability=0,
    location=Location(row=4, col=9),
    priority=100,
)


class Response:
    """Stands in for InterpretAnomaly.Response, which needs a colcon build.

    A plain object with the same fields: the decoder reads attributes and never
    the ROS type, so building the interface package to test string handling
    would only slow the suite down.
    """

    def __init__(self, **fields):
        self.ok = True
        self.error = ""
        self.action = ""
        self.reason_code = 0
        self.fallback = ""
        self.disposition = ""
        self.task_id = 0
        self.required_capability = 0
        self.grid_row = 0
        self.grid_col = 0
        self.priority = 0
        self.rationale = ""
        self.model = "test-model"
        for name, value in fields.items():
            setattr(self, name, value)


def decode(response, task=TASK):
    """Run the decoder without constructing a node or a service client."""
    context = AnomalyContext(task=task, detail="", at=100.0)
    return TriageClient._decode(_bare_client(), response, context)


def _bare_client() -> TriageClient:
    return TriageClient.__new__(TriageClient)


# ==========================================================================
# Valid answers
# ==========================================================================


def test_re_delegate_becomes_a_re_delegate_over_the_anomalys_own_task():
    action = decode(
        Response(action="re_delegate", reason_code=1, fallback="request_human")
    )
    assert isinstance(action, ReDelegate)
    # The task comes from the context, never from the response: the agent is
    # answering about work the coordinator already named, and letting it
    # substitute a different task would be a delegation nobody asked for.
    assert action.task is TASK
    assert action.reason_code == 1
    assert action.fallback is LocalDisposition.REQUEST_HUMAN


def test_a_re_delegate_with_no_fallback_holds():
    # Holding keeps the work and blocks only this robot. Dropping by default
    # would silently lose a task because a field was blank.
    action = decode(Response(action="re_delegate"))
    assert action.fallback is LocalDisposition.HOLD


def test_drop_task_becomes_a_drop_task():
    action = decode(Response(action="drop_task", disposition="hold", reason_code=3))
    assert isinstance(action, DropTask)
    assert action.disposition is LocalDisposition.HOLD
    assert action.reason_code == 3


def test_add_task_builds_the_new_task_from_the_response():
    action = decode(
        Response(
            action="add_task",
            task_id=77,
            required_capability=2,
            grid_row=11,
            grid_col=3,
            priority=200,
        )
    )
    assert isinstance(action, AddTask)
    assert action.task.task_id == 77
    assert action.task.required_capability == 2
    assert (action.task.location.row, action.task.location.col) == (11, 3)
    assert action.task.priority == 200


def test_the_action_name_is_matched_case_insensitively():
    assert isinstance(decode(Response(action="  RE_DELEGATE ")), ReDelegate)


# ==========================================================================
# Answers that must be refused
# ==========================================================================


def test_an_action_outside_the_schema_is_refused():
    with pytest.raises(TriageRefused, match="not in the schema"):
        decode(Response(action="add_constraint"))


def test_an_empty_action_is_refused():
    with pytest.raises(TriageRefused):
        decode(Response(action=""))


def test_an_add_task_with_no_task_id_is_refused():
    # 0 is what the agent sends when the model named no task. Announcing it
    # would put a task id on the radio that matches no work anywhere.
    with pytest.raises(TriageRefused, match="task_id=0"):
        decode(Response(action="add_task", task_id=0))


def test_an_add_task_outside_the_wire_range_is_refused():
    # Caught here rather than at encode time: a codec exception three steps
    # later has lost the context that would explain it.
    with pytest.raises(TriageRefused, match="outside the wire range"):
        decode(Response(action="add_task", task_id=70000))


def test_re_delegate_with_no_task_attached_is_refused():
    # Some anomalies have no task. Shedding "nothing" to the fleet is not a
    # coherent request, so it is a refusal rather than a silent no-op.
    with pytest.raises(TriageRefused, match="no task attached"):
        decode(Response(action="re_delegate"), task=None)


def test_drop_task_with_no_task_attached_is_refused():
    with pytest.raises(TriageRefused, match="no task attached"):
        decode(Response(action="drop_task"), task=None)


def test_a_nonsense_disposition_falls_back_rather_than_crashing():
    # Different from the agent's own parser, which refuses these outright. By
    # the time a value reaches here it has already been validated once; a
    # second opinion that raised would turn a survivable oddity into a lost
    # interpretation, so this one defaults and keeps the decision.
    action = decode(Response(action="drop_task", disposition="obliterate"))
    assert action.disposition is LocalDisposition.DROP


# ==========================================================================
# Prompt context assembled for the agent
# ==========================================================================


def test_the_peer_list_carries_what_a_delegation_decision_needs():
    from amiga_ros2_coordinator.model import PeerRecord

    context = AnomalyContext(
        task=TASK,
        at=100.0,
        peers=(
            PeerRecord(
                robot_id=2,
                cap_mask=0b101,  # DRIVE (index 0) and SPRAY (index 2)
                location=Location(row=1, col=2),
                battery=80,
                current_task=0,
                last_seen=94.5,
            ),
        ),
    )
    peers = __import__("json").loads(_peers_json(context))

    assert len(peers) == 1
    assert peers[0]["id"] == 2
    assert peers[0]["capabilities"] == ["DRIVE", "SPRAY"]
    assert peers[0]["battery_percent"] == 80
    assert peers[0]["idle"] is True
    # Relative, not absolute: "seen 5.5s ago" is a fact a model can reason
    # about, and a monotonic timestamp from this robot's clock is not.
    assert peers[0]["last_seen_sec_ago"] == 5.5


def test_an_empty_fleet_is_an_empty_list_not_an_omission():
    # The prompt says an empty list means nobody is out there, which is exactly
    # when re_delegate is the wrong answer. It has to actually be sent.
    assert _peers_json(AnomalyContext(task=TASK, at=0.0)) == "[]"
