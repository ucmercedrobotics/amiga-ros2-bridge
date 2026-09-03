"""The coordinator's half of the verification seam.

Two things are being pinned, and neither is "does the arbiter's gate work" --
that is ``amiga_ros2_agents/test/test_arbiter_replan.py``'s job.

First, that verification never blocks the coordinator. ``replan_and_verify`` is
called from inside the lock ``tick`` and ``on_message`` need, and it is a ROS
service round trip to another process. A synchronous client would stall
heartbeats, auctions and bids for the duration, so the contract is that this
returns immediately and the verdict arrives later.

Second, that a late rejection still ends up where a synchronous one would: back
on the anomaly path. The behaviour ``_take_on`` implements inline has to survive
being moved off the critical path, or transfers get verified and then ignored.

The service is faked. Standing up an arbiter here would test the arbiter.
"""

import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import Capability, Target, cap_mask  # noqa: E402
from amiga_ros2_coordinator.vocabulary.model import MissionDelta, Task  # noqa: E402
from amiga_ros2_coordinator.adapters.replanner_client import (  # noqa: E402
    VerifyingReplanner,
    _subtree_of,
)

TASK = Task(
    task_id=4210,
    required_capabilities=cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF),
    location=Target.tree(60),
    priority=100,
)


class FakeFuture:
    def __init__(self, response, delay=0.0):
        self._response = response
        self._callbacks = []
        self._done = False
        if delay:
            threading.Timer(delay, self._finish).start()
        else:
            self._finish()

    def _finish(self):
        self._done = True
        for callback in self._callbacks:
            callback(self)

    def done(self):
        return self._done

    def result(self):
        return self._response

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._done:
            callback(self)


class FakeResponse:
    def __init__(self, accepted=True, reason=""):
        self.accepted = accepted
        self.reason = reason


class FakeClient:
    def __init__(self, response=None, available=True, delay=0.0):
        self.response = response if response is not None else FakeResponse()
        self.available = available
        self.delay = delay
        self.requests = []

    def wait_for_service(self, timeout_sec=0.0):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture(self.response, self.delay)


class FakeLogger:
    def __init__(self):
        self.messages = []

    def warn(self, message):
        self.messages.append(("warn", message))

    def error(self, message):
        self.messages.append(("error", message))

    def info(self, message):
        self.messages.append(("info", message))


class FakeNode:
    def __init__(self, client):
        self._client = client
        self._logger = FakeLogger()

    def create_client(self, srv_type, name):
        return self._client

    def get_logger(self):
        return self._logger


def make(client=None, **kwargs):
    client = client if client is not None else FakeClient()
    node = FakeNode(client)
    replanner = VerifyingReplanner(node, **kwargs)
    return replanner, client, node


def settled(replanner):
    assert replanner.wait_idle(timeout=5.0), "verification never finished"
    return replanner.results


# ---------------------------------------------------------------------------
# It must not block
# ---------------------------------------------------------------------------


def test_returns_before_the_verdict_arrives():
    """The lock this runs under is the one the auction timers need."""
    replanner, _, _ = make(FakeClient(delay=0.5))
    started = time.monotonic()
    result = replanner.replan_and_verify(MissionDelta(added=[TASK]))
    elapsed = time.monotonic() - started

    assert elapsed < 0.2, f"replan_and_verify blocked for {elapsed:.2f}s"
    assert result.accepted
    settled(replanner)


def test_empty_delta_asks_nothing():
    replanner, client, _ = make()
    assert replanner.replan_and_verify(MissionDelta()).accepted
    assert client.requests == []


# ---------------------------------------------------------------------------
# The request
# ---------------------------------------------------------------------------


def test_added_task_is_sent_as_arriving():
    replanner, client, _ = make()
    replanner.replan_and_verify(MissionDelta(added=[TASK], cause="won auction"))
    settled(replanner)

    (request,) = client.requests
    assert request.removing is False
    assert request.task_id == TASK.task_id
    assert request.required_capabilities == TASK.required_capabilities
    assert request.target_a == 60
    assert request.task_xml == ""  # won from a peer: no subtree to send


def test_removed_task_is_sent_as_leaving():
    replanner, client, _ = make()
    replanner.replan_and_verify(MissionDelta(removed=[TASK], cause="transferred"))
    settled(replanner)

    (request,) = client.requests
    assert request.removing is True
    assert request.task_id == TASK.task_id


def test_local_subtree_is_forwarded_when_we_have_one():
    """A task our own planner shed carries its XML; a won one does not."""

    class Payload:
        xml = "<Sequence name='x'/>"

    replanner, client, _ = make()
    replanner.replan_and_verify(
        MissionDelta(added=[Task(**{**_fields(TASK), "payload": Payload()})])
    )
    settled(replanner)
    assert client.requests[0].task_xml == "<Sequence name='x'/>"


def test_subtree_of_tolerates_a_payload_without_xml():
    assert _subtree_of(TASK) == ""
    assert _subtree_of(Task(**{**_fields(TASK), "payload": object()})) == ""


# ---------------------------------------------------------------------------
# The verdict
# ---------------------------------------------------------------------------


def test_rejection_goes_to_the_anomaly_path():
    """What ``_take_on`` does inline, only later."""
    handled = []
    replanner, _, _ = make(
        FakeClient(FakeResponse(accepted=False, reason="objective violation")),
        on_rejected=lambda task, reason: handled.append((task, reason)),
    )
    replanner.replan_and_verify(MissionDelta(added=[TASK]))
    settled(replanner)

    assert len(handled) == 1
    task, reason = handled[0]
    assert task.task_id == TASK.task_id
    assert "objective violation" in reason


def test_rejected_removal_is_not_handed_back():
    """There is nothing to hand back: the task is already someone else's."""
    handled = []
    replanner, _, _ = make(
        FakeClient(FakeResponse(accepted=False, reason="nope")),
        on_rejected=lambda task, reason: handled.append(task),
    )
    replanner.replan_and_verify(MissionDelta(removed=[TASK]))
    settled(replanner)
    assert handled == []


def test_accepted_is_recorded_as_such():
    replanner, _, _ = make(FakeClient(FakeResponse(True, "accepted")))
    replanner.replan_and_verify(MissionDelta(added=[TASK]))
    (result,) = settled(replanner)
    assert result.accepted
    assert "unverified" not in result.reason


# ---------------------------------------------------------------------------
# Failing closed
# ---------------------------------------------------------------------------


def test_missing_arbiter_accepts_but_flags_by_default():
    """A coordinator has to be able to run without the agent stack."""
    replanner, _, _ = make(FakeClient(available=False))
    replanner.replan_and_verify(MissionDelta(added=[TASK]))
    (result,) = settled(replanner)
    assert result.accepted
    assert "unverified" in result.reason


def test_missing_arbiter_rejects_when_the_verifier_is_required():
    """For a run whose results depend on every change having been checked."""
    handled = []
    replanner, _, _ = make(
        FakeClient(available=False),
        require_verifier=True,
        on_rejected=lambda task, reason: handled.append(task),
    )
    replanner.replan_and_verify(MissionDelta(added=[TASK]))
    (result,) = settled(replanner)
    assert not result.accepted
    assert len(handled) == 1


def test_timeout_is_a_rejection():
    """A verifier that did not answer has not verified anything."""
    replanner, _, _ = make(FakeClient(delay=10.0), timeout_sec=0.2)
    replanner.replan_and_verify(MissionDelta(added=[TASK]))
    (result,) = settled(replanner)
    assert not result.accepted
    assert "timed out" in result.reason


def test_a_raising_callback_does_not_kill_the_thread():
    replanner, _, node = make(
        FakeClient(FakeResponse(accepted=False, reason="no")),
        on_rejected=lambda task, reason: 1 / 0,
    )
    replanner.replan_and_verify(MissionDelta(added=[TASK]))
    settled(replanner)
    assert any(level == "error" for level, _ in node._logger.messages)


def _fields(task) -> dict:
    return {
        "task_id": task.task_id,
        "required_capabilities": task.required_capabilities,
        "location": task.location,
        "priority": task.priority,
    }
