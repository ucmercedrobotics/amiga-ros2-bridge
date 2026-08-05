#!/usr/bin/env python3
"""The ROS side of ``interpret_anomaly``: a client for the triage agent.

Everything in this package except this file is written against ports. This is
one of the two places where a port meets a real ROS service, and it is kept out
of coordinator.py so the state machine stays testable without a middleware.

    coordinator  --/coordination/interpret_anomaly-->  triage agent (LLM)
                 <--  re_delegate | add_task | drop_task  --

Two things it is responsible for, beyond the call itself.

**Not blocking the state machine.** The service is a language model and takes
seconds. ``CoordinatorSession.report_infeasible`` holds the lock that ``tick``
and ``on_message`` need, so the call cannot happen inside it -- a robot whose
heartbeats, auctions and bids stop for the length of a model call is a robot the
fleet writes off as dead. The node calls this off the lock and hands the
finished action back through ``report_infeasible(action=...)``.

**Refusing a bad answer.** The service returns strings, because ROS IDL has no
unions. This translates them back into the closed schema and raises on anything
that is not in it. The wire being stringly-typed does not get to widen the
interface the state machine was tested against.

No decisions are made here. If the service is down, times out, or answers with
something outside the schema, this raises and the coordinator leaves the task
exactly where it was -- which is the safe direction to fail in.
"""

import json
import time
from typing import Optional

from amiga_ros2_comms.codec import (
    PRIORITY_MAX,
    TASK_ID_MAX,
    ReasonCode,
    Target,
    TargetKind,
)

from .model import Task, capability_names
from .schema import (
    ActionSchema,
    AddTask,
    AnomalyContext,
    DropTask,
    LocalDisposition,
    ReDelegate,
)

#: Seconds to wait for an interpretation before giving up on it. Generous,
#: because the thing on the other end is a model on a possibly-shared endpoint,
#: and a timeout here means the anomaly goes unanswered and the task sits. The
#: cost of waiting is bounded -- this runs off the coordinator's lock.
DEFAULT_TIMEOUT_SEC = 45.0

#: The service name the triage agent serves. Absolute, because the agent is one
#: per robot and not inside this node's namespace.
DEFAULT_SERVICE = "/coordination/interpret_anomaly"

_DISPOSITIONS = {
    "drop": LocalDisposition.DROP,
    "hold": LocalDisposition.HOLD,
    "request_human": LocalDisposition.REQUEST_HUMAN,
}


class TriageUnavailable(RuntimeError):
    """The triage agent could not be reached, or did not answer in time."""


class TriageRefused(ValueError):
    """The triage agent answered with something outside the action schema."""


class TriageClient:
    """Calls the triage agent and returns one typed action, or raises."""

    def __init__(
        self,
        node,
        service_name: str = DEFAULT_SERVICE,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
        callback_group=None,
    ):
        # Imported here rather than at module scope so that importing this
        # package does not require amiga_interfaces to have been built. The
        # engine and its tests do not need it; only a running node does.
        from amiga_interfaces.srv import InterpretAnomaly

        self._srv_type = InterpretAnomaly
        self._node = node
        self._timeout_sec = float(timeout_sec)
        self._client = node.create_client(
            InterpretAnomaly, service_name, callback_group=callback_group
        )
        self.service_name = service_name

    def available(self, wait_sec: float = 0.0) -> bool:
        return self._client.wait_for_service(timeout_sec=wait_sec)

    def interpret_anomaly(self, context: AnomalyContext) -> ActionSchema:
        """One anomaly in, one typed action out. Blocks; never call under a lock."""
        if not self._client.wait_for_service(timeout_sec=1.0):
            raise TriageUnavailable(f"no triage agent on {self.service_name}")

        future = self._client.call_async(self._request(context))
        # Spinning is the caller's executor's job. This waits on the future the
        # executor completes, which is why the node hands this a reentrant
        # callback group and runs on a MultiThreadedExecutor -- a single-
        # threaded executor blocked here would never deliver the response.
        if not _wait(future, self._timeout_sec):
            raise TriageUnavailable(
                f"triage agent did not answer within {self._timeout_sec}s"
            )

        response = future.result()
        if response is None:
            raise TriageUnavailable("triage call completed with no response")
        if not response.ok:
            raise TriageRefused(response.error or "triage declined to interpret")
        return self._decode(response, context)

    # ------------------------------------------------------------------

    def _request(self, context: AnomalyContext):
        request = self._srv_type.Request()
        task = context.task
        request.task_id = int(task.task_id) if task else 0
        request.required_capabilities = int(task.required_capabilities) if task else 0
        target = task.location if task else Target.none()
        request.target_kind = int(target.kind)
        request.target_a = int(target.a)
        request.target_b = int(target.b)
        request.priority = int(task.priority) if task else 0
        request.battery_percent = max(0, min(int(context.battery), 100))
        request.peers_json = _peers_json(context)
        # The evidence fields are left empty on purpose. The triage agent
        # watches /bt/status_change, /rosout and /world_state itself; this layer
        # knows about tasks and peers and has no business carrying behaviour-tree
        # internals it cannot interpret. They exist on the service so the agent
        # can also be driven by hand.
        request.fault_json = ""
        request.log_context = ""
        request.world_state = ""
        request.local_attempts = context.detail or ""
        return request

    def _decode(self, response, context: AnomalyContext) -> ActionSchema:
        action = (response.action or "").strip().lower()
        reason_code = _bounded(response.reason_code, 0xFFFF, ReasonCode.UNSPECIFIED)

        if action == "re_delegate":
            if context.task is None:
                raise TriageRefused("re_delegate with no task attached to the anomaly")
            return ReDelegate(
                task=context.task,
                reason_code=reason_code,
                fallback=_disposition(response.fallback, LocalDisposition.HOLD),
            )

        if action == "drop_task":
            if context.task is None:
                raise TriageRefused("drop_task with no task attached to the anomaly")
            return DropTask(
                task=context.task,
                disposition=_disposition(response.disposition, LocalDisposition.DROP),
                reason_code=reason_code,
            )

        if action == "add_task":
            return AddTask(task=self._task_from(response), reason_code=reason_code)

        raise TriageRefused(f"action {action!r} is not in the schema")

    def _task_from(self, response) -> Task:
        """Build the announced task from an add_task answer.

        Bounds-checked here rather than at encode time: a task that cannot fit
        in a TASK_ANNOUNCE is a task this layer cannot delegate, and finding
        that out from a codec exception three steps later loses the context
        that would explain it.
        """
        task_id = int(response.task_id)
        if not 0 < task_id <= TASK_ID_MAX:
            raise TriageRefused(
                f"add_task with task_id={task_id}, outside the wire range"
            )
        try:
            location = Target(
                kind=TargetKind(int(response.target_kind)),
                a=int(response.target_a),
                b=int(response.target_b),
            )
        except ValueError as exc:
            # A target we cannot represent is not a target we can go to. This
            # is the second of the two validation points -- the agent refuses a
            # malformed answer before transmitting, and this refuses one that
            # got here anyway -- and the pair is what keeps the closed schema
            # closed across a stringly-typed IDL.
            raise TriageRefused(
                f"add_task names a place that is not one: {exc}"
            ) from None
        return Task(
            task_id=task_id,
            required_capabilities=int(response.required_capabilities),
            location=location,
            priority=_bounded(response.priority, PRIORITY_MAX, 0),
        )


def _wait(future, timeout_sec: float) -> bool:
    """Wait for ``future``, returning whether it completed in time."""
    deadline = time.monotonic() + timeout_sec
    while not future.done():
        if time.monotonic() >= deadline:
            future.cancel()
            return False
        time.sleep(0.02)
    return True


def _peers_json(context: AnomalyContext) -> str:
    """The live fleet, as the agent sees it.

    Shedding work is only a sensible answer if somebody is out there to take
    it, so this is load-bearing prompt context rather than decoration.
    """
    peers = []
    for peer in context.peers:
        peers.append(
            {
                "id": int(peer.robot_id),
                # XML element names, so the model reads the same words the
                # mission it is reasoning about is written in.
                "capabilities": capability_names(peer.cap_mask),
                "battery_percent": int(peer.battery),
                "current_task": int(peer.current_task),
                "idle": bool(peer.idle),
                "where": str(peer.location) if peer.location else None,
                "last_seen_sec_ago": round(context.at - float(peer.last_seen), 1),
            }
        )
    return json.dumps(peers)


def _disposition(name: str, default: LocalDisposition) -> LocalDisposition:
    return _DISPOSITIONS.get((name or "").strip().lower(), default)


def _bounded(value, maximum: int, default: int) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError):
        return default
    return number if 0 <= number <= maximum else default


def optional_client(node, *args, **kwargs) -> Optional["TriageClient"]:
    """A TriageClient, or None if amiga_interfaces is not available.

    Lets a bench run come up without the generated service type, which is the
    difference between "coordination is degraded" and "the node will not start".
    """
    try:
        return TriageClient(node, *args, **kwargs)
    except ImportError:
        node.get_logger().warn(
            "amiga_interfaces is not built: no triage agent will be consulted, "
            "and anomalies will fall back to the local interpreter"
        )
        return None
