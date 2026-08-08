"""The real ``MissionReplanner``: tell the arbiter our workload changed.

Replaces ``AcceptEverything``. Implements the same protocol, so the contract-net
state machine does not change by one line.

**Both replan triggers land in the same place.** A fault in our own behaviour
tree goes planner -> ``/mission/candidate_xml`` -> arbiter. Winning someone
else's auction comes here, and this reaches that same arbiter, which applies the
edit and runs it through the same ``_evaluate``. They are one event seen from
two sides -- work this robot now has that it did not have before -- and gating
them differently would mean the guarantee only covers whichever path was tested.

**No XML crosses this file.** The task goes over as the flat fields the radio
already carries, and the arbiter edits the plan it holds. That keeps the
coordinator's rule intact -- it never parses a mission -- and removes any window
where we would be editing a copy the arbiter has since replaced.

Why this is asynchronous
------------------------

``replan_and_verify`` is called from inside the coordinator's lock:
``_grant_delivered`` takes it, and ``_take_on`` runs under it. That lock is the
one ``tick`` and ``on_message`` need, so blocking there stops heartbeats,
auctions and bids for the duration. And the duration is not small -- ``spin
-search`` regenerates and compiles a C verifier on every run, about 1.4 s
measured on this hardware, before any model call is counted.

So the request is dispatched on a thread and this returns immediately. What that
does *not* do is let an unverified plan reach the robot: the arbiter is the sole
writer of ``/mission/xml``, and it publishes only after the gate passes. The
mission the behaviour tree executes is still never committed without
verification -- the coordinator simply stops waiting around to be told so.

A rejection therefore arrives late, and is routed to the anomaly path -- which
is exactly what ``_take_on`` does with a synchronous rejection today (release
the task, re-interpret it as an anomaly). Only the timing differs, and that path
is inherently after-the-fact: by then the transfer has been ACKed and the task
is ours whether it verifies or not.
"""

from threading import Event, Lock, Thread
from typing import Callable, Optional

from amiga_interfaces.srv import VerifyReplan

from ..vocabulary.model import MissionDelta
from ..ports.reasoning import ReplanResult

#: Ceiling on one verification round trip. Off the critical path, so it can be
#: generous: a formula that needs a cold model call is the slow case, and giving
#: up early on it means handing back a task that was about to be cleared.
DEFAULT_TIMEOUT_SEC = 90.0

#: How long to wait for the arbiter to exist at all. Short: if the agent stack
#: is not running, it will not start mid-mission.
SERVICE_WAIT_SEC = 5.0


class VerifyingReplanner:
    """Reports a committed ownership change and acts on the arbiter's verdict.

    ``on_rejected(task, reason)`` is called, off any coordinator lock, when the
    arbiter refuses the edit. Wire it to the anomaly path: work this robot holds
    and cannot legitimately do is precisely what that path is for.
    """

    def __init__(
        self,
        node,
        on_rejected: Optional[Callable[[object, str], None]] = None,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
        service_name: str = "/mission/verify_replan",
        require_verifier: bool = False,
    ):
        self._node = node
        self._on_rejected = on_rejected
        self._timeout = float(timeout_sec)
        self._service_name = service_name
        #: What an unreachable arbiter means. False -- the default -- accepts
        #: and records the change as unverified, so a coordinator can run
        #: without the agent stack, which is how a radio-only bringup works.
        #: True refuses: a run whose results depend on verification must not
        #: quietly produce unchecked ones.
        self._require_verifier = bool(require_verifier)
        self._client = node.create_client(VerifyReplan, service_name)
        self._lock = Lock()
        self._inflight = 0
        #: Verdicts, newest last. The acceptance tests read this; nothing in the
        #: state machine does.
        self.results: "list[ReplanResult]" = []

    # ------------------------------------------------------------------
    # MissionReplanner
    # ------------------------------------------------------------------

    def replan_and_verify(self, delta: MissionDelta) -> ReplanResult:
        """Dispatch verification of ``delta``. Returns before it finishes.

        The returned result is not the verdict and does not claim to be -- see
        the module docstring for why waiting for one here would stall the
        coordinator, and why nothing unverified reaches the robot regardless.
        """
        pending = [(task, True) for task in delta.removed]
        pending += [(task, False) for task in delta.added]
        if not pending:
            return ReplanResult(True, None, "empty delta: nothing to verify")

        for task, removing in pending:
            with self._lock:
                self._inflight += 1
            Thread(
                target=self._run,
                args=(task, removing, delta.note),
                daemon=True,
                name=f"verify-task-{int(task.task_id)}",
            ).start()

        return ReplanResult(True, None, "verification dispatched")

    # ------------------------------------------------------------------
    # Off-lock
    # ------------------------------------------------------------------

    def wait_idle(self, timeout: float = 30.0) -> bool:
        """Block until no verification is in flight. For tests and shutdown."""
        deadline = Event()
        step = 0.02
        waited = 0.0
        while waited < timeout:
            with self._lock:
                if self._inflight == 0:
                    return True
            deadline.wait(step)
            waited += step
        with self._lock:
            return self._inflight == 0

    def _run(self, task, removing: bool, note: str = "") -> None:
        try:
            result = self._ask(task, removing, note)
        except Exception as exc:  # noqa: BLE001 - a thread that dies silently
            result = ReplanResult(False, None, f"verification raised: {exc}")
        finally:
            with self._lock:
                self._inflight -= 1

        with self._lock:
            self.results.append(result)

        if result.rejected:
            self._node.get_logger().warn(
                f"task {int(task.task_id)}: replan rejected — {result.reason}"
            )
            if self._on_rejected is not None and not removing:
                # Only for work we took on. A rejected *removal* leaves the plan
                # as it was, so there is nothing to hand back -- the task is
                # already somebody else's.
                try:
                    self._on_rejected(task, result.reason)
                except Exception as exc:  # noqa: BLE001
                    self._node.get_logger().error(
                        f"on_rejected raised for task {int(task.task_id)}: {exc}"
                    )
        elif "unverified" in result.reason:
            self._node.get_logger().warn(
                f"task {int(task.task_id)}: accepted without verification — "
                f"{result.reason}"
            )

    def _ask(self, task, removing: bool, note: str = "") -> ReplanResult:
        if not self._client.wait_for_service(timeout_sec=SERVICE_WAIT_SEC):
            message = f"{self._service_name} unavailable"
            if self._require_verifier:
                return ReplanResult(False, None, f"rejected: {message}")
            return ReplanResult(True, None, f"unverified: {message}")

        request = VerifyReplan.Request()
        request.removing = bool(removing)
        request.task_id = int(task.task_id)
        request.required_capabilities = int(task.required_capabilities)
        request.target_kind = int(task.location.kind)
        request.target_a = int(task.location.a)
        request.target_b = int(task.location.b)
        request.priority = int(task.priority)
        request.task_xml = _subtree_of(task)
        # Captured when we bid, not looked up now: by GRANT time the note
        # has usually already expired out of the reliability layer.
        request.note = str(note or "")

        future = self._client.call_async(request)
        # Deliberately not spin_until_future_complete: this may run on a thread
        # of the executor that would have to do the spinning, and asking it to
        # spin itself deadlocks. Waiting on the future is enough given main()'s
        # multithreaded executor is free to service the response.
        if not _wait(future, self._timeout):
            return ReplanResult(
                False, None, f"verification timed out after {self._timeout:.0f}s"
            )

        response = future.result()
        if response is None:
            return ReplanResult(False, None, "verification call failed")

        detail = response.reason or ("verified" if response.verified else "no reason")
        if not response.accepted:
            return ReplanResult(False, None, f"rejected: {detail}")
        return ReplanResult(
            True, None, detail if response.verified else f"unverified: {detail}"
        )


def _subtree_of(task) -> str:
    """The task's own XML, when this robot has it.

    Populated for a task our own planner shed -- ``payload`` is the mission
    node's handle back to its own structure. Empty for one won from a peer,
    which is the normal case: TASK_ANNOUNCE carries a capability mask and a
    target, not a subtree, so the arbiter rebuilds one from those.
    """
    payload = getattr(task, "payload", None)
    return getattr(payload, "xml", "") or ""


def _wait(future, timeout: float) -> bool:
    """Block until ``future`` completes, without spinning an executor."""
    if future.done():
        return True
    done = Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout) or future.done()
