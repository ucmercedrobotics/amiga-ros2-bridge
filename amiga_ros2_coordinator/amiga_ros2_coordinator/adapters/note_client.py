#!/usr/bin/env python3
"""The ROS side of ``interpret_note``: a client for the note agent.

The second of the two places a port meets a real ROS service, and the more
dangerous one.

    coordinator  --/coordination/interpret_note-->  note agent (LLM)
                 <--  keep | revise | withdraw  --

``triage_client`` asks a model about *our own* fault, from evidence we gathered.
This asks a model about *someone else's sentence*, which arrived over a radio
with a self-asserted source and no MAC. Anyone within range can put text in
front of this model. What makes that survivable is not the model and not the
prompt -- it is that the answer can only be one of three things, and all three
of them only ever change a bid we were already going to make. A note cannot
start an auction, take on work, drop work, or move the robot. ``schema.py``'s
``validate_revision`` is what holds that line, and this file's job is to
translate the wire's strings back into it and raise on everything else.

**The deadline is the design constraint here, not a safety margin.** Reading a
note makes the auction deliberative: the bid backoff stretches by
``note_backoff_multiplier`` (10x, so 20 s at the defaults) precisely to leave
room for this call. An answer that arrives after the bid has gone out is not
late, it is useless -- the coordinator counts it ``notes_too_late`` and bids on
mechanics. So DEFAULT_TIMEOUT_SEC has to sit *under* the stretched backoff, and
is deliberately a third of the triage client's 45 s.

Failing means bidding as fitness decided, unrevised. That is the direction the
auction worked in before notes existed, so a dead note agent costs the fleet
nothing it had.
"""

import time
from typing import Optional

from amiga_ros2_comms.codec import COST_MAX, Target

from ..vocabulary.schema import (
    BidRevision,
    KeepBid,
    NoteContext,
    ReviseBid,
    WithdrawBid,
)

#: Seconds to wait for a revision. Must stay below the note-stretched bid
#: backoff -- see the module docstring. 15.0 against a 20 s backoff leaves the
#: bid five seconds to be assembled and sent after the answer lands.
DEFAULT_TIMEOUT_SEC = 15.0

#: The service name the note agent serves. Absolute, for the same reason the
#: triage service is: one agent stack per robot, outside this node's namespace.
DEFAULT_SERVICE = "/coordination/interpret_note"


class NoteUnavailable(RuntimeError):
    """The note agent could not be reached, or did not answer in time."""


class NoteRefused(ValueError):
    """The note agent answered with something outside the revision schema."""


class NoteClient:
    """Calls the note agent and returns one typed revision, or raises."""

    def __init__(
        self,
        node,
        service_name: str = DEFAULT_SERVICE,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
        callback_group=None,
    ):
        # Imported here rather than at module scope for the same reason
        # triage_client does it: the engine and its tests must import without
        # amiga_interfaces having been built. Only a running node needs it.
        from amiga_interfaces.srv import InterpretNote

        self._srv_type = InterpretNote
        self._node = node
        self._timeout_sec = float(timeout_sec)
        self._client = node.create_client(
            InterpretNote, service_name, callback_group=callback_group
        )
        self.service_name = service_name

    def available(self, wait_sec: float = 0.0) -> bool:
        return self._client.wait_for_service(timeout_sec=wait_sec)

    def interpret_note(self, context: NoteContext) -> BidRevision:
        """One note in, one typed revision out. Blocks; never call under a lock."""
        if not self._client.wait_for_service(timeout_sec=1.0):
            raise NoteUnavailable(f"no note agent on {self.service_name}")

        future = self._client.call_async(self._request(context))
        if not _wait(future, self._timeout_sec):
            raise NoteUnavailable(
                f"note agent did not answer within {self._timeout_sec}s"
            )

        response = future.result()
        if response is None:
            raise NoteUnavailable("note call completed with no response")
        if not response.ok:
            raise NoteRefused(response.error or "note agent declined to interpret")
        return self._decode(response)

    # ------------------------------------------------------------------

    def _request(self, context: NoteContext):
        request = self._srv_type.Request()
        request.note_text = context.text
        request.announcer_id = max(0, min(int(context.src), 0xFF))
        request.task_id = int(context.task_id)

        # The offer, if it got here first. When it did not, these stay zero and
        # `our_bid_unknown` says so -- a model told "required_capabilities: 0"
        # would otherwise read that as a task needing no actions.
        task = context.task
        request.required_capabilities = int(task.required_capabilities) if task else 0
        target = task.location if task else Target.none()
        request.target_kind = int(target.kind)
        request.target_a = int(target.a)
        request.target_b = int(target.b)
        request.priority = int(task.priority) if task else 0

        unknown = context.cost is None or context.feasible is None
        request.our_bid_unknown = bool(unknown)
        request.our_cost = (
            0 if context.cost is None else _bounded(context.cost, COST_MAX)
        )
        request.our_eta_s = (
            0 if context.eta_s is None else _bounded(context.eta_s, 0xFFFF)
        )
        request.our_feasible = bool(context.feasible) if not unknown else False
        return request

    def _decode(self, response) -> BidRevision:
        revision = (response.revision or "").strip().lower()
        reason = (response.reason or "").strip() or (response.rationale or "").strip()

        if revision == "keep":
            return KeepBid(reason=reason)

        if revision == "revise":
            # Clamped, not refused. A delta is a claim about the work rather
            # than a number with a meaning of its own, and a model that says
            # "much worse" by writing 4000 has still said something the
            # coordinator can act on. An unparseable one has not.
            try:
                delta = int(response.cost_delta)
            except (TypeError, ValueError):
                raise NoteRefused(
                    f"revise with cost_delta={response.cost_delta!r}, which is "
                    f"not a number"
                ) from None
            return ReviseBid(
                cost_delta=max(-COST_MAX, min(delta, COST_MAX)), reason=reason
            )

        if revision == "withdraw":
            return WithdrawBid(reason=reason)

        raise NoteRefused(f"revision {revision!r} is not in the schema")


def _wait(future, timeout_sec: float) -> bool:
    """Wait for ``future``, returning whether it completed in time."""
    deadline = time.monotonic() + timeout_sec
    while not future.done():
        if time.monotonic() >= deadline:
            future.cancel()
            return False
        time.sleep(0.02)
    return True


def _bounded(value, maximum: int) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError):
        return 0
    return max(0, min(number, maximum))


def optional_client(node, *args, **kwargs) -> Optional["NoteClient"]:
    """A NoteClient, or None if amiga_interfaces is not available.

    Same tolerance as the triage client's: a workspace without the generated
    service still starts, and notes are recorded and ignored -- which is the
    behaviour the coordinator has without an interpreter anyway.
    """
    try:
        return NoteClient(node, *args, **kwargs)
    except ImportError:
        node.get_logger().warn(
            "amiga_interfaces is not built: notes will be recorded and will "
            "not revise any bid"
        )
        return None
