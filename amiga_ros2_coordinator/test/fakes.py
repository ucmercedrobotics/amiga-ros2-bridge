#!/usr/bin/env python3
"""Everything the coordinator talks to, faked, and a clock the test drives.

The engine takes its transport, its navigation, its mission stack, its clock
and its two reasoning points as injected objects, so the acceptance suite can
replace all of them and still exercise the real state machine. Nothing here
sleeps, nothing is probabilistic, and nothing needs ROS: an auction window, a
bid backoff and a peer liveness timeout are all just numbers the
test moves.

The important fake is ``FakeReliability``. It is *not* a working radio -- it
records what was sent and hands back a Future the test resolves by hand. That
is what makes unassigned-until-ACKed testable: "reliability reports the GRANT
delivered" and "reliability reports it failed" are two lines in a test rather
than two link conditions to arrange.

``HardInterruptDetector`` is the negative-space fake. The nav and mission fakes
carry the abort-style methods a careless implementation might reach for, and
the safe-preemption test asserts none of them was ever called.
"""

import os
import sys
from concurrent.futures import Future
from dataclasses import dataclass, field
from typing import Callable, List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import (  # noqa: E402
    Bid,
    Grant,
    Heartbeat,
    Message,
    TaskAnnounce,
)
from amiga_ros2_comms.reliability import (  # noqa: E402
    CompletedNote,
    Outcome,
    text_bytes_per_fragment,
)

#: What one note fragment carries at the default payload budget. Taken from the
#: real splitter rather than restated, so the fake's fragment count stays the
#: number the radio would actually produce.
NOTE_TEXT_BYTES = text_bytes_per_fragment()

from amiga_ros2_comms.codec import Target  # noqa: E402
from amiga_ros2_coordinator.vocabulary.model import Task  # noqa: E402


class Clock:
    """Monotonic seconds the test sets by hand.

    Time only moves when a test says so, which is what makes every deadline in
    this layer -- announce windows, bid backoffs, peer timeouts --
    exact rather than approximately observed.
    """

    def __init__(self, start: float = 1000.0):
        self.now = float(start)

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> float:
        self.now += float(seconds)
        return self.now


@dataclass
class SentReliable:
    """One ``send_reliable`` call, and the Future the test will resolve."""

    dst: int
    msg: Message
    future: Future

    def deliver(self) -> None:
        """Report the message acknowledged by ``dst``."""
        self.future.set_result(Outcome.DELIVERED)

    def fail(self) -> None:
        """Report the retransmit budget spent without an ACK."""
        self.future.set_result(Outcome.FAILED)


class FakeReliability:
    """The transport port: records sends, replays inbound, resolves by hand.

    Deliberately does not deliver anything anywhere. A test that wants robot A
    to hear robot B's bid calls ``a.on_message(bid)`` itself, so what each
    robot heard is written in the test rather than emerging from a simulated
    medium -- and a suppression test can therefore deliver a rival bid at an
    exact point inside a backoff window.
    """

    def __init__(self, node_id: int = 1):
        self.node_id = int(node_id)
        self.broadcasts: "List[Message]" = []
        self.reliable: "List[SentReliable]" = []
        self._on_deliver: Optional[Callable[[Message], None]] = None
        self._on_note: Optional[Callable[[object], None]] = None
        #: Set to make the next send_reliable raise, for the refused-send path.
        self.refuse_reliable = False
        #: (task_id, text) per send_note, in order.
        self.notes: "List[tuple]" = []
        #: Completed inbound notes, as the real reassembler would hold them,
        #: keyed (src, task_id). Tests fill this with ``note_arrives``.
        self.inbound_notes: dict = {}
        #: Set to make the next send_note raise, for the note-failed path.
        self.refuse_note = False

    # -- the three-item contract the real layer offers -------------------

    def send_broadcast(self, msg: Message) -> bool:
        # The real layer stamps identity on the way out; stamping it here keeps
        # a recorded message readable in an assertion.
        msg.src = self.node_id
        msg.seq = len(self.broadcasts) + len(self.reliable)
        self.broadcasts.append(msg)
        return True

    def send_reliable(self, dst: int, msg: Message) -> Future:
        if self.refuse_reliable:
            raise RuntimeError("fake reliability refused the send")
        msg.src = self.node_id
        msg.seq = len(self.broadcasts) + len(self.reliable)
        future: Future = Future()
        self.reliable.append(SentReliable(dst=int(dst), msg=msg, future=future))
        return future

    def send_note(self, task_id: int, text: str) -> int:
        if self.refuse_note:
            raise RuntimeError("fake reliability refused the note")
        self.notes.append((int(task_id), text))
        # The real splitter's arithmetic, so a test can assert a note went out
        # as more than one packet without reaching into the codec.
        return max(1, -(-len(text.encode("utf-8")) // NOTE_TEXT_BYTES))

    def completed_note(self, src: int, task_id: int):
        return self.inbound_notes.get((int(src), int(task_id)))

    def set_on_deliver(self, callback) -> None:
        self._on_deliver = callback

    def set_on_note(self, callback) -> None:
        self._on_note = callback

    # -- test-side helpers ----------------------------------------------

    def deliver(self, msg: Message) -> None:
        """Feed one inbound message up, as the real layer would after dedup."""
        if self._on_deliver is not None:
            self._on_deliver(msg)

    def note_arrives(self, src: int, task_id: int, text: str, at: float = 0.0):
        """A note from ``src`` about ``task_id`` arrived whole.

        Does both halves of what the real layer does: puts it in the lookup the
        announce path consults, and calls the completed-note callback. A test
        controls the *order* of this against the announcement, which is the
        thing the whole design turns on.
        """
        note = CompletedNote(
            src=int(src),
            task_id=int(task_id),
            note_id=1,
            text=text,
            completed_at=at,
        )
        self.inbound_notes[(int(src), int(task_id))] = note
        if self._on_note is not None:
            self._on_note(note)
        return note

    def sent(self, message_type) -> "List[Message]":
        """Every broadcast of a given type, in order."""
        return [m for m in self.broadcasts if isinstance(m, message_type)]

    @property
    def announces(self) -> "List[TaskAnnounce]":
        return self.sent(TaskAnnounce)

    @property
    def bids(self) -> "List[Bid]":
        return self.sent(Bid)

    @property
    def heartbeats(self) -> "List[Heartbeat]":
        return self.sent(Heartbeat)

    @property
    def grants(self) -> "List[SentReliable]":
        return [s for s in self.reliable if isinstance(s.msg, Grant)]

    @property
    def last_grant(self) -> Optional[SentReliable]:
        grants = self.grants
        return grants[-1] if grants else None


class HardInterruptDetector:
    """Records any attempt to forcibly stop what the robot is doing.

    Mixed into the nav and mission fakes. The methods exist *so that* the
    safe-preemption test can assert they were never called: an assertion that
    an absent method was not called would pass no matter what the coordinator
    did.
    """

    def __init__(self):
        self.hard_interrupts: "List[str]" = []

    def cancel_current_action(self) -> None:
        self.hard_interrupts.append("cancel_current_action")

    def abort(self) -> None:
        self.hard_interrupts.append("abort")

    def stop(self) -> None:
        self.hard_interrupts.append("stop")


class FakeNav(HardInterruptDetector):
    """The navigation port. Answers from fields the test sets."""

    def __init__(
        self,
        eta_sec: float = 60.0,
        reachable: bool = True,
        location: Optional[Target] = None,
    ):
        super().__init__()
        self.eta_sec = eta_sec
        self.reachable = reachable
        # A real orchard fix, from examples/quad.xml. Target.none() would be
        # the other honest default, but a fake that reports no position makes
        # every heartbeat test about the no-fix path by accident.
        self.location = location or Target.gps(37.366449, -120.423065)
        self.calls: "List[str]" = []

    def eta(self, target: Target) -> float:
        self.calls.append("eta")
        return self.eta_sec

    def can_reach(self, target: Target) -> bool:
        self.calls.append("can_reach")
        return self.reachable

    def current_location(self) -> Optional[Target]:
        return self.location


class FakeMission(HardInterruptDetector):
    """The mission / behaviour-tree port. Records what was done to the mission."""

    def __init__(
        self,
        can_absorb: bool = True,
        battery: int = 90,
        current_task: int = 0,
    ):
        super().__init__()
        self.can_absorb_answer = can_absorb
        self.battery = battery
        self.current_task = current_task
        self.absorbed: "List[Task]" = []
        self.released: "List[Task]" = []
        self.transferred: "List[Task]" = []
        self.asked: "List[Task]" = []

    def can_absorb(self, task: Task) -> bool:
        self.asked.append(task)
        return self.can_absorb_answer

    def absorb(self, task: Task) -> None:
        self.absorbed.append(task)

    def release(self, task: Task) -> None:
        self.released.append(task)

    def mark_transferred(self, task: Task) -> None:
        self.transferred.append(task)

    def current_task_id(self) -> int:
        return self.current_task

    def battery_percent(self) -> int:
        return self.battery


class FakePreemption:
    """The blackboard flag, as a pair of fields and a history."""

    def __init__(self):
        self.requested = False
        self.reason = ""
        self.requests: "List[str]" = []
        self.clears = 0

    def request_yield(self, reason: str) -> None:
        self.requested = True
        self.reason = reason
        self.requests.append(reason)

    def clear(self) -> None:
        self.requested = False
        self.reason = ""
        self.clears += 1


@dataclass
class EventLog:
    """Captures the session's log line stream, so warnings can be asserted on."""

    entries: "List[tuple[str, str]]" = field(default_factory=list)

    def __call__(self, level: str, message: str) -> None:
        self.entries.append((level, message))

    @property
    def warnings(self) -> "List[str]":
        return [message for level, message in self.entries if level == "warn"]

    def mentioning(self, needle: str) -> "List[str]":
        return [message for _, message in self.entries if needle in message]
