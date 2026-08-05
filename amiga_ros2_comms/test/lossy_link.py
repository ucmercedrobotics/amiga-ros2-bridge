#!/usr/bin/env python3
"""A lossy radio made of a deque, for testing the reliability layer.

Two things make the acceptance tests deterministic rather than merely likely to
pass, and both are here.

**Time is a variable.** :class:`FakeClock` is what the sessions read, so a test
advances four seconds by adding 4.0 to a float. A whole retransmit campaign,
and dedup TTLs measured in minutes, run in microseconds with no sleeps and no
timing-dependent flakes.

**Loss is a decision, not a probability.** :class:`Medium` drops frames by a
predicate the test writes, so "the first GRANT is lost" is exactly what
happens, every run. There is a seeded random rule for soak-style tests, but no
acceptance test depends on chance.

The medium models the two properties of a shared radio channel that this layer
actually cares about: every attached node hears a frame except the sender, and
nothing is delivered until it is pumped, so a frame in flight is genuinely in
flight. It models neither collisions nor time on air -- lora/virtual_medium.py
does that for the bridge, and this layer's behaviour does not depend on either.
"""

import os
import random
import sys
from collections import deque
from dataclasses import dataclass
from typing import Callable, List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import CodecError, Message, decode  # noqa: E402
from amiga_ros2_comms.reliability import (  # noqa: E402
    ReliabilityParams,
    ReliabilitySession,
)


class FakeClock:
    """Monotonic seconds under the test's control."""

    def __init__(self, start: float = 1000.0):
        self.now = float(start)

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> float:
        self.now += float(seconds)
        return self.now


@dataclass
class Frame:
    """One payload handed to the link, whether or not it survived."""

    index: int
    sender: int
    payload: bytes
    #: Decoded for the test's convenience. None if it will not decode.
    msg: Optional[Message]
    dropped: bool = False

    @property
    def kind(self) -> str:
        return type(self.msg).__name__ if self.msg is not None else "undecodable"


class Medium:
    """A shared channel between sessions. Delivers only when pumped."""

    def __init__(self):
        self.frames: List[Frame] = []
        self._nodes = {}
        self._inbox = deque()
        #: Predicate over a Frame. True drops it. Rewritten freely by tests.
        self.drop_rule: Callable[[Frame], bool] = lambda frame: False

    # -- wiring ---------------------------------------------------------

    def attach(self, session: ReliabilitySession) -> ReliabilitySession:
        self._nodes[session.node_id] = session
        return session

    def sender_for(self, node_id: int) -> Callable[[bytes], None]:
        """The ``send_frame`` callable a session at ``node_id`` should use."""

        def send(payload: bytes) -> None:
            self._transmit(node_id, payload)

        return send

    def _transmit(self, node_id: int, payload: bytes) -> None:
        try:
            msg = decode(payload)
        except CodecError:
            msg = None
        frame = Frame(index=len(self.frames), sender=node_id, payload=payload, msg=msg)
        frame.dropped = bool(self.drop_rule(frame))
        self.frames.append(frame)
        if frame.dropped:
            return
        for peer_id in self._nodes:
            # Everyone but the sender, like a real radio and like the LoRa
            # simulator's virtual medium.
            if peer_id != node_id:
                self._inbox.append((peer_id, payload))

    def pump(self, max_rounds: int = 1000) -> int:
        """Deliver everything queued, including whatever that queues in turn.

        Terminates because the only frame an inbound frame can provoke is an
        ACK, and ACKs provoke nothing. ``max_rounds`` turns a violation of that
        invariant into a failed test rather than a hung one.
        """
        delivered = 0
        while self._inbox:
            if delivered >= max_rounds:
                raise AssertionError(
                    f"medium still delivering after {max_rounds} frames; "
                    f"something is answering an answer"
                )
            node_id, payload = self._inbox.popleft()
            self._nodes[node_id].on_frame(payload)
            delivered += 1
        return delivered

    # -- inspection -----------------------------------------------------

    def sent(self, kind: Optional[str] = None, sender: Optional[int] = None):
        """Every frame put on the air, dropped or not, optionally filtered."""
        return [
            f
            for f in self.frames
            if (kind is None or f.kind == kind)
            and (sender is None or f.sender == sender)
        ]

    def delivered_frames(self, kind: Optional[str] = None):
        """Frames that actually survived the channel."""
        return [f for f in self.sent(kind) if not f.dropped]

    # -- loss rules -----------------------------------------------------

    @staticmethod
    def drop_all(frame: Frame) -> bool:
        return True

    @staticmethod
    def drop_kind(kind: str, count: Optional[int] = None) -> Callable[[Frame], bool]:
        """Drop the first ``count`` frames of a type (all of them if None)."""
        seen = {"n": 0}

        def rule(frame: Frame) -> bool:
            if frame.kind != kind:
                return False
            seen["n"] += 1
            return count is None or seen["n"] <= count

        return rule

    @staticmethod
    def drop_random(rate: float, seed: int = 0) -> Callable[[Frame], bool]:
        """Seeded Bernoulli loss. For soak tests, never for an assertion."""
        rng = random.Random(seed)
        return lambda frame: rng.random() < rate


class Recorder:
    """Collects what a session delivers upward."""

    def __init__(self):
        self.messages: List[Message] = []

    def __call__(self, msg: Message) -> None:
        self.messages.append(msg)

    def of_kind(self, cls) -> List[Message]:
        return [m for m in self.messages if isinstance(m, cls)]


class Fleet:
    """A medium, a clock, and some sessions on it. The usual test fixture."""

    def __init__(self, *node_ids: int, params: Optional[ReliabilityParams] = None):
        self.clock = FakeClock()
        self.medium = Medium()
        self.params = params or ReliabilityParams(
            retransmit_timeout_sec=2.0,
            max_retries=3,
            retransmit_backoff=1.0,
            dedup_ttl_sec=60.0,
        )
        self.sessions = {}
        self.inbox = {}
        for node_id in node_ids:
            recorder = Recorder()
            session = ReliabilitySession(
                node_id=node_id,
                send_frame=self.medium.sender_for(node_id),
                params=self.params,
                on_deliver=recorder,
                clock=self.clock,
            )
            self.medium.attach(session)
            self.sessions[node_id] = session
            self.inbox[node_id] = recorder

    def __getitem__(self, node_id: int) -> ReliabilitySession:
        return self.sessions[node_id]

    def tick(self) -> None:
        for session in self.sessions.values():
            session.tick()
        self.medium.pump()

    def advance(self, seconds: float, step: float = 0.1) -> None:
        """Run the clock forward, ticking and pumping as a node would.

        ``step`` stands in for the node's tick timer, so retransmit deadlines
        are noticed with the same granularity they would be in ROS rather than
        exactly on their deadline.
        """
        target = self.clock.now + seconds
        while self.clock.now < target:
            self.clock.advance(min(step, target - self.clock.now))
            self.tick()
