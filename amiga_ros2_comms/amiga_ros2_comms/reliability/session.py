#!/usr/bin/env python3
"""Make bytes arrive, and arrive once. The engine, with no ROS in it.

One instance per robot. It owns sequence numbers, ACKs, retransmit timers and
the dedup cache, and it owes the layer above exactly three things::

    send_reliable(dst, msg) -> Future[Outcome]   unicast, ACKed, retransmitted
    send_broadcast(msg)     -> bool              one shot, no timer
    on_deliver(msg)                              inbound, decoded, once each

What it does *not* do is decide anything. It never looks at a task ID, never
compares a bid, never forms an opinion about who should own what. It reports
delivered-or-failed and hands the coordinator the facts; the coordinator draws
the conclusions. The one exception is addressing -- "who is this for" -- and
that is quarantined in addressing.py.

It also never fragments. Every built message fits one packet by construction
(codec.MAX_MESSAGE_BYTES is 13), so a message too large to send is a design
error to be raised, not a stream to be split.

Two conventions make this testable without a radio, and are worth stating:

**The clock is injected.** ``clock()`` returns monotonic seconds and defaults to
time.monotonic, but a test passes a counter it controls and drives an entire
retransmit campaign, TTL expiry included, in microseconds with no sleeping and
no flakiness.

**The link is a callable.** ``send_frame(payload)`` takes bytes and is expected
to lose some of them. In the node it publishes to /lora/tx; in the tests it is a
lossy in-memory pipe to another session. Nothing here knows the difference.

Threading: every public method is safe to call from any thread. Callbacks --
``on_deliver`` and future resolutions -- are always invoked *after* the lock is
released, so a coordinator that sends from inside its own deliver callback
re-enters cleanly instead of deadlocking.
"""

import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional

from ..codec import (
    DEFAULT_MAX_PAYLOAD_BYTES,
    ROBOT_ID_NONE,
    SEQ_MAX,
    SRC_MAX,
    Ack,
    CodecError,
    Message,
    ReservedMessageType,
    UnknownMessageType,
    decode,
    encode,
)
from .addressing import addressee_of, is_reliable, is_unicast
from .dedup import DedupCache


class Outcome(Enum):
    """What became of a reliable send. The only two answers the caller gets.

    Deliberately binary. "Delivered" means a specific robot acknowledged a
    specific ``(src, seq)``, which is the fact the coordinator needs to hold
    task ownership consistent; every other ending is a failure and the
    distinctions between them (timed out, refused, never encoded) are
    diagnostics that live in the counters, not decisions the caller should be
    branching on.
    """

    DELIVERED = "delivered"
    FAILED = "failed"


class ReliabilityError(RuntimeError):
    """A caller asked for something this layer will not do.

    Raised synchronously, never reported as a failed delivery: sending a
    HEARTBEAT reliably or a GRANT best-effort is a bug in the caller, not a bad
    day on the radio, and a Future resolving FAILED would bury it.
    """


@dataclass(frozen=True)
class ReliabilityParams:
    """Everything tunable, in one object.

    ``retransmit_timeout_sec`` is the one to get right. It must exceed the
    round trip -- the message's time on air plus the ACK's, plus whatever the
    bridge's queues add -- or retransmits are emitted while the first ACK is
    still in flight, and the duplicates congest a half-duplex channel faster
    than ACKs can clear it. The node checks the configured value against the
    airtime model at startup and warns; see node.py.
    """

    #: Wait before the first retransmit. Floor is the round-trip time on air.
    #: The default clears that floor with headroom from SF7 through SF10, which
    #: is the practical orchard range. Time on air doubles per spreading
    #: factor, so SF11 and SF12 need this raised -- the node says so at startup
    #: rather than leaving it to be discovered as a retransmit storm.
    retransmit_timeout_sec: float = 3.0
    #: Retransmits *after* the original, so 3 means up to 4 copies on the air.
    max_retries: int = 3
    #: Multiplier applied per retransmit. 1.0 is a flat timer.
    retransmit_backoff: float = 1.5
    #: Ceiling on the backed-off interval, so a long campaign stays bounded.
    max_retransmit_timeout_sec: float = 10.0
    #: How long a delivered (src, seq) is remembered.
    dedup_ttl_sec: float = 120.0
    #: Hard entry cap on the dedup cache.
    dedup_max_entries: int = 512
    #: Reliable sends allowed in flight at once. Beyond it, sends fail fast.
    max_pending: int = 32
    #: Per-message budget handed to the codec.
    max_payload_bytes: int = DEFAULT_MAX_PAYLOAD_BYTES

    def __post_init__(self):
        if self.retransmit_timeout_sec <= 0:
            raise ValueError("retransmit_timeout_sec must be positive")
        if self.max_retries < 0:
            raise ValueError("max_retries must be >= 0")
        if self.retransmit_backoff < 1.0:
            raise ValueError("retransmit_backoff must be >= 1.0")
        if self.max_retransmit_timeout_sec < self.retransmit_timeout_sec:
            raise ValueError(
                "max_retransmit_timeout_sec must be >= retransmit_timeout_sec"
            )
        if self.max_pending < 1:
            raise ValueError("max_pending must be >= 1")


@dataclass
class _Pending:
    """One reliable send awaiting its ACK."""

    seq: int
    dst: int
    payload: bytes
    future: Future
    #: Retransmits sent so far. The original transmission is not counted.
    attempts: int = 0
    deadline: float = 0.0
    kind: str = ""


class ReliabilitySession:
    """The reliability layer, minus the ROS wiring."""

    def __init__(
        self,
        node_id: int,
        send_frame: Callable[[bytes], None],
        params: Optional[ReliabilityParams] = None,
        on_deliver: Optional[Callable[[Message], None]] = None,
        clock: Callable[[], float] = None,
        on_event: Optional[Callable[[str, str], None]] = None,
    ):
        if not ROBOT_ID_NONE < int(node_id) <= SRC_MAX:
            # A robot transmitting as ROBOT_ID_NONE would have its ACKs
            # addressed to nobody, so this is refused rather than clamped.
            raise ValueError(
                f"node_id must be {ROBOT_ID_NONE + 1}..{SRC_MAX} "
                f"({ROBOT_ID_NONE} is ROBOT_ID_NONE), got {node_id}"
            )

        self.node_id = int(node_id)
        self.params = params or ReliabilityParams()
        self._send_frame = send_frame
        self._on_deliver = on_deliver
        self._clock = clock or time.monotonic
        self._on_event = on_event

        self._lock = threading.RLock()
        self._next_seq = 0
        self._pending: Dict[int, _Pending] = {}
        self._dedup = DedupCache(
            ttl_sec=self.params.dedup_ttl_sec,
            max_entries=self.params.dedup_max_entries,
        )
        self._counters = {
            "tx_broadcast": 0,
            "tx_reliable": 0,
            "tx_retransmits": 0,
            "tx_acks": 0,
            "tx_link_errors": 0,
            "tx_delivered": 0,
            "tx_failed": 0,
            "tx_rejected": 0,
            "rx_frames": 0,
            "rx_delivered": 0,
            "rx_duplicates": 0,
            "rx_acks": 0,
            "rx_self": 0,
            "rx_unknown_type": 0,
            "rx_reserved_type": 0,
            "rx_malformed": 0,
            "ack_unmatched": 0,
            "ack_wrong_src": 0,
        }

    # ------------------------------------------------------------------
    # Sequence numbers
    # ------------------------------------------------------------------

    def next_seq(self) -> int:
        """The next outgoing sequence number for this sender.

        Monotonic, wrapping at 16 bits. Wrap is safe only because the dedup TTL
        is vastly shorter than the time it takes to emit 65536 messages on a
        LoRa link -- an entry for the previous lap is long gone before the
        number comes round again.
        """
        with self._lock:
            seq = self._next_seq
            self._next_seq = (self._next_seq + 1) & SEQ_MAX
            return seq

    # ------------------------------------------------------------------
    # Outbound
    # ------------------------------------------------------------------

    def send_reliable(self, dst: int, msg: Message) -> "Future[Outcome]":
        """Send ``msg`` to ``dst``, confirmed. Returns immediately.

        The returned Future resolves to Outcome.DELIVERED when ``dst`` ACKs, or
        Outcome.FAILED once the retransmit budget is spent. It is resolved from
        whichever thread calls :meth:`tick` or :meth:`on_frame`, so a
        done-callback attached to it runs there too -- keep it short, and do
        the real work on your own executor.

        ``msg.src`` and ``msg.seq`` are overwritten: message identity belongs to
        this layer, and a caller-supplied seq is how you get two different
        messages sharing an ID and one of them silently deduped away.

        Raises ReliabilityError if ``msg`` is not a unicast type, or if it names
        an addressee other than ``dst``.
        """
        if isinstance(msg, Ack):
            raise ReliabilityError(
                "ACKs are emitted by this layer, not sent by the caller"
            )
        if not is_reliable(msg):
            raise ReliabilityError(
                f"{type(msg).__name__} is a broadcast type; use send_broadcast(). "
                f"Reliability follows addressing: there is no set of ACKs that "
                f"would confirm a broadcast."
            )
        if not 1 <= int(dst) <= SRC_MAX:
            raise ReliabilityError(
                f"dst must be 1..{SRC_MAX} (0 is ROBOT_ID_NONE), got {dst}"
            )

        # The body names its own addressee, so the two ways of saying where
        # this is going must agree. They disagree when a caller edits one and
        # not the other, which produces a GRANT that one robot acts on and a
        # different robot acknowledges -- precisely the double-assignment this
        # layer exists to prevent.
        named = addressee_of(msg)
        if named is not None and named != int(dst):
            raise ReliabilityError(
                f"{type(msg).__name__} is addressed to {named} but send_reliable "
                f"was told dst={dst}"
            )

        future: "Future[Outcome]" = Future()
        with self._lock:
            if len(self._pending) >= self.params.max_pending:
                # Fail fast rather than grow. An unbounded pending table on a
                # link that has gone quiet turns a dead radio into a dead node,
                # and the caller can retry a rejection but cannot retry a hang.
                self._counters["tx_rejected"] += 1
                self._event(
                    "warn",
                    f"{len(self._pending)} reliable sends already in flight "
                    f"(max_pending); rejecting {type(msg).__name__}",
                )
                future.set_result(Outcome.FAILED)
                return future

            msg.src = self.node_id
            msg.seq = self.next_seq()
            # Encoding failures raise: a field out of range is a caller bug, and
            # burying it in a FAILED future would make it look like radio loss.
            payload = encode(msg, max_payload_bytes=self.params.max_payload_bytes)

            self._pending[msg.seq] = _Pending(
                seq=msg.seq,
                dst=int(dst),
                payload=payload,
                future=future,
                deadline=self._clock() + self.params.retransmit_timeout_sec,
                kind=type(msg).__name__,
            )
            self._counters["tx_reliable"] += 1

        self._transmit(payload)
        return future

    def send_broadcast(self, msg: Message) -> bool:
        """Send ``msg`` to everyone, once. No ACK, no timer, no retry.

        Returns whether the packet reached the link. That is not a delivery
        report and must not be read as one -- nobody knows who heard it. If the
        coordinator needs a broadcast to land it repeats it itself, on its own
        schedule, for its own reasons; receivers dedup the copies.

        Raises ReliabilityError for a unicast type, which belongs on
        :meth:`send_reliable`.
        """
        if isinstance(msg, Ack):
            raise ReliabilityError(
                "ACKs are emitted by this layer, not sent by the caller"
            )
        if is_unicast(msg):
            raise ReliabilityError(
                f"{type(msg).__name__} is addressed to a single robot; use "
                f"send_reliable(dst, msg) so its loss is detected"
            )

        with self._lock:
            msg.src = self.node_id
            msg.seq = self.next_seq()
            payload = encode(msg, max_payload_bytes=self.params.max_payload_bytes)
            self._counters["tx_broadcast"] += 1

        return self._transmit(payload)

    # ------------------------------------------------------------------
    # Inbound
    # ------------------------------------------------------------------

    def on_frame(self, payload: bytes) -> None:
        """Feed one payload off the link. Never raises.

        Everything that can be wrong with inbound bytes has already been given
        a name by the codec, and all of them are counted and dropped here. A
        peer running newer firmware, a corrupted packet that survived the frame
        CRC, or a flat-out hostile transmitter must not be able to take this
        node down -- it is the only thing standing between the radio and the
        coordinator.
        """
        ack_payload: Optional[bytes] = None
        deliver: Optional[Message] = None
        confirmed: Optional[_Pending] = None

        with self._lock:
            self._counters["rx_frames"] += 1
            try:
                msg = decode(payload)
            except UnknownMessageType:
                self._counters["rx_unknown_type"] += 1
                return
            except ReservedMessageType:
                # FREEFORM. Allocated, unbuilt, and needs the fragmentation
                # this layer deliberately does not have.
                self._counters["rx_reserved_type"] += 1
                return
            except CodecError:
                self._counters["rx_malformed"] += 1
                return

            if isinstance(msg, Ack):
                # An ACK is transport bookkeeping, not traffic. It is never
                # deduped, never delivered upward and never itself ACKed: the
                # coordinator gets the delivered/failed outcome instead, which
                # is the fact it actually wanted.
                self._counters["rx_acks"] += 1
                confirmed = self._match_ack(msg)
                if confirmed is not None:
                    self._counters["tx_delivered"] += 1

            elif msg.src == self.node_id:
                # Our own transmission, echoed by a repeater or a loopback. The
                # simulator excludes the sender, but a real deployment may not,
                # and self-delivery would have us ACK ourselves.
                self._counters["rx_self"] += 1

            else:
                now = self._clock()
                if is_reliable(msg) and addressee_of(msg) == self.node_id:
                    # Before the dedup check, and independent of it. A duplicate
                    # arriving means our previous ACK was lost or is still in
                    # the air; staying silent because we have seen the message
                    # before would let the sender exhaust its retries and
                    # declare a delivered message failed. Re-ACK every copy,
                    # deliver only the first.
                    ack_payload = self._encode_ack(msg)

                # Note the absence of an addressed-to-someone-else filter. A
                # GRANT naming another robot is still delivered up (deduped,
                # unACKed) -- that is how losing bidders learn an auction
                # closed. Addressing decides who owes an ACK, not who may
                # listen.
                if self._dedup.first_sight((msg.src, msg.seq), now):
                    self._counters["rx_delivered"] += 1
                    deliver = msg
                else:
                    self._counters["rx_duplicates"] += 1

        # Every effect below runs with the lock released.
        if ack_payload is not None:
            self._transmit(ack_payload, is_ack=True)
        if confirmed is not None:
            self._finish(confirmed, Outcome.DELIVERED)
        if deliver is not None:
            self._dispatch(deliver)

    def _match_ack(self, ack: Ack) -> Optional[_Pending]:
        """Find the pending send this ACK confirms. Caller holds the lock."""
        if ack.ack_src != self.node_id:
            # Someone else's ACK, overheard on a shared medium. Not an error.
            return None
        pending = self._pending.get(ack.ack_seq)
        if pending is None:
            # Already resolved, already given up on, or never ours. The common
            # cause is benign: a retransmit and its ACK crossed, and this is the
            # second ACK for a send we closed on the first.
            self._counters["ack_unmatched"] += 1
            return None
        if ack.src != pending.dst:
            # A third robot acknowledging a message addressed to someone else.
            # Refused: "delivered" has to mean the intended owner has it, or the
            # ownership guarantee built on top of it means nothing.
            self._counters["ack_wrong_src"] += 1
            self._event(
                "warn",
                f"ACK for seq={ack.ack_seq} came from {ack.src}, "
                f"but it was addressed to {pending.dst}; ignoring",
            )
            return None
        del self._pending[ack.ack_seq]
        return pending

    def _encode_ack(self, msg: Message) -> bytes:
        """Build the ACK for ``msg``. Caller holds the lock."""
        return encode(
            Ack(
                src=self.node_id,
                seq=self.next_seq(),
                ack_src=msg.src,
                ack_seq=msg.seq,
            ),
            max_payload_bytes=self.params.max_payload_bytes,
        )

    # ------------------------------------------------------------------
    # Retransmit
    # ------------------------------------------------------------------

    def tick(self) -> None:
        """Drive retransmit timers. Call periodically; cheap when idle.

        One sweep of a small table rather than a timer object per message: the
        table is bounded by ``max_pending``, and a single periodic sweep cannot
        leak a timer when a send resolves early.
        """
        now = self._clock()
        retransmit: List[bytes] = []
        expired: List[_Pending] = []

        with self._lock:
            for seq, pending in list(self._pending.items()):
                if pending.deadline > now:
                    continue
                if pending.attempts >= self.params.max_retries:
                    del self._pending[seq]
                    self._counters["tx_failed"] += 1
                    expired.append(pending)
                    self._event(
                        "warn",
                        f"{pending.kind} seq={seq} to {pending.dst} unacknowledged "
                        f"after {pending.attempts + 1} transmissions; giving up",
                    )
                    continue
                pending.attempts += 1
                pending.deadline = now + self._interval(pending.attempts)
                self._counters["tx_retransmits"] += 1
                retransmit.append(pending.payload)
            self._dedup.prune(now)

        for payload in retransmit:
            self._transmit(payload)
        for pending in expired:
            self._finish(pending, Outcome.FAILED)

    def _interval(self, attempts: int) -> float:
        """Wait before the next retransmit, after ``attempts`` of them."""
        interval = self.params.retransmit_timeout_sec * (
            self.params.retransmit_backoff**attempts
        )
        return min(interval, self.params.max_retransmit_timeout_sec)

    # ------------------------------------------------------------------
    # Plumbing
    # ------------------------------------------------------------------

    def _transmit(self, payload: bytes, is_ack: bool = False) -> bool:
        """Hand a packet to the link. Never raises into a caller."""
        if is_ack:
            with self._lock:
                self._counters["tx_acks"] += 1
        try:
            self._send_frame(payload)
            return True
        except Exception as exc:  # noqa: BLE001 - any link fault, same handling
            with self._lock:
                self._counters["tx_link_errors"] += 1
            self._event("warn", f"link rejected a {len(payload)}-byte payload: {exc}")
            return False

    def _finish(self, pending: _Pending, outcome: Outcome) -> None:
        """Resolve a caller's future. Must run with the lock released."""
        if not pending.future.set_running_or_notify_cancel():
            return
        try:
            pending.future.set_result(outcome)
        except Exception as exc:  # noqa: BLE001 - a caller's done-callback threw
            self._event("warn", f"delivery callback raised: {exc}")

    def _dispatch(self, msg: Message) -> None:
        """Hand a message up. Must run with the lock released."""
        if self._on_deliver is None:
            return
        try:
            self._on_deliver(msg)
        except Exception as exc:  # noqa: BLE001 - the coordinator's problem, not ours
            self._event("warn", f"on_deliver raised for {type(msg).__name__}: {exc}")

    def _event(self, level: str, message: str) -> None:
        if self._on_event is not None:
            self._on_event(level, message)

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    def set_on_deliver(self, callback: Optional[Callable[[Message], None]]) -> None:
        """Install the inbound callback. Safe to call after construction."""
        with self._lock:
            self._on_deliver = callback

    @property
    def pending(self) -> int:
        """Reliable sends currently awaiting an ACK."""
        with self._lock:
            return len(self._pending)

    def stats(self) -> dict:
        with self._lock:
            counters = dict(self._counters)
            counters.update(self._dedup.stats())
            counters["pending"] = len(self._pending)
            counters["next_seq"] = self._next_seq
            return counters
