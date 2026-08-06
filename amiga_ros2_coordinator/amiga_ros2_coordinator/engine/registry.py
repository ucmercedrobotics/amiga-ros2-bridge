#!/usr/bin/env python3
"""Who else is out there, what they can do, and whether they are still there.

HEARTBEATs go in, a table comes out. The table answers the two questions the
rest of the layer keeps asking: *which peers could possibly do this task* (so an
auction knows how many answers it is waiting for) and *is this peer still
alive* (so it does not wait for one that has been switched off).

The second question is why entries expire. Capabilities do not go stale -- a
robot that could spray still can -- but the claim that it is *there* does, and a
registry that never forgets is a registry that stalls an auction forever on a
robot that drove out of range an hour ago. Liveness is the whole reason this is
a table with a clock in it rather than a config file.

Aging is by **timeout, not by counting missed heartbeats**. A count needs a
known period, and the period is the sender's parameter, not ours; a peer that
slows its heartbeat down for its own reasons would be declared dead by a rule
that was only ever counting.

The clock is injected, like everywhere else here, so an hour of liveness
behaviour runs in microseconds with nothing sleeping.

No ROS, no radio, no I/O.
"""

from typing import Callable, Dict, Iterable, List, Optional

from amiga_ros2_comms.codec import Heartbeat, has_capabilities, target_of

from ..vocabulary.model import PeerRecord

#: Default seconds without a HEARTBEAT before a peer is considered gone.
#:
#: Wants to be a small multiple of the fleet's heartbeat period, so a peer
#: survives losing a couple of heartbeats to the radio -- broadcasts are
#: best-effort and losing one is ordinary. Three times the 10 s default period,
#: which tolerates two consecutive losses.
DEFAULT_PEER_TIMEOUT_SEC = 30.0


class PeerRegistry:
    """The fleet as this robot currently believes it to be."""

    def __init__(
        self,
        timeout_sec: float = DEFAULT_PEER_TIMEOUT_SEC,
        clock: Optional[Callable[[], float]] = None,
        on_change: Optional[Callable[[str, PeerRecord], None]] = None,
    ):
        if timeout_sec <= 0:
            raise ValueError("timeout_sec must be positive")
        self.timeout_sec = float(timeout_sec)
        self._clock = clock
        self._on_change = on_change
        self._peers: Dict[int, PeerRecord] = {}
        self.counters = {"observed": 0, "appeared": 0, "aged_out": 0}

    # ------------------------------------------------------------------
    # Inbound
    # ------------------------------------------------------------------

    def observe(self, heartbeat: Heartbeat, now: float) -> PeerRecord:
        """Fold one HEARTBEAT into the table.

        ``now`` is passed in rather than read from the clock because the
        coordinator already has the timestamp it stamped the whole inbound
        message with, and two different "now"s for one message is how a peer
        ends up aged out by a fraction of a millisecond it never got.
        """
        robot_id = int(heartbeat.src)
        record = self._peers.get(robot_id)
        appeared = record is None
        if record is None:
            record = PeerRecord(robot_id=robot_id)
            self._peers[robot_id] = record
            self.counters["appeared"] += 1

        record.cap_mask = int(heartbeat.cap_mask)
        record.location = target_of(heartbeat)
        record.battery = int(heartbeat.battery)
        record.current_task = int(heartbeat.cur_task)
        record.last_seen = now
        self.counters["observed"] += 1

        if appeared:
            self._notify("appeared", record)
        return record

    def age_out(self, now: float) -> "List[PeerRecord]":
        """Drop peers that have gone quiet. Returns the ones dropped.

        Returned rather than merely deleted because a peer disappearing is an
        event with consequences elsewhere -- an auction waiting on its bid can
        stop waiting -- and the caller should not have to diff the table to
        notice.
        """
        gone = [
            record
            for record in self._peers.values()
            if now - record.last_seen > self.timeout_sec
        ]
        for record in gone:
            del self._peers[record.robot_id]
            self.counters["aged_out"] += 1
            self._notify("aged_out", record)
        return gone

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get(self, robot_id: int) -> Optional[PeerRecord]:
        return self._peers.get(int(robot_id))

    def __contains__(self, robot_id: object) -> bool:
        return int(robot_id) in self._peers  # type: ignore[arg-type]

    def __len__(self) -> int:
        return len(self._peers)

    def __iter__(self) -> Iterable[PeerRecord]:
        return iter(list(self._peers.values()))

    @property
    def peers(self) -> "tuple[PeerRecord, ...]":
        """Snapshot of the table. A tuple because callers hand it to the
        reasoning step, which must not be able to mutate the registry."""
        return tuple(self._peers.values())

    def ids(self) -> "tuple[int, ...]":
        return tuple(sorted(self._peers))

    def capable(self, required: int) -> "tuple[PeerRecord, ...]":
        """Live peers advertising *every* action in ``required``.

        This is what lets an auction close early instead of waiting out its
        whole window: once every peer in here has answered -- bid or declined
        -- there is nobody left to hear from. A peer that is capable but silent
        keeps the window open, which is correct: it may simply be busy.

        All of the actions and not any of them, because a task is a subtree: a
        robot that can sample but cannot drive to the tree would keep the
        window open waiting for a bid it can never sensibly make.
        """
        return tuple(
            record
            for record in self._peers.values()
            if has_capabilities(record.cap_mask, required)
        )

    def stats(self) -> dict:
        counters = dict(self.counters)
        counters["peers"] = len(self._peers)
        return counters

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _notify(self, event: str, record: PeerRecord) -> None:
        if self._on_change is not None:
            self._on_change(event, record)
