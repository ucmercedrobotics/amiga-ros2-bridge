#!/usr/bin/env python3
"""The owner side of contract net: announce, collect, arbitrate.

One ``Auction`` per task we are trying to shed. It holds the bids that arrive,
knows when it has heard enough, and picks a winner. It does not send anything
and does not own the task -- the coordinator does both. Keeping it that way is
what lets arbitration be tested as a function of the bids that arrived rather
than as a consequence of a whole node.

Two ways an auction closes:

**The window expires.** The safe, always-available ending. Bidders that never
answered are simply not considered.

**Everyone capable has answered.** If the registry says four peers advertise
the capability and all four have bid -- including ones that bid *infeasible* --
there is nobody left to hear from and waiting out the rest of the window buys
nothing but delay. This is exactly why BID carries a ``feasible`` flag instead
of a declining bidder staying silent: silence and "I cannot" are different
facts and the wire distinguishes them.

Arbitration is deliberately dull. Lowest cost wins; ETA breaks a tie; robot ID
breaks that. Dull matters here because every robot must be able to predict the
outcome from the same bids -- a losing bidder that disagrees about who won is a
double-assigned task.

No ROS, no radio, no I/O.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from amiga_ros2_comms.codec import Bid

from .model import Task


@dataclass(frozen=True)
class ReceivedBid:
    """One peer's answer, as it arrived."""

    bidder_id: int
    task_id: int
    cost: int
    eta_s: int
    feasible: bool
    at: float

    @property
    def rank(self) -> "tuple[int, int, int]":
        """Sort key. Lower is better, all the way down.

        Cost first because that is what the bidder was asked to minimise. ETA
        second, because between two equally costly offers the sooner one is
        worth more. Robot ID last, and only to make the result total: two bids
        that tie on everything else must still resolve the same way on every
        robot that heard them, and an arbitrary-but-fixed tiebreak is the only
        thing that guarantees it.
        """
        return (self.cost, self.eta_s, self.bidder_id)


@dataclass
class Auction:
    """One task offered to the fleet, and the answers so far."""

    task: Task
    #: Robot IDs that advertised the required capability when we announced.
    #: A snapshot: a peer that appears later may still bid and be considered,
    #: it just does not hold the auction open.
    expected_bidders: "frozenset[int]" = frozenset()
    opened_at: float = 0.0
    closes_at: float = 0.0
    #: Copies of the TASK_ANNOUNCE put on the air. Announcements are broadcasts
    #: and therefore best-effort; re-announcing is how they reach a robot that
    #: was transmitting when the first went out.
    announcements: int = 0
    next_announce_at: float = 0.0
    _bids: Dict[int, ReceivedBid] = field(default_factory=dict)

    # ------------------------------------------------------------------
    # Collecting
    # ------------------------------------------------------------------

    def record(self, bid: Bid, now: float) -> bool:
        """Take one BID for this task. False if it was not for us or is stale.

        A second bid from the same peer replaces the first. Peers re-bid when
        their situation changes, and the freshest offer is the one they mean;
        keeping both would let a bidder win on a number it has withdrawn.
        """
        if int(bid.task_id) != self.task.task_id:
            return False
        bidder = int(bid.src)
        previous = self._bids.get(bidder)
        if previous is not None and previous.at > now:
            # Out-of-order arrival. Rare, but the newest bid must win on time,
            # not on arrival order, or a delayed duplicate undoes a withdrawal.
            return False
        self._bids[bidder] = ReceivedBid(
            bidder_id=bidder,
            task_id=int(bid.task_id),
            cost=int(bid.cost),
            eta_s=int(bid.eta_s),
            feasible=bool(bid.feasible),
            at=now,
        )
        return True

    def forget(self, robot_id: int) -> None:
        """Drop a bidder entirely -- its bid *and* our expectation of one.

        Called when a peer ages out of the registry, and when a GRANT to it
        could not be delivered. Both mean the same thing here: an offer from a
        robot we can no longer reach, which would burn a whole retransmit
        campaign to discover a second time.

        Dropping it from ``expected_bidders`` as well as from the bids is the
        half that is easy to miss and matters most. An auction closes early
        once everyone it is waiting for has answered; a departed peer left in
        that set is one the auction waits out its entire window for, and after
        a failed GRANT it is what stops the runner-up from ever being offered
        the task.
        """
        robot_id = int(robot_id)
        self._bids.pop(robot_id, None)
        self.expected_bidders = self.expected_bidders - {robot_id}

    # ------------------------------------------------------------------
    # Closing
    # ------------------------------------------------------------------

    def should_close(self, now: float) -> bool:
        return now >= self.closes_at or self.heard_from_everyone()

    def heard_from_everyone(self) -> bool:
        """Whether every peer we expected to answer has.

        False when we expected nobody: an empty expectation means the registry
        was empty at announce time, and closing instantly on that would give a
        peer whose heartbeat is merely late no chance at all. That case waits
        out the window, which is the honest thing to do when we do not know who
        is listening.
        """
        if not self.expected_bidders:
            return False
        return self.expected_bidders.issubset(self._bids)

    def winner(self) -> Optional[ReceivedBid]:
        """The best feasible bid, or None if nobody viable answered."""
        viable = [bid for bid in self._bids.values() if bid.feasible]
        if not viable:
            return None
        return min(viable, key=lambda bid: bid.rank)

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    @property
    def bids(self) -> "List[ReceivedBid]":
        """Every bid received, best first."""
        return sorted(self._bids.values(), key=lambda bid: bid.rank)

    @property
    def feasible_bids(self) -> "List[ReceivedBid]":
        return [bid for bid in self.bids if bid.feasible]

    def __len__(self) -> int:
        return len(self._bids)
