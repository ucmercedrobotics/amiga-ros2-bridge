#!/usr/bin/env python3
"""The bidder side of contract net: fitness, backoff, suppression.

When a peer announces a task we could do, the naive thing is to answer at once.
On a shared half-duplex radio that is the worst thing: every capable robot
answers in the same instant, the packets collide, and the announcer hears
nothing from the fleet it just asked.

So a bid waits, and **how long it waits encodes how good it is**. Backoff is
fitness-proportional -- the best-suited robot waits the shortest time -- so the
strongest bid transmits into a quiet channel. Meanwhile every robot is
listening: a bidder that overhears a *better* bid for the same task while its
own timer is still running **suppresses its bid entirely and never transmits**.

Both halves matter, and the second is the one that saves the airtime. Ordering
alone would still put N packets on the air for N bidders; suppression means a
strong bid silences the weak ones before they cost anything. The announcer gets
the bid it would have chosen anyway, and the channel carries a fraction of the
traffic. It is the same trick as CSMA collision avoidance, with fitness in
place of randomness.

The randomness that remains is a small jitter, and it is there for one specific
case: two robots with *identical* fitness would otherwise wait identical times
and collide, which is precisely the situation backoff exists to prevent. The
RNG is injected so tests are deterministic.

No ROS, no radio, no I/O.
"""

import random
from dataclasses import dataclass
from typing import Callable, Optional

from amiga_ros2_comms.codec import COST_MAX, ETA_MAX_S, ETA_RESOLUTION_S

from .model import Fitness, Task

#: Default longest a bidder will sit on a bid, in seconds. The worst-fitting
#: robot waits about this long; the best waits about none of it.
#:
#: Must be comfortably shorter than the announcer's collection window, or the
#: best bid arrives after the auction closed and the fitness ordering achieves
#: the exact opposite of its purpose. The coordinator checks the two against
#: each other at construction and complains.
DEFAULT_MAX_BACKOFF_SEC = 2.0

#: Fraction of the backoff window added as random jitter. Small on purpose: it
#: exists only to separate identically-fit bidders, and any larger would start
#: reordering bidders that fitness had already ordered.
DEFAULT_JITTER_FRACTION = 0.05


def backoff_for(
    cost: int,
    max_backoff_sec: float = DEFAULT_MAX_BACKOFF_SEC,
    jitter_fraction: float = DEFAULT_JITTER_FRACTION,
    rng: Optional[random.Random] = None,
) -> float:
    """Seconds to wait before transmitting a bid of this cost.

    Linear in cost over the full 0..COST_MAX range, which is what makes the
    wait comparable between robots: every bidder maps the same cost to the same
    delay, so the ordering on the air is the ordering of the bids.
    """
    fraction = min(max(int(cost), 0), COST_MAX) / COST_MAX
    delay = max_backoff_sec * fraction
    if jitter_fraction:
        source = rng or random
        delay += source.uniform(0.0, max_backoff_sec * jitter_fraction)
    return delay


def quantized_eta(eta_sec: float) -> int:
    """Clamp and round an ETA to what the wire can carry.

    ETA rides in one byte at 4 s/LSB, so 1020 s is the ceiling. Clamping here
    rather than letting the encoder raise means an honest bid on a very distant
    task becomes a maximally unattractive bid instead of an exception.
    """
    eta = max(0.0, float(eta_sec))
    quantized = int(round(eta / ETA_RESOLUTION_S)) * ETA_RESOLUTION_S
    return min(quantized, ETA_MAX_S)


def default_fitness(
    task: Task,
    eta_sec: float,
    battery: int,
    busy: bool,
    max_eta_sec: float = float(ETA_MAX_S),
) -> Fitness:
    """The stock cost function: mostly travel time, plus a penalty for being busy.

    Unitless and 0..COST_MAX, because that is the range the wire carries and the
    range ``backoff_for`` maps over. Only ever compared against other bids on
    the same task, so the absolute value means nothing and the *ordering* means
    everything.

    Deliberately simple and deliberately replaceable -- the coordinator takes a
    fitness function as a parameter. A real deployment will want energy, tool
    state and row geometry in here. What must not change is the range and the
    direction: lower is better, 0..255.
    """
    travel = min(max(eta_sec, 0.0) / max_eta_sec, 1.0)

    # A busy robot can still be the right answer -- it may be the only one with
    # the capability -- so this is a penalty and not a veto. Vetoing here would
    # make a fleet with no idle robot unable to redistribute anything.
    load = 0.35 if busy else 0.0

    # Battery enters as a penalty that is flat until it starts to matter, so a
    # robot at 90% and one at 70% bid alike and one at 15% bids badly.
    headroom = max(0, min(int(battery), 100))
    depletion = 0.0 if headroom >= 50 else (50 - headroom) / 50.0

    score = 0.55 * travel + load + 0.35 * depletion
    cost = int(round(min(score, 1.0) * COST_MAX))
    return Fitness(
        feasible=True,
        cost=cost,
        eta_s=quantized_eta(eta_sec),
        reason=f"travel={travel:.2f} load={load:.2f} depletion={depletion:.2f}",
    )


@dataclass
class PendingBid:
    """A bid we intend to send, once its backoff elapses and nothing better lands.

    Exists only during the backoff. It is either transmitted, suppressed by a
    better overheard bid, or cancelled because the auction closed without us.
    """

    task_id: int
    #: Everything needed to build the TASK_ANNOUNCE's answer.
    task: Task
    announcer_id: int
    fitness: Fitness
    send_at: float
    created_at: float = 0.0
    #: When it went on the air, or None while it is still suppressible. This is
    #: also what keeps the entry useful after transmission: the task has to be
    #: remembered until a GRANT arrives, or winning would mean absorbing a task
    #: whose location we no longer hold.
    sent_at: Optional[float] = None
    #: Best cost overheard from another bidder on this task, if any. Kept for
    #: the log line: "suppressed, 40 beat our 90" is a diagnosable sentence.
    best_overheard: Optional[int] = None

    def outranked_by(self, cost: int, eta_s: int, bidder_id: int, our_id: int) -> bool:
        """Whether an overheard bid beats ours on the arbiter's own ordering.

        The comparison has to be *identical* to the announcer's arbitration
        (auction.ReceivedBid.rank), including the robot-ID tiebreak. If a
        bidder suppressed itself using a different rule than the one the
        announcer arbitrates with, the fleet could suppress the bid that would
        have won and leave the announcer choosing among the leftovers.
        """
        return (int(cost), int(eta_s), int(bidder_id)) < (
            int(self.fitness.cost),
            int(self.fitness.eta_s),
            int(our_id),
        )


def make_backoff(
    max_backoff_sec: float = DEFAULT_MAX_BACKOFF_SEC,
    jitter_fraction: float = DEFAULT_JITTER_FRACTION,
    rng: Optional[random.Random] = None,
) -> Callable[[int], float]:
    """Bind backoff parameters into a one-argument function of cost."""

    def backoff(cost: int) -> float:
        return backoff_for(cost, max_backoff_sec, jitter_fraction, rng)

    return backoff
