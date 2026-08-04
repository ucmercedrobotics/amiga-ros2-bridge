#!/usr/bin/env python3
"""The ports to this robot's own nodes. Defined here, implemented elsewhere.

The coordinator talks to navigation, to the mission/behaviour-tree stack, and
to whatever surface the behaviour tree reads its preemption flag from. None of
those are built here and none of them are ROS types here: each is a Protocol,
so the state machine is written against *the interface, not the behaviour*, and
the acceptance tests drive the whole thing against trivial fakes.

That split is the point of the layer. Swapping a fake nav for the real Nav2
client must not touch one line of the contract-net logic; if it does, the
boundary was drawn in the wrong place.

Three ports:

    NavInterface        can we get there, and how long would it take
    MissionInterface    can we take this on, and here is what changed
    PreemptionSignal    a flag the behaviour tree reads at safe tick points

The last one is the one to be careful about. Coordination events arrive
asynchronously and must never hard-interrupt a robot mid-action -- an
interrupted arm move is a broken arm move. So this layer *raises a flag* and a
reactive condition node in the existing behaviour tree yields at a tick point
it has chosen to be safe. There is deliberately no ``abort()``, ``cancel()`` or
``stop()`` anywhere in these protocols: the capability to hard-interrupt is
absent by construction rather than merely unused.

Pure interface. No ROS, no radio, no I/O.
"""

from typing import Optional, Protocol, runtime_checkable

from .model import Location, Task


@runtime_checkable
class NavInterface(Protocol):
    """What the coordinator needs to know from navigation, and nothing else.

    Three questions. Notably absent: anything that moves the robot.
    Coordination decides *what* this robot should be responsible for; the
    behaviour tree decides when to act on it.
    """

    def eta(self, location: Location) -> float:
        """Estimated seconds to reach ``location`` from where we are now.

        Used to build a bid. May be optimistic -- every bidder's is, and a bid
        is an offer rather than a promise -- but must be comparable between
        calls on the same robot, because that comparison is what orders our own
        candidate tasks.
        """

    def can_reach(self, location: Location) -> bool:
        """Whether a route exists at all.

        Separate from ``eta`` because "unreachable" and "far away" are
        different answers and collapsing them into a large ETA makes an
        impossible task merely unattractive.
        """

    def current_location(self) -> Optional[Location]:
        """Where we are, for HEARTBEAT and for bid context. None if unknown."""


@runtime_checkable
class MissionInterface(Protocol):
    """What the coordinator needs from the mission / behaviour-tree stack.

    The asymmetry between ``absorb`` and ``mark_transferred`` is the
    unassigned-until-ACKed rule showing through: we tell the mission node a
    task is gone only once another robot has acknowledged owning it, so the
    window in which a task is nobody's is never open.
    """

    def can_absorb(self, task: Task) -> bool:
        """Whether this robot could add ``task`` to its mission at all.

        A capacity and compatibility question, not a cost one -- cost is the
        bid. False here means we do not bid; a high cost means we bid badly.
        """

    def absorb(self, task: Task) -> None:
        """Take ``task`` into the local mission. Called after winning a GRANT."""

    def release(self, task: Task) -> None:
        """Take ``task`` back out of the local mission.

        The inverse of ``absorb``, for the case where a task was absorbed and
        the replan that followed rejected it. Also used when a locally handled
        give-up disposes of a task.
        """

    def mark_transferred(self, task: Task) -> None:
        """``task`` is another robot's problem now. Only ever called on a
        confirmed delivery -- never on the mere sending of a GRANT."""

    def current_task_id(self) -> int:
        """The task being executed, or TASK_NONE when idle. For HEARTBEAT."""

    def battery_percent(self) -> int:
        """Whole percent, 0..100. For HEARTBEAT and for anomaly context."""


@runtime_checkable
class PreemptionSignal(Protocol):
    """The blackboard flag a reactive behaviour-tree condition node reads.

    Setting it is a *request* to yield, honoured by the tree at a tick point it
    considers safe. This layer never learns whether or when that happened, and
    that ignorance is deliberate: a coordinator that waited for an
    acknowledgement of preemption would be a coordinator that could block on
    the robot finishing an action.
    """

    def request_yield(self, reason: str) -> None:
        """Raise the flag. Idempotent; the reason is for logs and the operator."""

    def clear(self) -> None:
        """Lower the flag, once the coordination event has been dealt with."""


class NullPreemption:
    """A preemption signal that goes nowhere.

    For bench runs and for tests that are not about preemption. Named rather
    than defaulted to ``None`` so that "no behaviour tree attached" is a thing
    the logs can say.
    """

    def __init__(self):
        self.requested = False
        self.reason = ""

    def request_yield(self, reason: str) -> None:
        self.requested = True
        self.reason = reason

    def clear(self) -> None:
        self.requested = False
        self.reason = ""
