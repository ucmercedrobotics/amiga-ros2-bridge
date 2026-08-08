#!/usr/bin/env python3
"""The nouns this layer coordinates over: tasks, places, peers.

Small value objects, shared by the action schema, the local-node ports and the
state machine, so that "a task" means one thing across all three. They are
deliberately thinner than whatever the mission node calls a task internally:
what crosses the radio is a task ID, the set of behaviour-tree actions the work
needs, where it is, and a priority -- and coordination cannot decide anything on
fields the wire does not carry.

``Target`` and ``Capability`` are imported from the codec rather than restated,
because both are statements about the behaviour tree: a capability is an action
type the mission schema permits, and a target is a place a BT leaf can name.
Defining a second copy here is how the two would come to disagree about what a
robot can do.

Pure data. No ROS, no radio, no I/O.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from amiga_ros2_comms.codec import (
    CAP_MASK_BITS,
    PRIORITY_MAX,
    TASK_ID_MAX,
    TASK_NONE,
    XML_ELEMENT,
    Capability,
    Target,
    capabilities_in,
)


@dataclass
class Task:
    """A unit of work this robot may own, shed, or take on.

    One behaviour-tree subtree: an objective and everything done at it. That is
    why ``required_capabilities`` is a mask -- approaching tree 60 and sampling
    it is two action types, and a robot that has one without the other cannot
    execute the task even though it could execute a leaf of it.

    Everything here fits in a TASK_ANNOUNCE, which is the point: a task this
    layer cannot announce is a task it cannot delegate, and discovering that at
    encode time would be too late.
    """

    task_id: int
    #: Mask of the Capability indices the executing robot must advertise.
    required_capabilities: int
    #: Where the work is, in the terms the behaviour tree uses.
    location: Target
    #: Unitless, higher is more urgent.
    priority: int = 0
    #: Opaque handle back to whatever the mission node calls this task. This
    #: layer never interprets it; it hands it back on absorb/transfer so the
    #: mission node does not have to keep its own id mapping.
    payload: Optional[object] = None

    def __post_init__(self):
        if not TASK_NONE < int(self.task_id) <= TASK_ID_MAX:
            raise ValueError(
                f"task_id must be {TASK_NONE + 1}..{TASK_ID_MAX} "
                f"({TASK_NONE} is TASK_NONE), got {self.task_id}"
            )
        if not 0 <= int(self.required_capabilities) < (1 << CAP_MASK_BITS):
            raise ValueError(
                f"required_capabilities={self.required_capabilities} does not fit "
                f"the {CAP_MASK_BITS}-bit mask the wire carries"
            )
        if not 0 <= int(self.priority) <= PRIORITY_MAX:
            raise ValueError(f"priority={self.priority} outside 0..{PRIORITY_MAX}")
        if not isinstance(self.location, Target):
            raise TypeError(
                f"location must be a Target, got {type(self.location).__name__}"
            )

    @property
    def delegable(self) -> bool:
        """Whether this task is one another robot could be sent to do.

        Work with no place of its own is not. ``SampleLeaf`` on its own, or a
        subtree whose only movement is ``MoveToRelativeLocation``, describes an
        offset from *this* robot's pose -- announcing it would name somewhere
        the winner cannot find.
        """
        return self.location.placed


class TaskState(Enum):
    """Where a task this robot is responsible for currently stands.

    The states exist to make *unassigned-until-ACKed* a thing you can look at
    rather than a rule you have to remember. A task is OURS from the moment the
    mission node hands it to us until reliability confirms some other robot
    acknowledged the GRANT, and not one moment sooner: ANNOUNCED and GRANTED
    are both still ours.
    """

    #: Ours, being executed or waiting to be. No coordination in flight.
    OURS = "ours"
    #: Ours, announced to the fleet, collecting bids.
    ANNOUNCED = "announced"
    #: Ours, GRANTed to a winner, waiting for reliability to confirm delivery.
    GRANTED = "granted"
    #: Confirmed delivered to another robot. The only state that is not ours.
    TRANSFERRED = "transferred"
    #: Given up on locally -- dropped, held, or escalated to a human.
    RELINQUISHED = "relinquished"


@dataclass
class PeerRecord:
    """One row of the peer registry, assembled from HEARTBEATs.

    ``last_seen`` is what makes the row trustworthy rather than merely present:
    a peer's capabilities do not expire, but the claim that it is *there* does.
    """

    robot_id: int
    cap_mask: int = 0
    #: Where the peer last said it was. ``TargetKind.NONE`` when it has no fix,
    #: which is why this is a Target and not a bare coordinate pair: "I do not
    #: know where I am" is an answer a peer can give, and one that ought not be
    #: confused with an answer of zero.
    location: Optional[Target] = None
    #: Whole percent, 0..100.
    battery: int = 0
    #: Task the peer reported executing, or TASK_NONE when idle.
    current_task: int = TASK_NONE
    last_seen: float = 0.0

    @property
    def idle(self) -> bool:
        return self.current_task == TASK_NONE


@dataclass
class Fitness:
    """A bidder's self-assessment of an announced task.

    ``cost`` is the scalar that is actually compared -- unitless, lower is
    better, and only comparable between bids on the same task. ``eta_s`` rides
    along because the arbiter wants it for tie-breaks and the operator wants it
    for display.

    ``feasible`` false is a real answer rather than the absence of one: it tells
    the announcer this robot heard and is ruling itself out, which lets an
    auction close early instead of waiting out its whole window.
    """

    feasible: bool
    cost: int = 0
    eta_s: int = 0
    #: Free text for logs only. Never crosses the radio.
    reason: str = ""


@dataclass
class MissionDelta:
    """What changed about our own mission, handed to replan-and-verify.

    A description of the edit, not the new mission: the replanner owns the
    mission and only needs to know what moved. Populated one field at a time --
    a delta describes a single committed change.
    """

    #: Tasks no longer ours (transferred away, dropped).
    removed: "list[Task]" = field(default_factory=list)
    #: Tasks now ours (absorbed from a peer).
    added: "list[Task]" = field(default_factory=list)
    #: Why this delta happened. Diagnostics and prompt context, not dispatch.
    cause: str = ""
    #: What the peer that raised this work said about doing it, when a
    #: deliberative note reached us before we bid. Carried on the delta because
    #: this is the path that reaches the replanner, and the replanner is the
    #: only thing that can act on a sentence -- everything else in a delta is a
    #: field the fleet derived, and this is the one thing it could not have.
    note: str = ""


def capability_name(capability: int) -> str:
    """The XML element a capability stands for, e.g. ``"SampleLeaf"``.

    The element name rather than the enum name, because that is the word an
    operator sees in the mission they wrote and the word a prompt can be asked
    about. ``MoveToTreeID`` appears in the XML; ``MOVE_TO_TREE_ID`` appears
    nowhere a person looks.

    Peers may advertise capability bits from newer firmware than ours. That is
    a thing to log, not a thing to crash on.
    """
    try:
        return XML_ELEMENT[Capability(int(capability))]
    except (ValueError, KeyError):
        return f"CAP_{int(capability)}"


def capability_names(mask: int) -> "list[str]":
    """Every action a mask advertises, as XML element names."""
    return [capability_name(c) for c in capabilities_in(mask)]
