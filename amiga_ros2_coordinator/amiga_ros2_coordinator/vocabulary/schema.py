#!/usr/bin/env python3
"""The constrained vocabulary an interpretation is allowed to speak.

Reasoning happens in two places in this system -- deciding what to do about a
task we cannot finish, and reading a note another robot sent about one -- and
this file is what keeps either from becoming an open-ended *interface*. Each has
its own closed union, and neither can return anything else.

``interpret_anomaly`` returns one of exactly three typed actions:

    ReDelegate  -- shed this task to the fleet
    AddTask     -- take on new work we discovered
    DropTask    -- abandon this task, locally

Never free text. A model that emits a sentence emits something the state
machine has to parse, and a state machine that parses sentences has no
enumerable set of behaviours to test. Constraining the output to these three
means the reasoning step chooses *among decisions this layer already knows how
to execute*, and every path below it stays deterministic and testable with the
reasoning stubbed out.

The types are also what makes the stub honest: a scripted interpreter returning
``ReDelegate(task)`` exercises exactly the code path the real one will, because
the real one cannot return anything the stub could not.

``interpret_note`` returns one of exactly three bid revisions -- keep, revise,
withdraw -- and is deliberately weaker still, because its input comes off the
radio from an unauthenticated sender. See the section below.

Pure data. No ROS, no radio, no I/O.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Union

from amiga_ros2_comms.codec import ReasonCode

from .model import PeerRecord, Task


class LocalDisposition(Enum):
    """What to do with a task nobody will take.

    The give-up path still has to end somewhere, and "somewhere" is a decision
    with consequences: dropping it loses the work, holding it blocks the robot,
    and escalating it costs a human's attention. Naming the three means the
    choice is made by the interpretation step rather than defaulted to by
    whichever branch happened to be written first.
    """

    #: Abandon it. The work does not get done and everyone stops waiting.
    DROP = "drop"
    #: Keep it, unexecuted, and try again later.
    HOLD = "hold"
    #: Stop and ask an operator.
    REQUEST_HUMAN = "request_human"


@dataclass(frozen=True)
class ReDelegate:
    """Shed ``task`` to the fleet: announce it, collect bids, grant the best.

    ``fallback`` is what happens when the announce window closes with no viable
    bid. It is part of the decision rather than a separate later question,
    because by the time the window closes the context that justified the
    decision is gone.
    """

    task: Task
    #: ReasonCode carried in the announcement. Diagnostics, not dispatch.
    reason_code: int = ReasonCode.TASK_FAILED
    fallback: LocalDisposition = LocalDisposition.HOLD
    #: Free text broadcast alongside the announcement -- what a bidder should
    #: know before taking this on, in the words of whoever decided to shed it.
    #:
    #: Costs nothing to produce: the interpretation that chose to re-delegate
    #: is already a model call with the whole context in front of it, so the
    #: sentence comes out of a call this system was making anyway. What it costs
    #: is on the *receiving* side, where reading it makes the auction
    #: deliberative and therefore slow -- so an empty note is not a degraded
    #: version of a good one, it is the right answer whenever the announcement's
    #: structured fields already say everything true about the task.
    note: str = ""


@dataclass(frozen=True)
class AddTask:
    """Take on work we discovered -- an unmapped obstacle to inspect, a missed row."""

    task: Task
    reason_code: int = ReasonCode.UNSPECIFIED


@dataclass(frozen=True)
class DropTask:
    """Give this task up locally, without asking the fleet.

    Distinct from a ReDelegate whose fallback is DROP: this one never announces
    at all. Used when the task is not merely beyond *us* -- a spray task with
    an empty tank is worth offering to a peer, a spray task on a tree that has
    already been felled is not.
    """

    task: Task
    disposition: LocalDisposition = LocalDisposition.DROP
    reason_code: int = ReasonCode.UNSPECIFIED


#: Everything ``interpret_anomaly`` may return. The union is closed on purpose:
#: adding a fourth action means adding a branch to the coordinator that executes
#: it, and the type checker is what makes those two edits happen together.
ActionSchema = Union[ReDelegate, AddTask, DropTask]

ACTION_TYPES = (ReDelegate, AddTask, DropTask)


# --------------------------------------------------------------------------
# Reading a note
#
# The second place open-ended reasoning happens, and the second closed union
# that keeps it from becoming an open-ended interface. A note is free text from
# another robot -- prompt material, and the only inbound path in this system
# where somebody else's sentences steer a decision. The reply is three types.
#
# Worth being explicit about why that matters here more than it does for
# ``interpret_anomaly``: ``src`` is self-asserted and there is no MAC on the
# radio, so any transmitter can claim to be robot 3. Constraining what a note
# can *do* to "bid as planned, bid worse, or do not bid" means the worst a
# hostile note achieves is making us decline work. It cannot make us go
# somewhere, take something on, or drop what we already hold -- those need an
# ActionSchema, which no inbound message can produce.
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class KeepBid:
    """The note changes nothing. Bid exactly as fitness already decided."""

    #: Why, for the log. Never parsed.
    reason: str = ""


@dataclass(frozen=True)
class ReviseBid:
    """The task is harder than the announcement implied. Bid, but worse.

    ``cost_delta`` is *relative* on purpose. A note is interpreted before, or
    alongside, the announcement it annotates -- so at the moment the model reads
    "the row is muddy" it does not know what this robot's own traverse would
    have cost, and an absolute number would be one it invented. A delta is a
    claim about the work, which is the only thing a note is evidence of.

    Negative deltas are allowed: a note can say the job is easier than it looks.
    The coordinator clamps the result into 0..COST_MAX either way.
    """

    cost_delta: int
    reason: str = ""


@dataclass(frozen=True)
class WithdrawBid:
    """Do not bid on this at all.

    Distinct from a large ReviseBid, and not merely a stronger version of one:
    a withdrawn bid is silence, while a bad bid is still an answer that can win
    an auction nobody better entered. "I could do this badly" and "I should not
    do this" are different sentences and the wire already distinguishes them.
    """

    reason: str = ""


#: Everything ``interpret_note`` may return. Closed for the same reason
#: ActionSchema is, and narrower: a note may only influence a bid we were
#: already going to make. It cannot start an auction, take on work, or drop it.
BidRevision = Union[KeepBid, ReviseBid, WithdrawBid]

REVISION_TYPES = (KeepBid, ReviseBid, WithdrawBid)


@dataclass
class NoteContext:
    """Everything the note interpretation gets to reason about.

    A snapshot, for the same reason AnomalyContext is one: the model call takes
    seconds, and the announce window it has to beat is measured in the same
    units.
    """

    #: The text, as another robot wrote it. Prompt material, never parsed.
    text: str
    #: The task it annotates.
    task_id: int
    #: Who sent it. Self-asserted and unauthenticated -- see the note above.
    src: int
    #: The task as we reconstructed it from the ANNOUNCE, if one has arrived.
    #: None when the note got here first, which is the intended ordering.
    task: Optional[Task] = None
    #: What our own fitness said before reading the note, so the model can be
    #: asked to adjust a number rather than invent one. None if we have not
    #: assessed it yet.
    cost: Optional[int] = None
    eta_s: Optional[float] = None
    feasible: Optional[bool] = None
    #: Seconds, from the coordinator's injected clock.
    at: float = 0.0


def validate_revision(revision: object) -> BidRevision:
    """Refuse anything that is not one of the three revisions.

    Same guard as ``validate_action`` and for the same reason, with one more
    behind it: this one stands between another robot's sentences and our own
    bidding, so the set of things a remote text can cause is worth being
    unable to widen by accident.
    """
    if not isinstance(revision, REVISION_TYPES):
        raise TypeError(
            f"interpret_note returned {type(revision).__name__}; expected one of "
            f"{', '.join(t.__name__ for t in REVISION_TYPES)}"
        )
    return revision


@dataclass
class AnomalyContext:
    """Everything the interpretation step gets to reason about.

    Assembled by the coordinator at the moment the mission node reports it
    cannot complete something. It is deliberately a snapshot rather than a live
    handle: the reasoning step may be slow, and a model reasoning over state
    that changes underneath it produces decisions about a robot that no longer
    exists.
    """

    #: The task that could not be completed. None for anomalies with no task
    #: attached.
    task: Optional[Task]
    #: Free text from the mission node describing what went wrong. Prompt
    #: material for the reasoning step; never parsed by the state machine.
    detail: str = ""
    #: ReasonCode the mission node attached, if any.
    reason_code: int = ReasonCode.UNSPECIFIED
    #: Live peers at the time of the anomaly, so re-delegation can be judged
    #: against who is actually out there rather than who once was.
    peers: "tuple[PeerRecord, ...]" = ()
    #: This robot's own state, for the "can I still do anything" question.
    battery: int = 100
    location: Optional[object] = None
    #: Seconds, from the coordinator's injected clock.
    at: float = 0.0


def validate_action(action: object) -> ActionSchema:
    """Refuse anything that is not one of the three actions.

    The guard exists because the real interpreter is a language
    model behind a parser, and a parser that half-succeeds is the failure mode
    worth being loud about. Raising here keeps a malformed interpretation from
    reaching the state machine as a ``None`` that quietly does nothing.
    """
    if not isinstance(action, ACTION_TYPES):
        raise TypeError(
            f"interpret_anomaly returned {type(action).__name__}; expected one of "
            f"{', '.join(t.__name__ for t in ACTION_TYPES)}"
        )
    return action
