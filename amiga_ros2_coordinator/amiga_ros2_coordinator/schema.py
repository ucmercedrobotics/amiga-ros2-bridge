#!/usr/bin/env python3
"""The constrained vocabulary an anomaly interpretation is allowed to speak.

``interpret_anomaly`` is one of the two places in this system where open-ended
reasoning happens, and this file is what keeps it from becoming an open-ended
*interface*. It returns one of exactly three typed actions:

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
