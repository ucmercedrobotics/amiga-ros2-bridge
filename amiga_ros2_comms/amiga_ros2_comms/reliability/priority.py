#!/usr/bin/env python3
"""Which messages go first when the radio is busy.

A sibling of addressing.py, and deliberately not part of it: that file answers
"who is this for", this one answers "which of these goes next". They happen to
agree today -- the reliable unicast type is also the urgent one -- but they are
different questions and a change to one is not a change to the other.

**Two classes, not a scale.**

    URGENT   ACK, GRANT
    BULK     everything else

The line is drawn at task ownership. An ACK is what resolves a sender's
retransmit campaign, and a GRANT is the only message this layer sends reliably;
delaying either past ``retransmit_timeout_sec`` turns a delivered task into a
failed one, and the coordinator responds to that by handing the work back to
itself. Everything else is either repeated by the coordinator on its own
schedule (HEARTBEAT, TASK_ANNOUNCE) or is one packet whose loss costs one bid.

Six graded levels would let us order BID against HEARTBEAT, but those two
almost never contend for the same queue slot, and every boundary we draw is one
we would have to defend. Two is what the problem actually has.

**Why this matters at all.** The radio is half-duplex and the outbound path is
FIFO, so a robot part-way through a long transmission cannot get an ACK onto the
air until the queue drains. At SF10 the dwell budget allows a 24-byte payload,
so anything that does not fit one packet becomes a train of them -- and a train
long enough to outlast a retransmit timeout makes coordination fail *because
robots were talking*. This table is what stops that.

Pure data and one lookup. No ROS, no serial, no I/O.
"""

from typing import Type

from ..codec import Ack, Grant, Message

#: Goes first. The messages whose delay breaks task ownership.
URGENT = 0
#: Everything else. Repeated by the coordinator, or cheap to lose.
BULK = 1

CLASSES = (URGENT, BULK)

#: Human-readable names, for log lines and stats keys.
CLASS_NAME = {URGENT: "urgent", BULK: "bulk"}

#: The message classes that jump the queue. A set rather than a full mapping
#: because there are only two classes: membership *is* the answer.
#:
#: COMPLETE and RELEASE join it when they are built, for the same reason GRANT
#: is here -- they are the other unicast types, and a reliable send whose ACK
#: arrives late is a reliable send that failed.
_URGENT_TYPES = frozenset({Ack, Grant})


def priority_of(msg: Message) -> int:
    """URGENT or BULK for ``msg``.

    Anything not named in the table is BULK. That default is the safe one *here*
    -- unlike the .msg field, whose unset value reads as URGENT. The two point
    opposite ways on purpose: inside this layer we know the whole type set, so
    an unclassified message is by definition not one of the two that hold
    ownership consistent, and letting it jump the queue would dilute the only
    guarantee this table exists to provide. A publisher outside this layer has
    made no such claim, and starving it silently would be worse.
    """
    return URGENT if type(msg) in _URGENT_TYPES else BULK


def is_urgent(msg: Message) -> bool:
    return priority_of(msg) == URGENT


def urgent_types() -> "tuple[Type[Message], ...]":
    """Every message class that jumps the queue. Used by tests."""
    return tuple(_URGENT_TYPES)
