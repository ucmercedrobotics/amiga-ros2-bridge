#!/usr/bin/env python3
"""Typed coordination messages -- the six built wire types.

One dataclass per tag. These are plain value objects: they hold fields, they
compare by value (which is what makes the round-trip tests meaningful), and they
carry no behaviour. Packing them into bytes is codec.py's job.

Every message begins with the common header -- ``src`` and ``seq`` -- because
(src, seq) is the globally unique message ID the reliability layer will need for
dedup and ACKs. Carrying them is all this layer does with them.
"""

from dataclasses import dataclass
from typing import ClassVar

from .definitions import MessageType


@dataclass
class Message:
    """Common header shared by every message type.

    ``src`` is the sender's robot ID; ``seq`` is that sender's monotonic
    sequence number, which wraps at 16 bits. The codec neither generates,
    validates the monotonicity of, nor tracks either one.
    """

    src: int
    seq: int

    #: The tag byte this class occupies. Set by every concrete subclass.
    TAG: ClassVar[MessageType]


@dataclass
class Heartbeat(Message):
    """I exist, here is what I can do, where I am, and what I am doing.

    Sent periodically by every robot. This is what populates the fleet's view of
    who is available to bid.
    """

    TAG: ClassVar[MessageType] = MessageType.HEARTBEAT

    #: 16-bit mask of Capability bit indices. Build with definitions.cap_mask().
    cap_mask: int
    grid_row: int
    grid_col: int
    #: Whole percent, 0..100.
    battery: int
    #: Task currently being executed, or TASK_NONE (0) when idle.
    cur_task: int


@dataclass
class TaskAnnounce(Message):
    """There is work at this cell; whoever can do it, bid.

    Broadcast by whichever robot or operator station is originating the task.
    """

    TAG: ClassVar[MessageType] = MessageType.TASK_ANNOUNCE

    task_id: int
    #: A single Capability *index* (not a mask) the bidder must advertise.
    req_capability: int
    grid_row: int
    grid_col: int
    #: Unitless, higher is more urgent.
    priority: int
    #: ReasonCode -- why this task exists. Diagnostics, not dispatch logic.
    reason_code: int


@dataclass
class Bid(Message):
    """My offer on an announced task.

    ``feasible`` false is a meaningful answer, not a non-answer: it tells the
    announcer this robot heard the announcement and is ruling itself out, which
    is what lets an auction close early instead of waiting out a timeout.
    """

    TAG: ClassVar[MessageType] = MessageType.BID

    task_id: int
    #: Estimated seconds to arrive, 0..1020, quantized to 4 s on the wire.
    eta_s: int
    feasible: bool
    #: Unitless, lower is better. Only comparable between bids on one task.
    cost: int


@dataclass
class Grant(Message):
    """Task awarded to ``winner_id``.

    The only *addressed* message in the vocabulary, which is why it is also the
    only one the reliability layer sends reliably: losing a GRANT either
    double-assigns a task or orphans it, so the winner must confirm it. There
    is no ``dst`` in the header -- ``winner_id`` is the address, and
    reliability/addressing.py is the one place that reads it as such.

    Addressed is not the same as private. It goes out over the same broadcast
    radio as everything else, so losing bidders hear it too and learn the
    auction closed; they simply do not ACK it. Only the winner does.
    """

    TAG: ClassVar[MessageType] = MessageType.GRANT

    task_id: int
    #: The robot this task is awarded to, and the only one that will ACK.
    winner_id: int


@dataclass
class Ack(Message):
    """I received message (ack_src, ack_seq).

    The codec packs this and nothing more. Deciding when to send one, what to do
    when one fails to arrive, and how long to keep a message for retransmit are
    all the reliability layer's problems -- this type exists now only so the
    vocabulary is complete and the tag is allocated.
    """

    TAG: ClassVar[MessageType] = MessageType.ACK

    #: The ``src`` of the message being acknowledged.
    ack_src: int
    #: The ``seq`` of the message being acknowledged.
    ack_seq: int


@dataclass
class Hazard(Message):
    """Something is in the way at this cell, for about this long."""

    TAG: ClassVar[MessageType] = MessageType.HAZARD

    #: HazardClass.
    hazard_class: int
    grid_row: int
    grid_col: int
    #: Radius in grid cells around (grid_row, grid_col).
    radius: int
    #: Whole percent, 0..100.
    confidence: int
    #: Seconds this report should be believed for, 0..65535.
    ttl_s: int


#: Every built (non-reserved) message class. codec.py builds its tag table from
#: this, so adding a type here is the only edit a new type needs.
BUILT_MESSAGES = (Heartbeat, TaskAnnounce, Bid, Grant, Ack, Hazard)
