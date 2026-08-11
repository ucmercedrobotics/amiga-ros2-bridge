#!/usr/bin/env python3
"""Typed coordination messages -- the six built wire types.

One dataclass per tag. These are plain value objects: they hold fields, they
compare by value (which is what makes the round-trip tests meaningful), and they
carry no behaviour. Packing them into bytes is codec.py's job.

Every message begins with the common header -- ``src`` and ``seq`` -- because
(src, seq) is the globally unique message ID the reliability layer will need for
dedup and ACKs. Carrying them is all this layer does with them.

Where a message names a place it does so as three flat fields --
``target_kind``, ``target_a``, ``target_b`` -- rather than a nested ``Target``.
The codec's layout table is a flat list of struct codes per type, and keeping
the wire form flat is what lets encode and decode stay driven from that one
table. ``Target.from_fields`` / ``as_fields`` in codec.py convert at the edge,
so nothing above this layer handles the loose integers.
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

    #: 16-bit mask of Capability bit indices -- the behaviour-tree action types
    #: this robot's mission schema permits. Build with definitions.cap_mask().
    cap_mask: int
    #: Where we are. Normally TargetKind.GPS; NONE when there is no fix, which
    #: is a different fact from being at the origin and now says so.
    target_kind: int
    target_a: int
    target_b: int
    #: Whole percent, 0..100.
    battery: int
    #: Task currently being executed, or TASK_NONE (0) when idle.
    cur_task: int


@dataclass
class TaskAnnounce(Message):
    """There is work here; whoever can do all of it, bid.

    Broadcast by whichever robot or operator station is originating the task.
    """

    TAG: ClassVar[MessageType] = MessageType.TASK_ANNOUNCE

    task_id: int
    #: Mask of every Capability the task's subtree uses -- a *set*, because a
    #: task is a subtree and sampling a tree needs both the navigation action
    #: and the arm one. A bidder must advertise all of them.
    req_cap_mask: int
    #: Where the work is, as the behaviour tree names it. TargetKind.NONE means
    #: the work has no place of its own, which makes it undelegable -- the
    #: originator is expected not to announce those.
    target_kind: int
    target_a: int
    target_b: int
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
class Freeform(Message):
    """One fragment of a note: free text about a task somebody announced.

    A note never *replaces* a structured message. It annotates one, and the
    TASK_ANNOUNCE it accompanies still carries the whole machine-readable
    requirement -- capabilities, place, priority. That is the point: lose every
    fragment of a note and the auction is bit-for-bit what it would have been,
    so partial delivery costs a decision some context rather than costing the
    fleet a task. Text that could invalidate an auction by going missing would
    be a worse design than not carrying text at all.

    The only variable-length type, and the only one where several packets make
    up one logical message. Both facts are confined to this type: splitting and
    reassembly live in reliability/notes.py, and every other message in the
    vocabulary is still exactly one packet with a size known from its class.
    """

    TAG: ClassVar[MessageType] = MessageType.FREEFORM

    #: The announced task this text is about. The join key, and the reason a
    #: note is an annotation rather than a message in its own right -- there is
    #: no such thing here as text about nothing in particular.
    task_id: int
    #: Which note this is, so two notes about one task from one sender cannot
    #: interleave their fragments into a sentence neither of them said.
    note_id: int
    #: 0-based position in the note. Fragments may arrive in any order.
    frag_index: int
    #: How many fragments the whole note is. Carried in *every* fragment so a
    #: receiver knows what it is waiting for without having to have seen the
    #: first one -- on a lossy broadcast link, "the one that tells you the
    #: length" is exactly as likely to be lost as any other.
    frag_count: int
    #: This fragment's slice of the UTF-8 encoding of the note. Bytes, not str:
    #: a split lands wherever the byte budget lands, which is frequently in the
    #: middle of a multi-byte character, so only the reassembled whole is
    #: decodable text.
    text: bytes = b""


#: Every built (non-reserved) message class. codec.py builds its tag table from
#: this, so adding a type here is the only edit a new type needs.
BUILT_MESSAGES = (Heartbeat, TaskAnnounce, Bid, Grant, Ack, Freeform)
