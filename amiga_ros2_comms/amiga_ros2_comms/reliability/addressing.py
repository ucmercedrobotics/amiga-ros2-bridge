#!/usr/bin/env python3
"""Which messages are addressed to one robot, and which are shouted at everyone.

This is the whole basis for the layer's central rule -- *reliability follows
addressing mode* -- so it lives in one table rather than being rediscovered at
each call site:

    unicast   -> reliable     (ACK + bounded retransmit)
    broadcast -> best-effort  (send once; the app repeats if it cares)

The asymmetry is forced, not chosen. A broadcast goes to a set of receivers
whose membership nobody knows, so there is no set of ACKs whose absence means
anything: waiting for them would stall on a robot that was switched off, and
retransmitting until they arrive would never terminate. Repetition for those
belongs to the coordinator, which knows *why* it is repeating (an auction still
open, a hazard still live). All this layer owes them is receiver-side dedup.

The wire header carries no ``dst`` field, so the addressee is a property of the
message body: GRANT names its ``winner_id``. That is the only interpretation of
message content in this layer, and it is confined to this file on purpose --
"who is this for" is addressing, not meaning. What a GRANT obliges the winner to
do is the coordinator's business.

Pure data and one lookup. No ROS, no serial, no I/O.
"""

from typing import Optional, Type

from ..codec import Ack, Grant, Message

# The two addressing modes. Strings rather than an enum because they surface in
# log lines and stats keys, and a str is what both of those want anyway.
UNICAST = "unicast"
BROADCAST = "broadcast"

MODES = (UNICAST, BROADCAST)

#: Unicast message class -> the field naming its addressee.
#:
#: GRANT is the only entry today. COMPLETE and RELEASE join it when they are
#: built, which is the point of a table: a new unicast type is one line here,
#: and forgetting the line makes it best-effort broadcast rather than silently
#: half-reliable.
_ADDRESSEE_FIELD = {
    Grant: "winner_id",
}


def addressing_of(msg: Message) -> str:
    """UNICAST or BROADCAST for ``msg``.

    Anything not named in the table is broadcast. That default is deliberate:
    an unrecognised type treated as broadcast is merely un-ACKed, whereas one
    treated as unicast would sit on a retransmit timer waiting for an ACK that
    nobody was ever going to send.
    """
    return UNICAST if type(msg) in _ADDRESSEE_FIELD else BROADCAST


def is_unicast(msg: Message) -> bool:
    return addressing_of(msg) == UNICAST


def addressee_of(msg: Message) -> Optional[int]:
    """The robot ID a unicast message is for, or None if it is a broadcast."""
    field = _ADDRESSEE_FIELD.get(type(msg))
    return None if field is None else int(getattr(msg, field))


def is_reliable(msg: Message) -> bool:
    """Whether this layer owes ``msg`` an ACK and retransmits.

    Unicast and not an ACK. ACKs are excluded structurally rather than by
    convention: acknowledging an acknowledgement is how you build a system that
    never stops talking, and the exclusion is worth being unable to forget.
    """
    return not isinstance(msg, Ack) and is_unicast(msg)


def unicast_types() -> "tuple[Type[Message], ...]":
    """Every message class this layer will send reliably. Used by tests."""
    return tuple(_ADDRESSEE_FIELD)
