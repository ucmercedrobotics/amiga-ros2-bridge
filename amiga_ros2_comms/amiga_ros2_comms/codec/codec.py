#!/usr/bin/env python3
"""One typed message <-> one packet of bytes. Nothing else.

``encode(msg) -> bytes`` and ``decode(bytes) -> msg``, over the vocabulary in
definitions.py. This layer knows field layout, quantization and the tag byte.

It knows nothing about *delivery*. No ACK logic, no retransmit, no dedup, no
fragmentation, no reassembly, no coordination policy. It does not interpret
``seq``, does not track what it has seen, and does not remember one call to the
next -- both functions are pure. Every message it produces fits in a single LoRa
payload by construction, which is what makes fragmentation someone else's
problem rather than an omission.

That stays true now that FREEFORM is built. A Freeform is one *fragment* of a
note, and one call still produces one packet; deciding where a note is cut and
putting the pieces back together belong to reliability/notes.py. All this file
gained is a field whose length is "the rest of the buffer".

Layout is declared once, in ``_LAYOUTS``, and both directions are driven from
it. That is deliberate: an encoder and a decoder written out by hand twice is an
invitation for the two to disagree about a field width in a way no round-trip
test on a single implementation would ever catch.

Pure Python, no ROS, no serial. Unit-testable with no radio in the loop.
"""

import math
import struct
from dataclasses import dataclass
from typing import Optional, Tuple, Type

from .definitions import (
    BATTERY_MAX,
    BYTE_ORDER,
    CAP_MASK_BITS,
    COST_MAX,
    DEFAULT_MAX_PAYLOAD_BYTES,
    ETA_MAX_S,
    ETA_RESOLUTION_S,
    FRAG_MAX,
    HEADER_BYTES,
    HEADER_FORMAT,
    NOTE_ID_MAX,
    PRIORITY_MAX,
    RESERVED_TYPES,
    SEQ_MAX,
    SRC_MAX,
    TARGET_WORD_MAX,
    TARGET_WORD_MIN,
    TASK_ID_MAX,
    MessageType,
    ReasonCode,
    Target,
    TargetKind,
)
from .messages import (
    BUILT_MESSAGES,
    Ack,
    Bid,
    Freeform,
    Grant,
    Heartbeat,
    Message,
    TaskAnnounce,
)


# --------------------------------------------------------------------------
# Errors
# --------------------------------------------------------------------------


class CodecError(ValueError):
    """A message could not be encoded or decoded.

    Carries a ``kind`` string for the same reason lora.framing.FrameError does:
    the layer above wants to count failure classes without string-matching an
    error message. Kinds are ``unknown_type``, ``reserved_type``, ``truncated``,
    ``trailing``, ``oversize``, ``range`` and ``value``.
    """

    kind = "value"

    def __init__(self, reason: str, kind: Optional[str] = None):
        super().__init__(reason)
        if kind is not None:
            self.kind = kind


class UnknownMessageType(CodecError):
    """The tag byte is not allocated at all. A peer newer than us, or garbage.

    Deliberately *not* a parent or child of ReservedMessageType: "we have never
    heard of this" and "we know exactly what this is and cannot handle it yet"
    call for different operator responses, so a caller must be able to tell them
    apart with isinstance.
    """

    kind = "unknown_type"


class ReservedMessageType(CodecError):
    """The tag is allocated but unimplemented -- currently only FREEFORM."""

    kind = "reserved_type"


class TruncatedMessage(CodecError):
    """The buffer ended before the message did. Also covers an empty buffer."""

    kind = "truncated"


class TrailingBytes(CodecError):
    """The buffer held a complete message and then kept going.

    Rejected rather than ignored. One message is one packet here, so trailing
    bytes mean the sender and receiver disagree about a field width or someone
    concatenated packets -- both of which are worth failing loudly over, and
    neither of which is repaired by silently dropping the tail.
    """

    kind = "trailing"


class PayloadTooLarge(CodecError):
    """The encoded message exceeds max_payload_bytes. Never fragmented."""

    kind = "oversize"


class FieldRangeError(CodecError):
    """A field value does not fit its wire field, or is not a valid value.

    Raised in both directions. On encode it stops a value being silently
    truncated into a different number; on decode it stops a structurally valid
    but semantically impossible packet (battery of 200%) becoming an object the
    rest of the stack trusts.
    """

    kind = "range"


# --------------------------------------------------------------------------
# Field table
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class _Field:
    """One wire field: where it lives, what fits in it, how it is scaled."""

    name: str
    #: Single struct code. Byte order comes from BYTE_ORDER, never from here.
    fmt: str
    #: Inclusive bounds in *engineering* units, before scaling.
    hi: int
    lo: int = 0
    #: Wire LSB size in engineering units. 1 means no quantization.
    scale: int = 1
    #: Coerced to this IntEnum on decode, if the value is a known member.
    enum: Optional[type] = None
    #: Carried as 0/1, surfaced as a Python bool.
    is_bool: bool = False


@dataclass(frozen=True)
class _Tail:
    """A trailing bytes field that runs to the end of the packet.

    Exactly one type has one (Freeform), and the restriction is structural
    rather than stylistic: a tail has no length of its own, so it can only be
    the last field, and only one field can be last. Everything before it is
    still the ordinary fixed table, which is why ``_PAYLOAD_FORMAT`` and
    ``_PAYLOAD_SIZE`` keep meaning what they meant -- for a tail-bearing type
    they describe its fixed part, and the packet is that plus whatever remains.
    """

    name: str


_HEADER_FIELDS: Tuple[_Field, ...] = (
    _Field("src", "B", SRC_MAX),
    _Field("seq", "H", SEQ_MAX),
)

# A place, as three flat fields. The kind is bounded to the enum's own range
# rather than to a full byte, unlike reason_code: an unrecognised reason from a
# newer peer is informational and can be logged as a number, but an
# unrecognised target kind means the coordinates are in a system we cannot
# interpret, and treating them as if we could is worse than refusing the packet.
_TARGET_FIELDS: Tuple[_Field, ...] = (
    _Field("target_kind", "B", int(max(TargetKind)), enum=TargetKind),
    _Field("target_a", "i", TARGET_WORD_MAX, lo=TARGET_WORD_MIN),
    _Field("target_b", "i", TARGET_WORD_MAX, lo=TARGET_WORD_MIN),
)

_LAYOUTS = {
    Heartbeat: (
        _Field("cap_mask", "H", (1 << CAP_MASK_BITS) - 1),
        *_TARGET_FIELDS,
        _Field("battery", "B", BATTERY_MAX),
        _Field("cur_task", "H", TASK_ID_MAX),
    ),
    TaskAnnounce: (
        _Field("task_id", "H", TASK_ID_MAX),
        # A *mask*, not an index: a task is a behaviour-tree subtree and a
        # subtree uses a set of actions. Same width as cap_mask, so the
        # requirement and the advertisement are directly comparable.
        _Field("req_cap_mask", "H", (1 << CAP_MASK_BITS) - 1),
        *_TARGET_FIELDS,
        _Field("priority", "B", PRIORITY_MAX),
        # Full byte range: an unrecognised reason from a newer peer is
        # informational, so it passes through as an int instead of failing.
        _Field("reason_code", "B", 0xFF, enum=ReasonCode),
    ),
    Bid: (
        _Field("task_id", "H", TASK_ID_MAX),
        _Field("eta_s", "B", ETA_MAX_S, scale=ETA_RESOLUTION_S),
        _Field("feasible", "B", 1, is_bool=True),
        _Field("cost", "B", COST_MAX),
    ),
    Grant: (
        _Field("task_id", "H", TASK_ID_MAX),
        _Field("winner_id", "B", SRC_MAX),
    ),
    Ack: (
        _Field("ack_src", "B", SRC_MAX),
        _Field("ack_seq", "H", SEQ_MAX),
    ),
    Freeform: (
        _Field("task_id", "H", TASK_ID_MAX),
        _Field("note_id", "B", NOTE_ID_MAX),
        # Bounded to the byte, not to MAX_NOTE_FRAGMENTS. How long a note this
        # fleet is willing to *send* is a policy the splitter enforces; a peer
        # configured to send longer ones is telling us something true, and
        # failing the decode would lose the fragments we did receive along with
        # our ability to say why. The reassembler refuses the note instead,
        # where the count can be reported.
        _Field("frag_index", "B", FRAG_MAX),
        _Field("frag_count", "B", FRAG_MAX),
    ),
}

#: The variable-length tail of each type that has one. Only Freeform does.
_TAILS = {Freeform: _Tail("text")}

assert set(_LAYOUTS) == set(BUILT_MESSAGES), "layout table and message list disagree"
assert set(_TAILS) <= set(_LAYOUTS), "a tail was declared for a type with no layout"


def _payload_format(fields: Tuple[_Field, ...]) -> str:
    return BYTE_ORDER + "".join(f.fmt for f in fields)


_PAYLOAD_FORMAT = {cls: _payload_format(fields) for cls, fields in _LAYOUTS.items()}
_PAYLOAD_SIZE = {cls: struct.calcsize(fmt) for cls, fmt in _PAYLOAD_FORMAT.items()}

#: Tag byte -> message class, for the built types only.
_BY_TAG = {cls.TAG: cls for cls in _LAYOUTS}

#: The message classes that name a place. Derived from the layout table rather
#: than listed, so a new message type that carries a target gets the validation
#: and the accessors without a second edit anyone could forget.
_TARGETED = frozenset(
    cls
    for cls, fields in _LAYOUTS.items()
    if all(f.name in {ff.name for ff in fields} for f in _TARGET_FIELDS)
)

assert len(_BY_TAG) == len(_LAYOUTS), "two message classes share a tag"

#: Encoded size of each built type, header included. For a tail-bearing type
#: this is the size with an empty tail -- a floor rather than the size.
MESSAGE_SIZES = {cls: HEADER_BYTES + size for cls, size in _PAYLOAD_SIZE.items()}

#: Largest message this codec can produce *from a type whose size its class
#: determines*. Freeform is excluded, because for it there is no such number:
#: the answer is whatever max_payload_bytes the caller passed to encode.
#:
#: Excluded rather than folded in with some assumed tail length, because this
#: is what reliability/node.py budgets a request-plus-ACK round trip against.
#: A round-trip budget built on a guess about how long somebody's note was
#: would be a worse number than the exact one it replaced.
MAX_MESSAGE_BYTES = max(
    size for cls, size in MESSAGE_SIZES.items() if cls not in _TAILS
)

#: Header cost of a note fragment: the common header plus Freeform's fixed
#: fields. Subtract from a payload budget to get the text bytes per fragment --
#: which is what reliability/notes.py does, and the only reason this is public.
NOTE_HEADER_BYTES = MESSAGE_SIZES[Freeform]


# --------------------------------------------------------------------------
# Field conversion
# --------------------------------------------------------------------------


def _to_raw(field: _Field, value, where: str) -> int:
    """Engineering value -> the integer that goes on the wire."""
    if field.is_bool:
        if isinstance(value, bool) or value in (0, 1):
            return int(bool(value))
        raise FieldRangeError(f"{where}.{field.name}: expected a bool, got {value!r}")

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FieldRangeError(f"{where}.{field.name}: expected a number, got {value!r}")
    if isinstance(value, float) and not math.isfinite(value):
        raise FieldRangeError(f"{where}.{field.name}: {value!r} is not finite")

    if not field.lo <= value <= field.hi:
        raise FieldRangeError(
            f"{where}.{field.name}={value} outside {field.lo}..{field.hi}"
        )

    if field.scale == 1:
        # Unscaled fields must be exact. Quietly flooring 3.7 into a 3 that
        # round-trips as 3 is the kind of thing that is only ever noticed much
        # later and somewhere else.
        if isinstance(value, float) and not value.is_integer():
            raise FieldRangeError(
                f"{where}.{field.name}={value} is not a whole number "
                f"(this field has 1-unit resolution)"
            )
        return int(value)

    # Half-up rounding onto the quantization lattice. Integer arithmetic where
    # we can, so the common all-int path has no float rounding in it at all.
    if isinstance(value, int):
        return (value * 2 + field.scale) // (2 * field.scale)
    return int(math.floor((value + field.scale / 2) / field.scale))


def _from_raw(field: _Field, raw: int, where: str):
    """The integer off the wire -> engineering value."""
    if field.is_bool:
        if raw not in (0, 1):
            raise FieldRangeError(f"{where}.{field.name}: bool byte was {raw}")
        return bool(raw)

    value = raw * field.scale
    if not field.lo <= value <= field.hi:
        raise FieldRangeError(
            f"{where}.{field.name}={value} outside {field.lo}..{field.hi}"
        )

    if field.enum is not None:
        try:
            return field.enum(value)
        except ValueError:
            # A value we do not have a name for. Structurally fine, so hand it
            # back as a plain int rather than rejecting the whole message: a
            # newer peer naming a reason we lack a word for is still telling us
            # what it wants. IntEnum compares equal to its int value, so this
            # costs nothing at the call site.
            return value
    return value


# --------------------------------------------------------------------------
# Targets
#
# The wire carries a place as three loose integers because the layout table is
# flat. These two functions are the only place that is true: above them a place
# is a Target, below them it is struct fields, and nothing in between handles
# the words by hand.
# --------------------------------------------------------------------------


def target_of(msg: Message) -> Target:
    """The place a message names.

    Raises AttributeError for a message type that names no place, which is
    correct -- asking a Bid where it is has no answer to return.
    """
    return Target(
        kind=TargetKind(int(msg.target_kind)),
        a=int(msg.target_a),
        b=int(msg.target_b),
    )


def target_fields(target: Target) -> dict:
    """``Target`` -> the keyword arguments a targeted message constructor wants.

    >>> TaskAnnounce(src=1, seq=2, task_id=7, req_cap_mask=1,
    ...              **target_fields(Target.tree(60)),
    ...              priority=100, reason_code=1)  # doctest: +ELLIPSIS
    TaskAnnounce(...)
    """
    return {
        "target_kind": int(target.kind),
        "target_a": int(target.a),
        "target_b": int(target.b),
    }


# --------------------------------------------------------------------------
# Public API
# --------------------------------------------------------------------------


def encode(msg: Message, max_payload_bytes: int = DEFAULT_MAX_PAYLOAD_BYTES) -> bytes:
    """Pack one typed message into one packet.

    Raises UnknownMessageType for anything that is not one of the six built
    types, ReservedMessageType for a tag allocated but not yet built (none
    currently are), FieldRangeError for a value that does not fit its field,
    and PayloadTooLarge if the result would exceed ``max_payload_bytes``.

    A Freeform is one *fragment*, and PayloadTooLarge on one means the caller
    sized its fragments against a different budget than it is encoding with --
    still a design error and still never repaired by splitting here.
    """
    tag = getattr(type(msg), "TAG", None)
    if tag is None:
        raise CodecError(
            f"{type(msg).__name__} is not a codec message (no TAG)", "value"
        )

    # Before the layout lookup, so a reserved tag reports as reserved rather
    # than as an unknown type that merely happens to have no layout.
    if tag in RESERVED_TYPES:
        raise ReservedMessageType(
            f"{MessageType(tag).name} (0x{int(tag):02x}) is allocated but not "
            f"implemented"
        )

    fields = _LAYOUTS.get(type(msg))
    if fields is None:
        raise UnknownMessageType(f"no wire layout for {type(msg).__name__}")

    where = type(msg).__name__
    header = struct.pack(
        HEADER_FORMAT,
        int(tag),
        *(_to_raw(f, getattr(msg, f.name), where) for f in _HEADER_FIELDS),
    )
    payload = struct.pack(
        _PAYLOAD_FORMAT[type(msg)],
        *(_to_raw(f, getattr(msg, f.name), where) for f in fields),
    )

    tail = _TAILS.get(type(msg))
    if tail is not None:
        value = getattr(msg, tail.name)
        if not isinstance(value, (bytes, bytearray, memoryview)):
            # str is refused rather than encoded here. Where a note is split is
            # decided by a byte budget, so the caller that did the splitting is
            # the one holding bytes; a str arriving at this point means someone
            # skipped the splitter and is about to send a fragment that is not
            # a fragment of anything.
            raise FieldRangeError(
                f"{where}.{tail.name}: expected bytes, got " f"{type(value).__name__}"
            )
        payload += bytes(value)

    packet = header + payload
    if len(packet) > max_payload_bytes:
        # Never split. Fragmentation is the reliability layer's job, and a
        # codec that quietly returned two packets would be inventing one.
        raise PayloadTooLarge(
            f"{where} encodes to {len(packet)} bytes, over "
            f"max_payload_bytes={max_payload_bytes}"
        )
    return packet


def decode(data: bytes) -> Message:
    """Unpack one packet into a typed message.

    Never reads past the end of ``data`` and never raises anything but
    CodecError for bad input. Distinguishes, by exception type: an unallocated
    tag (UnknownMessageType), a tag allocated but unbuilt (ReservedMessageType),
    a buffer that ends early (TruncatedMessage), a buffer with extra bytes on
    the end (TrailingBytes), and a value that cannot mean what it says
    (FieldRangeError).

    "Extra bytes on the end" is only an error for the fixed-size types. For
    Freeform the extra bytes are the message.
    """
    # Checked by type rather than by trying bytes(): bytes(5) does not fail, it
    # quietly hands back five zero bytes, which would decode as a plausible
    # message rather than as the mistake it is.
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise CodecError(
            f"expected a bytes-like object, got {type(data).__name__}", "value"
        )
    data = bytes(data)

    if len(data) < HEADER_BYTES:
        raise TruncatedMessage(
            f"buffer of {len(data)} bytes is shorter than the "
            f"{HEADER_BYTES}-byte header"
        )

    tag, src, seq = struct.unpack_from(HEADER_FORMAT, data, 0)

    # Reserved is checked before the tag table and before any length
    # validation, so an allocated-but-unbuilt tag reports as reserved even when
    # the frame is also malformed. "We know what this is and cannot handle it"
    # and "we have never heard of this" call for different operator responses.
    if tag in RESERVED_TYPES:
        raise ReservedMessageType(
            f"{MessageType(tag).name} (0x{tag:02x}) is allocated but not "
            f"implemented"
        )

    cls: Optional[Type[Message]] = _BY_TAG.get(tag)
    if cls is None:
        raise UnknownMessageType(f"tag 0x{tag:02x} is not an allocated message type")

    tail = _TAILS.get(cls)
    expected = HEADER_BYTES + _PAYLOAD_SIZE[cls]
    if len(data) < expected:
        raise TruncatedMessage(
            f"{cls.__name__} needs {expected} bytes, buffer has {len(data)}"
        )
    if len(data) > expected and tail is None:
        raise TrailingBytes(
            f"{cls.__name__} is {expected} bytes, buffer has {len(data)} "
            f"({len(data) - expected} trailing)"
        )

    fields = _LAYOUTS[cls]
    raw = struct.unpack_from(_PAYLOAD_FORMAT[cls], data, HEADER_BYTES)
    values = {f.name: _from_raw(f, r, cls.__name__) for f, r in zip(fields, raw)}
    if tail is not None:
        # Whatever is left, including nothing. An empty tail is a legal packet
        # rather than a truncated one: a note whose text divides evenly can end
        # with a fragment carrying no bytes, and refusing that here would make
        # the splitter's arithmetic a special case instead of a division.
        values[tail.name] = data[expected:]

    if cls in _TARGETED:
        # Each target word is in range on its own, but the three together still
        # have to mean something: a TREE target with a longitude word set, or a
        # NONE target carrying coordinates, is a sender that disagrees with us
        # about the layout. Target's own constructor is the rule, so the check
        # is here rather than restated.
        try:
            Target(
                kind=values["target_kind"],
                a=values["target_a"],
                b=values["target_b"],
            )
        except ValueError as exc:
            raise FieldRangeError(f"{cls.__name__}: {exc}") from None

    return cls(src=src, seq=seq, **values)
