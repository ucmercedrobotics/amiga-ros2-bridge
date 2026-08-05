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
    HEADER_BYTES,
    HEADER_FORMAT,
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
}

assert set(_LAYOUTS) == set(BUILT_MESSAGES), "layout table and message list disagree"


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

#: Encoded size of each built type, header included.
MESSAGE_SIZES = {cls: HEADER_BYTES + size for cls, size in _PAYLOAD_SIZE.items()}

#: Largest message this codec can produce. The whole vocabulary is fixed-size
#: per type, so this is an exact bound, not an estimate.
MAX_MESSAGE_BYTES = max(MESSAGE_SIZES.values())


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

    Raises ReservedMessageType for FREEFORM, UnknownMessageType for anything
    that is not one of the six built types, FieldRangeError for a value that
    does not fit its field, and PayloadTooLarge if the result would exceed
    ``max_payload_bytes``.
    """
    tag = getattr(type(msg), "TAG", None)
    if tag is None:
        raise CodecError(
            f"{type(msg).__name__} is not a codec message (no TAG)", "value"
        )

    # Before the layout lookup, so FREEFORM reports as reserved rather than as
    # an unknown type that merely happens to have no layout.
    if tag in RESERVED_TYPES:
        raise ReservedMessageType(
            f"{MessageType(tag).name} (0x{int(tag):02x}) is reserved and not "
            f"implemented: it needs fragmentation from the reliability layer"
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
    tag (UnknownMessageType), the reserved FREEFORM tag (ReservedMessageType),
    a buffer that ends early (TruncatedMessage), a buffer with extra bytes on
    the end (TrailingBytes), and a value that cannot mean what it says
    (FieldRangeError).
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

    # Reserved is checked first so 0x07 can never be reported as unknown.
    if tag in RESERVED_TYPES:
        raise ReservedMessageType(
            f"{MessageType(tag).name} (0x{tag:02x}) is reserved and not "
            f"implemented: it needs fragmentation from the reliability layer"
        )

    cls: Optional[Type[Message]] = _BY_TAG.get(tag)
    if cls is None:
        raise UnknownMessageType(f"tag 0x{tag:02x} is not an allocated message type")

    expected = HEADER_BYTES + _PAYLOAD_SIZE[cls]
    if len(data) < expected:
        raise TruncatedMessage(
            f"{cls.__name__} needs {expected} bytes, buffer has {len(data)}"
        )
    if len(data) > expected:
        raise TrailingBytes(
            f"{cls.__name__} is {expected} bytes, buffer has {len(data)} "
            f"({len(data) - expected} trailing)"
        )

    fields = _LAYOUTS[cls]
    raw = struct.unpack_from(_PAYLOAD_FORMAT[cls], data, HEADER_BYTES)
    values = {f.name: _from_raw(f, r, cls.__name__) for f, r in zip(fields, raw)}

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
