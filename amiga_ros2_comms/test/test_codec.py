#!/usr/bin/env python3
"""Unit tests for the coordination codec. No ROS, no serial port, no radio.

The five acceptance groups from the brief -- round-trip, size, reserved tag,
unknown tag, malformed input -- plus the quantization bound that the round-trip
test cannot express on its own.

Test instances are generated *from the codec's own field table*, so a new
message type or a widened field is covered the moment it is declared and cannot
be added without its boundaries being exercised.
"""

import os
import random
import struct
import sys
from xml.etree import ElementTree

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import (  # noqa: E402
    DEFAULT_MAX_PAYLOAD_BYTES,
    ETA_MAX_S,
    ETA_RESOLUTION_S,
    GPS_SCALE,
    HEADER_BYTES,
    INDEX_MAX,
    MAX_MESSAGE_BYTES,
    MESSAGE_SIZES,
    NOTE_HEADER_BYTES,
    RESERVED_TYPES,
    Ack,
    Bid,
    Capability,
    CodecError,
    FieldRangeError,
    Freeform,
    Grant,
    Heartbeat,
    MessageType,
    PayloadTooLarge,
    ReasonCode,
    ReservedMessageType,
    Target,
    TargetKind,
    TaskAnnounce,
    TrailingBytes,
    TruncatedMessage,
    UnknownMessageType,
    XML_ELEMENT,
    cap_mask,
    capabilities_in,
    decode,
    encode,
    has_capabilities,
    has_capability,
    target_fields,
    target_of,
)
from amiga_ros2_comms.codec.codec import (  # noqa: E402
    _HEADER_FIELDS,
    _LAYOUTS,
    _TAILS,
    _TARGETED,
)

BUILT = tuple(_LAYOUTS)

#: The types whose encoded size is a property of the class. Everything about
#: "one message is one packet of a known length" is stated over these; Freeform
#: is deliberately excluded because it carries a variable tail, and folding it
#: in would weaken the claim for the five types that really do hold it.
FIXED = tuple(cls for cls in BUILT if cls not in _TAILS)

#: The names of the three fields that together mean one place. They are drawn
#: as a unit rather than field by field, because two of the three constrain the
#: third -- an AISLE target has no longitude -- and independent draws would
#: generate triples no sender could produce and the decoder rightly refuses.
TARGET_FIELD_NAMES = ("target_kind", "target_a", "target_b")

#: Every legal target worth encoding: both index extremes, the poles and the
#: antimeridian, a real orchard fix from examples/quad.xml, and the placeless
#: target that SampleLeaf-only work has.
LEGAL_TARGETS = (
    Target.none(),
    Target.tree(0),
    Target.tree(1),
    Target.tree(60),
    Target.tree(INDEX_MAX),
    Target.aisle(0),
    Target.aisle(INDEX_MAX),
    Target.gps(0.0, 0.0),
    Target.gps(90.0, 180.0),
    Target.gps(-90.0, -180.0),
    Target.gps(37.366449, -120.423065),
)


# --------------------------------------------------------------------------
# Instance generation, driven off the codec's field table
# --------------------------------------------------------------------------


def _lattice_values(field):
    """Representable boundary values for one field: lowest and highest."""
    if field.is_bool:
        return [False, True]
    return [field.lo, field.hi]


def _random_value(field, rng):
    """A value that sits exactly on the field's quantization lattice."""
    if field.is_bool:
        return rng.choice([False, True])
    return rng.randrange(field.lo, field.hi + 1, field.scale)


def _all_fields(cls):
    return _HEADER_FIELDS + _LAYOUTS[cls]


def _scalar_fields(cls):
    """Everything except the target triple, which is generated as a unit."""
    return tuple(f for f in _all_fields(cls) if f.name not in TARGET_FIELD_NAMES)


def _with_target(cls, values, target):
    if cls in _TARGETED:
        values = {**values, **target_fields(target)}
    return cls(**values)


def _extreme_instances(cls):
    """All-minimum and all-maximum instances of ``cls``."""
    fields = _scalar_fields(cls)
    return [
        _with_target(
            cls,
            {f.name: _lattice_values(f)[i] for f in fields},
            LEGAL_TARGETS[0] if i == 0 else LEGAL_TARGETS[-1],
        )
        for i in (0, 1)
    ]


def _boundary_instances(cls):
    """Every field driven to each of its bounds, with the rest mid-range.

    Sweeping one field at a time is what catches a field packed at the wrong
    offset: an all-max instance can look correct even if two same-width fields
    are transposed. The target sweeps by whole target instead, for the same
    reason at the level of a group of fields.
    """
    fields = _scalar_fields(cls)
    rng = random.Random(f"boundary-{cls.__name__}")
    base = {f.name: _random_value(f, rng) for f in fields}
    out = []
    for field in fields:
        for value in _lattice_values(field):
            out.append(
                _with_target(
                    cls, {**base, field.name: value}, rng.choice(LEGAL_TARGETS)
                )
            )
    if cls in _TARGETED:
        out.extend(_with_target(cls, base, target) for target in LEGAL_TARGETS)
    return out


def _random_instances(cls, count=200):
    rng = random.Random(f"random-{cls.__name__}")
    fields = _scalar_fields(cls)
    return [
        _with_target(
            cls,
            {f.name: _random_value(f, rng) for f in fields},
            rng.choice(LEGAL_TARGETS),
        )
        for _ in range(count)
    ]


def _every_instance(cls):
    return _extreme_instances(cls) + _boundary_instances(cls) + _random_instances(cls)


# --------------------------------------------------------------------------
# Group 1: round-trip
# --------------------------------------------------------------------------


@pytest.mark.parametrize("cls", BUILT, ids=lambda c: c.__name__)
def test_round_trip_across_the_full_range_of_every_field(cls):
    for msg in _every_instance(cls):
        assert decode(encode(msg)) == msg, f"round trip failed for {msg}"


@pytest.mark.parametrize("cls", BUILT, ids=lambda c: c.__name__)
def test_round_trip_preserves_the_message_type(cls):
    for msg in _extreme_instances(cls):
        assert type(decode(encode(msg))) is cls


def test_round_trip_of_hand_written_messages():
    # The generated cases prove self-consistency; these prove the fields mean
    # what the vocabulary says they mean, written out the way a caller would.
    # The task announced here is the one from examples/sample_leafs.xml: go to
    # tree 60 and sample its leaves.
    messages = [
        Heartbeat(
            src=3,
            seq=1024,
            cap_mask=cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF),
            **target_fields(Target.gps(37.366449, -120.423065)),
            battery=88,
            cur_task=0,
        ),
        TaskAnnounce(
            src=1,
            seq=7,
            task_id=4242,
            req_cap_mask=cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF),
            **target_fields(Target.tree(60)),
            priority=200,
            reason_code=ReasonCode.OPERATOR_REQUEST,
        ),
        Bid(src=3, seq=8, task_id=4242, eta_s=120, feasible=True, cost=31),
        Grant(src=1, seq=9, task_id=4242, winner_id=3),
        Ack(src=3, seq=10, ack_src=1, ack_seq=9),
    ]
    for msg in messages:
        assert decode(encode(msg)) == msg


def test_header_survives_the_round_trip_independently_of_the_payload():
    # (src, seq) is the message ID the reliability layer will key on, so it has
    # to come back untouched for every type, not just the ones with room.
    for cls in BUILT:
        template = _extreme_instances(cls)[0]
        for src, seq in [(0, 0), (1, 65535), (255, 0), (255, 65535), (7, 513)]:
            msg = type(template)(**{**template.__dict__, "src": src, "seq": seq})
            out = decode(encode(msg))
            assert (out.src, out.seq) == (src, seq)


def test_decoded_enums_are_enum_members_when_the_value_is_known():
    msg = an_announce(reason_code=ReasonCode.BATTERY_LOW)
    assert decode(encode(msg)).reason_code is ReasonCode.BATTERY_LOW


def test_unknown_enum_values_pass_through_as_ints():
    # Forward compatibility: a newer peer naming a reason we have no word for
    # is still telling us there is work, so the message must survive.
    unnamed = 200
    assert unnamed not in set(ReasonCode)
    msg = an_announce(reason_code=unnamed)
    out = decode(encode(msg))
    assert out.reason_code == unnamed
    assert out == msg


def test_an_unknown_target_kind_is_refused_rather_than_passed_through():
    # The opposite call to reason_code, deliberately. An unnamed reason is
    # informational and can be logged as a number; an unnamed target kind means
    # the two words after it are in a coordinate system we cannot read, and a
    # robot that drove to them anyway would be acting on a misreading.
    unnamed = max(TargetKind) + 1
    packet = bytearray(encode(an_announce()))
    packet[_target_kind_offset(TaskAnnounce)] = int(unnamed)
    with pytest.raises(FieldRangeError):
        decode(bytes(packet))


@pytest.mark.parametrize(
    "kind,a,b",
    [
        (TargetKind.NONE, 5, 0),  # placeless work with a coordinate
        (TargetKind.NONE, 0, 5),
        (TargetKind.TREE, 3, 9),  # a tree with a longitude
        (TargetKind.AISLE, 3, 9),
        (TargetKind.TREE, -1, 0),  # a negative index
    ],
)
def test_an_incoherent_target_triple_is_refused(kind, a, b):
    # Each word is in range on its own; together they describe nothing. A
    # sender that produces this disagrees with us about the layout, which is
    # the same class of problem as trailing bytes and is refused just as loudly.
    packet = bytearray(encode(an_announce()))
    offset = _target_kind_offset(TaskAnnounce)
    packet[offset : offset + 9] = struct.pack(">Bii", int(kind), a, b)
    with pytest.raises(FieldRangeError):
        decode(bytes(packet))


def _target_kind_offset(cls) -> int:
    """Byte offset of ``target_kind`` within an encoded ``cls``."""
    offset = HEADER_BYTES
    for field in _LAYOUTS[cls]:
        if field.name == "target_kind":
            return offset
        offset += struct.calcsize(field.fmt)
    raise AssertionError(f"{cls.__name__} carries no target")


def an_announce(task_id=4242, reason_code=ReasonCode.TASK_FAILED):
    return TaskAnnounce(
        src=1,
        seq=7,
        task_id=task_id,
        req_cap_mask=cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF),
        **target_fields(Target.tree(60)),
        priority=200,
        reason_code=reason_code,
    )


# --------------------------------------------------------------------------
# Group 2: size
# --------------------------------------------------------------------------

#: Pinned so a field width cannot drift silently. Heartbeat and TaskAnnounce
#: both grew by 5 bytes when the grid became a typed target: the kind byte plus
#: two int32 words, less the two uint16 grid fields they replaced.
EXPECTED_SIZES = {
    Heartbeat: 18,
    TaskAnnounce: 19,
    Bid: 9,
    Grant: 7,
    Ack: 7,
}


@pytest.mark.parametrize("cls", FIXED, ids=lambda c: c.__name__)
def test_encoded_size_is_the_documented_constant_for_all_valid_inputs(cls):
    expected = EXPECTED_SIZES[cls]
    for msg in _every_instance(cls):
        assert len(encode(msg)) == expected


@pytest.mark.parametrize("cls", BUILT, ids=lambda c: c.__name__)
def test_every_type_fits_the_single_packet_budget(cls):
    for msg in _every_instance(cls):
        assert len(encode(msg)) <= DEFAULT_MAX_PAYLOAD_BYTES


def test_max_message_bytes_bounds_every_fixed_size_type():
    # Freeform is excluded on purpose: there is no such number for it, and
    # reliability/node.py budgets a request-plus-ACK round trip against this.
    assert MAX_MESSAGE_BYTES == max(EXPECTED_SIZES.values())
    assert MAX_MESSAGE_BYTES <= DEFAULT_MAX_PAYLOAD_BYTES
    assert all(MESSAGE_SIZES[cls] <= MAX_MESSAGE_BYTES for cls in FIXED)


def test_encode_rejects_a_message_that_would_exceed_max_payload_bytes():
    msg = Grant(src=1, seq=1, task_id=1, winner_id=2)
    with pytest.raises(PayloadTooLarge) as exc:
        encode(msg, max_payload_bytes=len(encode(msg)) - 1)
    assert exc.value.kind == "oversize"


def test_encode_never_splits_an_oversized_message():
    # There is no fragmentation at this layer and there must never be one that
    # arrived by accident: encode returns one packet or it raises.
    msg = Heartbeat(
        src=1,
        seq=1,
        cap_mask=0,
        **target_fields(Target.none()),
        battery=0,
        cur_task=0,
    )
    with pytest.raises(PayloadTooLarge):
        encode(msg, max_payload_bytes=1)


def test_header_is_four_bytes_and_leads_every_message():
    assert HEADER_BYTES == 4
    for cls in BUILT:
        packet = encode(_extreme_instances(cls)[1])
        tag, src, seq = struct.unpack_from(">BBH", packet, 0)
        assert tag == int(cls.TAG)
        assert (src, seq) == (255, 65535)


# --------------------------------------------------------------------------
# Group 3: FREEFORM, the one type with a variable tail
# --------------------------------------------------------------------------


def _freeform(text=b"", **kwargs):
    fields = dict(
        src=5, seq=99, task_id=77, note_id=2, frag_index=0, frag_count=1, text=text
    )
    fields.update(kwargs)
    return Freeform(**fields)


@pytest.mark.parametrize(
    "text",
    [
        b"",
        b"x",
        b"soft ground past the gate",
        bytes(range(256)) * 2,
        "muddy \U0001f327 after rain".encode("utf-8"),
    ],
)
def test_a_freeform_round_trips_whatever_its_tail_holds(text):
    msg = _freeform(text=text)
    assert decode(encode(msg, max_payload_bytes=4096)) == msg


def test_the_tail_is_exactly_the_bytes_past_the_fixed_fields():
    msg = _freeform(text=b"hello")
    packet = encode(msg)
    assert len(packet) == NOTE_HEADER_BYTES + len(b"hello")
    assert packet[NOTE_HEADER_BYTES:] == b"hello"


def test_an_empty_tail_is_a_legal_packet_not_a_truncated_one():
    # A note whose text divides evenly ends with a fragment carrying nothing.
    # Refusing that would make the splitter's arithmetic a special case.
    packet = encode(_freeform(text=b""))
    assert len(packet) == NOTE_HEADER_BYTES
    assert decode(packet).text == b""


def test_trailing_bytes_are_the_message_for_a_tail_bearing_type():
    # The exact inverse of the rule the five fixed types hold, which is why
    # both are stated: extra bytes are an error there and the payload here.
    packet = encode(_freeform(text=b"one"))
    assert decode(packet + b"two").text == b"onetwo"


def test_a_freeform_shorter_than_its_fixed_fields_is_truncated_not_empty():
    frame = struct.pack(">BBH", int(MessageType.FREEFORM), 1, 1)
    with pytest.raises(TruncatedMessage):
        decode(frame)


def test_encode_refuses_a_fragment_that_does_not_fit_the_budget():
    # Still one packet or an error. That the type is fragmented elsewhere does
    # not license this layer to split anything.
    with pytest.raises(PayloadTooLarge):
        encode(_freeform(text=b"x" * 200), max_payload_bytes=DEFAULT_MAX_PAYLOAD_BYTES)


def test_encode_refuses_a_tail_that_is_not_bytes():
    # str is refused rather than encoded: a note is cut on a byte budget, so
    # whoever holds str has not been through the splitter.
    with pytest.raises(FieldRangeError):
        encode(_freeform(text="not bytes"))


def test_freeform_is_built_and_nothing_is_reserved_any_more():
    assert MessageType.FREEFORM == 0x07
    assert MessageType.FREEFORM in {cls.TAG for cls in BUILT}
    assert RESERVED_TYPES == frozenset()


def test_reserved_and_unknown_are_siblings_not_subclasses():
    assert not issubclass(ReservedMessageType, UnknownMessageType)
    assert not issubclass(UnknownMessageType, ReservedMessageType)
    assert issubclass(ReservedMessageType, CodecError)
    assert issubclass(UnknownMessageType, CodecError)


# --------------------------------------------------------------------------
# Group 4: unknown tags
# --------------------------------------------------------------------------


def test_every_unallocated_tag_decodes_to_a_clear_unknown_type_error():
    allocated = {int(t) for t in MessageType}
    for tag in range(256):
        if tag in allocated:
            continue
        frame = struct.pack(">BBH", tag, 1, 2) + bytes(9)
        with pytest.raises(UnknownMessageType) as exc:
            decode(frame)
        assert exc.value.kind == "unknown_type"
        assert f"0x{tag:02x}" in str(exc.value)


def test_tag_zero_is_unknown_rather_than_a_valid_message():
    # 0x00 is not allocated, and an all-zero buffer is the single most likely
    # thing to arrive from a wedged or resetting peer.
    with pytest.raises(UnknownMessageType):
        decode(bytes(13))


def test_encode_rejects_an_object_that_is_not_a_codec_message():
    for junk in [object(), "Heartbeat", 42, None]:
        with pytest.raises(CodecError):
            encode(junk)


# --------------------------------------------------------------------------
# Group 5: malformed input
# --------------------------------------------------------------------------


def test_empty_buffer_is_truncated_not_a_crash():
    with pytest.raises(TruncatedMessage) as exc:
        decode(b"")
    assert exc.value.kind == "truncated"


@pytest.mark.parametrize("cls", BUILT, ids=lambda c: c.__name__)
def test_every_short_prefix_of_a_valid_message_is_rejected(cls):
    packet = encode(_extreme_instances(cls)[1])
    for length in range(len(packet)):
        # struct.unpack_from on a short buffer raises struct.error, which is
        # not a CodecError -- so catching only CodecError here is what proves
        # the length was checked before any read, not after.
        with pytest.raises(TruncatedMessage):
            decode(packet[:length])


@pytest.mark.parametrize("cls", FIXED, ids=lambda c: c.__name__)
def test_trailing_bytes_are_rejected_rather_than_ignored(cls):
    packet = encode(_extreme_instances(cls)[0])
    for extra in [b"\x00", b"\xff", b"more", bytes(200)]:
        with pytest.raises(TrailingBytes) as exc:
            decode(packet + extra)
        assert exc.value.kind == "trailing"


@pytest.mark.parametrize("cls", FIXED, ids=lambda c: c.__name__)
def test_two_concatenated_messages_are_rejected(cls):
    # One message is one packet. Concatenation is a sender bug, and silently
    # decoding the first half would hide it.
    packet = encode(_extreme_instances(cls)[0])
    with pytest.raises(TrailingBytes):
        decode(packet + packet)


def test_decode_of_arbitrary_garbage_never_raises_anything_but_codec_error():
    rng = random.Random("fuzz")
    for _ in range(20000):
        data = rng.randbytes(rng.randrange(0, 64))
        try:
            decode(data)
        except CodecError:
            pass
        # Anything else -- struct.error, IndexError, ValueError, TypeError --
        # propagates and fails the test, which is the whole assertion.


def test_decode_accepts_any_bytes_like_object():
    packet = encode(Ack(src=1, seq=2, ack_src=3, ack_seq=4))
    expected = decode(packet)
    assert decode(bytearray(packet)) == expected
    assert decode(memoryview(packet)) == expected


def test_decode_rejects_a_non_buffer_without_crashing():
    with pytest.raises(CodecError):
        decode("not bytes")


def test_decode_rejects_structurally_valid_but_impossible_values():
    # 200% battery is not a bit flip the CRC missed -- the frame passed CRC to
    # get here -- it is a protocol bug, and passing it up as an object the rest
    # of the stack trusts is strictly worse than dropping it.
    frame = struct.pack(">BBH", int(MessageType.HEARTBEAT), 1, 1) + struct.pack(
        ">HBiiBH", 0, int(TargetKind.NONE), 0, 0, 200, 0
    )
    with pytest.raises(FieldRangeError) as exc:
        decode(frame)
    assert exc.value.kind == "range"


def test_decode_rejects_a_non_boolean_feasible_byte():
    frame = struct.pack(">BBH", int(MessageType.BID), 1, 1) + struct.pack(
        ">HBBB", 1, 10, 2, 5
    )
    with pytest.raises(FieldRangeError):
        decode(frame)


# --------------------------------------------------------------------------
# Encode-side range checking
# --------------------------------------------------------------------------


@pytest.mark.parametrize("cls", BUILT, ids=lambda c: c.__name__)
def test_encode_rejects_out_of_range_values_instead_of_truncating(cls):
    fields = _all_fields(cls)
    template = _extreme_instances(cls)[0]
    for field in fields:
        if field.is_bool:
            continue
        for bad in (field.lo - 1, field.hi + 1):
            msg = type(template)(**{**template.__dict__, field.name: bad})
            with pytest.raises(FieldRangeError) as exc:
                encode(msg)
            assert field.name in str(exc.value)


def test_encode_rejects_a_fractional_value_in_an_unquantized_field():
    # Note what this implies for GPS: degrees never reach the encoder as
    # floats. Target.gps() rounds them onto the 1e-7 lattice at construction,
    # and a caller who assembled the words by hand from raw degrees is caught
    # here rather than having 37.366449 quietly become tree 37.
    msg = Heartbeat(
        src=1,
        seq=1,
        cap_mask=0,
        target_kind=int(TargetKind.GPS),
        target_a=3.7,
        target_b=0,
        battery=50,
        cur_task=0,
    )
    with pytest.raises(FieldRangeError):
        encode(msg)


def test_encode_rejects_non_numeric_field_values():
    msg = Grant(src=1, seq=1, task_id="4242", winner_id=3)
    with pytest.raises(FieldRangeError):
        encode(msg)


# --------------------------------------------------------------------------
# Quantization
# --------------------------------------------------------------------------


def test_eta_round_trips_exactly_on_the_quantization_lattice():
    for eta in range(0, ETA_MAX_S + 1, ETA_RESOLUTION_S):
        msg = Bid(src=1, seq=1, task_id=1, eta_s=eta, feasible=True, cost=0)
        assert decode(encode(msg)).eta_s == eta


def test_eta_quantization_error_is_bounded_by_half_an_lsb():
    # The round-trip test uses lattice values, so this is the test that pins
    # what an off-lattice ETA actually costs: no more than 2 s, and never a
    # wrap or a silent truncation to zero.
    worst = 0
    for eta in range(0, ETA_MAX_S + 1):
        msg = Bid(src=1, seq=1, task_id=1, eta_s=eta, feasible=True, cost=0)
        decoded = decode(encode(msg)).eta_s
        assert 0 <= decoded <= ETA_MAX_S
        worst = max(worst, abs(decoded - eta))
    assert worst <= ETA_RESOLUTION_S / 2


def test_eta_beyond_the_representable_range_is_refused_not_wrapped():
    msg = Bid(src=1, seq=1, task_id=1, eta_s=ETA_MAX_S + 1, feasible=False, cost=0)
    with pytest.raises(FieldRangeError):
        encode(msg)


# --------------------------------------------------------------------------
# Targets
# --------------------------------------------------------------------------


@pytest.mark.parametrize("target", LEGAL_TARGETS, ids=str)
def test_every_target_survives_the_round_trip_unchanged(target):
    for cls in _TARGETED:
        template = _extreme_instances(cls)[0]
        msg = type(template)(**{**template.__dict__, **target_fields(target)})
        assert target_of(decode(encode(msg))) == target


def test_gps_round_trips_within_the_documented_resolution():
    # 1e-7 deg is about 1.1 cm. The orchard fixes in examples/quad.xml are
    # given to 1e-7 or coarser, so the wire loses nothing a mission ever had.
    fixes = [
        (37.366449, -120.423065),
        (37.3664050000283, -120.4230154999136),
        (37.366351898723934, -120.42261198151091),
        (0.0, 0.0),
        (90.0, 180.0),
        (-90.0, -180.0),
    ]
    for lat, lon in fixes:
        out = target_of(decode(encode(a_heartbeat_at(Target.gps(lat, lon)))))
        assert abs(out.lat_deg - lat) <= 0.5 / GPS_SCALE
        assert abs(out.lon_deg - lon) <= 0.5 / GPS_SCALE


def test_a_gps_target_outside_the_globe_is_refused_at_construction():
    # Caught in Target rather than at encode time: an out-of-range latitude is
    # a mistake in whatever computed it, and the traceback is worth more there
    # than three layers down inside struct.pack.
    for lat, lon in [(91.0, 0.0), (-91.0, 0.0), (0.0, 181.0), (0.0, -181.0)]:
        with pytest.raises(ValueError):
            Target.gps(lat, lon)


def test_a_placeless_target_is_distinguishable_from_the_origin():
    # The whole reason NONE exists. Under the old grid both "no GPS fix" and
    # "at 0,0" encoded as zeros, so a robot with no fix advertised itself in
    # the Gulf of Guinea and every ETA computed against it was fiction.
    assert Target.none() != Target.gps(0.0, 0.0)
    here = target_of(decode(encode(a_heartbeat_at(Target.none()))))
    origin = target_of(decode(encode(a_heartbeat_at(Target.gps(0.0, 0.0)))))
    assert here != origin
    assert not here.placed and origin.placed


def a_heartbeat_at(target):
    return Heartbeat(
        src=1,
        seq=1,
        cap_mask=cap_mask(Capability.MOVE_TO_TREE_ID),
        **target_fields(target),
        battery=100,
        cur_task=0,
    )


# --------------------------------------------------------------------------
# Capability mask
# --------------------------------------------------------------------------


def test_cap_mask_and_has_capability_are_inverses():
    caps = [
        Capability.MOVE_TO_TREE_ID,
        Capability.SAMPLE_LEAF,
        Capability.FOLLOW_PERSON,
    ]
    mask = cap_mask(*caps)
    for capability in Capability:
        assert has_capability(mask, capability) == (capability in caps)


def test_cap_mask_survives_a_heartbeat_round_trip():
    msg = a_heartbeat_at(Target.none())
    msg.cap_mask = cap_mask(*Capability)
    assert set(capabilities_in(decode(encode(msg)).cap_mask)) == set(Capability)


def test_a_task_requirement_can_be_tested_against_an_advertised_mask():
    # The one operation the arbiter needs, and the reason capabilities are bit
    # indices rather than mask values: the two message types interoperate.
    # The task is the real one from examples/sample_leafs.xml -- approach tree
    # 60 and sample it -- which needs both actions and not either alone.
    sampler = a_heartbeat_at(Target.gps(37.366449, -120.423065))
    sampler.cap_mask = cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF)
    announce = an_announce()

    assert has_capabilities(
        decode(encode(sampler)).cap_mask, decode(encode(announce)).req_cap_mask
    )


def test_a_robot_missing_one_action_of_a_task_does_not_qualify():
    # The reason the requirement is a mask and not an index. An arm that cannot
    # drive to the tree passes any single-action test and then cannot place the
    # work it just won.
    armless = cap_mask(Capability.MOVE_TO_TREE_ID)
    rootless = cap_mask(Capability.SAMPLE_LEAF)
    required = decode(encode(an_announce())).req_cap_mask

    assert not has_capabilities(armless, required)
    assert not has_capabilities(rootless, required)
    assert has_capabilities(armless | rootless, required)


def test_an_empty_requirement_is_satisfied_by_anything():
    assert has_capabilities(0, 0)
    assert has_capabilities(cap_mask(Capability.SAMPLE_LEAF), 0)


def test_every_capability_index_fits_the_advertised_mask_width():
    for capability in Capability:
        assert 0 <= int(capability) < 16


def test_capability_indices_are_unique():
    values = [int(c) for c in Capability]
    assert len(values) == len(set(values))


def test_the_capability_vocabulary_is_the_behaviour_trees_action_group():
    """Every capability names an action the mission schema actually permits.

    This is the test that keeps the wire honest. The whole point of these bits
    is that a robot advertises what its behaviour tree can be asked to do, so a
    capability with no corresponding XSD element is a robot claiming a skill no
    mission could ever invoke -- and an XSD element with no bit is work the
    fleet can never delegate. Both directions have to hold.
    """
    schema = _installed_schema()
    permitted = _action_group_elements(schema)

    assert permitted, f"no ActionGroup elements found in {schema}"
    assert {XML_ELEMENT[c] for c in Capability} == permitted

    # DetectObject is registered in bt.cpp but commented out of the schema.
    # Advertising it would be a claim no mission could exercise.
    assert "DetectObject" not in permitted


def _installed_schema() -> str:
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(
        os.path.dirname(here),
        "amiga_ros2_behavior_tree",
        "schemas",
        "amiga_btcpp.xsd",
    )


def _action_group_elements(path: str) -> set:
    """The element names inside <xs:group name="ActionGroup">.

    Deliberately parsed here rather than imported from the coordinator's
    capabilities.py: a test that used the same parser as the code it checks
    would agree with it about a schema neither had read correctly.
    """
    xs = "{http://www.w3.org/2001/XMLSchema}"
    root = ElementTree.parse(path).getroot()
    for group in root.iter(f"{xs}group"):
        if group.get("name") != "ActionGroup":
            continue
        return {
            element.get("name")
            for element in group.iter(f"{xs}element")
            if element.get("name")
        }
    return set()


# --------------------------------------------------------------------------
# Vocabulary integrity
# --------------------------------------------------------------------------


def test_every_built_type_has_a_distinct_tag():
    tags = [cls.TAG for cls in BUILT]
    assert len(tags) == len(set(tags))


def test_the_built_types_are_exactly_the_ones_specified():
    assert {cls.TAG for cls in BUILT} == {
        MessageType.HEARTBEAT,
        MessageType.TASK_ANNOUNCE,
        MessageType.BID,
        MessageType.GRANT,
        MessageType.ACK,
        MessageType.FREEFORM,
    }


def test_only_freeform_carries_a_variable_tail():
    # The restriction is structural: a tail has no length of its own, so it can
    # only be the last field, and only one field can be last.
    assert set(_TAILS) == {Freeform}


def test_message_type_values_match_the_wire_contract():
    # Renumbering a shipped tag silently breaks every peer, so the numbers are
    # pinned here rather than merely being whatever the enum happens to say.
    assert [int(t) for t in MessageType] == [1, 2, 3, 4, 5, 7]


def test_the_retired_hazard_tag_is_not_quietly_reused():
    # 0x06 was HAZARD. Nothing ever detected a hazard, so the type was removed
    # rather than left as a promise -- but the gap stays a gap: giving 0x06 to
    # a new type would make any peer still holding the old code silently
    # misread it.
    assert 0x06 not in {int(t) for t in MessageType}
    with pytest.raises(UnknownMessageType):
        decode(struct.pack(">BBH", 0x06, 1, 1))
