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
from dataclasses import dataclass
from typing import ClassVar

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import (  # noqa: E402
    DEFAULT_MAX_PAYLOAD_BYTES,
    ETA_MAX_S,
    ETA_RESOLUTION_S,
    HEADER_BYTES,
    MAX_MESSAGE_BYTES,
    Ack,
    Bid,
    Capability,
    CodecError,
    FieldRangeError,
    Grant,
    Hazard,
    HazardClass,
    Heartbeat,
    Message,
    MessageType,
    PayloadTooLarge,
    ReasonCode,
    ReservedMessageType,
    TaskAnnounce,
    TrailingBytes,
    TruncatedMessage,
    UnknownMessageType,
    cap_mask,
    capabilities_in,
    decode,
    encode,
    has_capability,
)
from amiga_ros2_comms.codec.codec import (  # noqa: E402
    _HEADER_FIELDS,
    _LAYOUTS,
)

BUILT = tuple(_LAYOUTS)


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


def _extreme_instances(cls):
    """All-minimum and all-maximum instances of ``cls``."""
    fields = _all_fields(cls)
    return [cls(**{f.name: _lattice_values(f)[i] for f in fields}) for i in (0, 1)]


def _boundary_instances(cls):
    """Every field driven to each of its bounds, with the rest mid-range.

    Sweeping one field at a time is what catches a field packed at the wrong
    offset: an all-max instance can look correct even if two same-width fields
    are transposed.
    """
    fields = _all_fields(cls)
    rng = random.Random(f"boundary-{cls.__name__}")
    base = {f.name: _random_value(f, rng) for f in fields}
    out = []
    for field in fields:
        for value in _lattice_values(field):
            values = dict(base)
            values[field.name] = value
            out.append(cls(**values))
    return out


def _random_instances(cls, count=200):
    rng = random.Random(f"random-{cls.__name__}")
    fields = _all_fields(cls)
    return [
        cls(**{f.name: _random_value(f, rng) for f in fields}) for _ in range(count)
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
    messages = [
        Heartbeat(
            src=3,
            seq=1024,
            cap_mask=cap_mask(Capability.DRIVE, Capability.SPRAY),
            grid_row=12,
            grid_col=47,
            battery=88,
            cur_task=0,
        ),
        TaskAnnounce(
            src=1,
            seq=7,
            task_id=4242,
            req_capability=Capability.SPRAY,
            grid_row=12,
            grid_col=47,
            priority=200,
            reason_code=ReasonCode.OPERATOR_REQUEST,
        ),
        Bid(src=3, seq=8, task_id=4242, eta_s=120, feasible=True, cost=31),
        Grant(src=1, seq=9, task_id=4242, winner_id=3),
        Ack(src=3, seq=10, ack_src=1, ack_seq=9),
        Hazard(
            src=2,
            seq=11,
            hazard_class=HazardClass.HUMAN,
            grid_row=13,
            grid_col=47,
            radius=2,
            confidence=91,
            ttl_s=600,
        ),
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
    msg = Hazard(
        src=1,
        seq=1,
        hazard_class=HazardClass.ANIMAL,
        grid_row=0,
        grid_col=0,
        radius=1,
        confidence=50,
        ttl_s=60,
    )
    assert decode(encode(msg)).hazard_class is HazardClass.ANIMAL


def test_unknown_enum_values_pass_through_as_ints():
    # Forward compatibility: a newer peer naming a hazard class we lack is
    # still telling us where the hazard is, so the message must survive.
    unnamed = 200
    assert unnamed not in set(HazardClass)
    msg = Hazard(
        src=1,
        seq=1,
        hazard_class=unnamed,
        grid_row=5,
        grid_col=5,
        radius=1,
        confidence=50,
        ttl_s=60,
    )
    out = decode(encode(msg))
    assert out.hazard_class == unnamed
    assert out == msg


# --------------------------------------------------------------------------
# Group 2: size
# --------------------------------------------------------------------------

#: The sizes the brief specifies. Pinned so a field width cannot drift silently.
EXPECTED_SIZES = {
    Heartbeat: 13,
    TaskAnnounce: 13,
    Bid: 9,
    Grant: 7,
    Ack: 7,
    Hazard: 13,
}


@pytest.mark.parametrize("cls", BUILT, ids=lambda c: c.__name__)
def test_encoded_size_is_the_documented_constant_for_all_valid_inputs(cls):
    expected = EXPECTED_SIZES[cls]
    for msg in _every_instance(cls):
        assert len(encode(msg)) == expected


@pytest.mark.parametrize("cls", BUILT, ids=lambda c: c.__name__)
def test_every_type_fits_the_single_packet_budget(cls):
    for msg in _every_instance(cls):
        assert len(encode(msg)) <= DEFAULT_MAX_PAYLOAD_BYTES


def test_max_message_bytes_bounds_every_type():
    assert MAX_MESSAGE_BYTES == max(EXPECTED_SIZES.values())
    assert MAX_MESSAGE_BYTES <= DEFAULT_MAX_PAYLOAD_BYTES


def test_encode_rejects_a_message_that_would_exceed_max_payload_bytes():
    msg = Grant(src=1, seq=1, task_id=1, winner_id=2)
    with pytest.raises(PayloadTooLarge) as exc:
        encode(msg, max_payload_bytes=len(encode(msg)) - 1)
    assert exc.value.kind == "oversize"


def test_encode_never_splits_an_oversized_message():
    # There is no fragmentation at this layer and there must never be one that
    # arrived by accident: encode returns one packet or it raises.
    msg = Heartbeat(
        src=1, seq=1, cap_mask=0, grid_row=0, grid_col=0, battery=0, cur_task=0
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
# Group 3: the reserved FREEFORM tag
# --------------------------------------------------------------------------


@dataclass
class _Freeform(Message):
    """Stands in for the unbuilt FREEFORM type, to prove encode refuses it."""

    TAG: ClassVar[MessageType] = MessageType.FREEFORM


def test_encode_refuses_to_produce_freeform():
    with pytest.raises(ReservedMessageType) as exc:
        encode(_Freeform(src=1, seq=1))
    assert exc.value.kind == "reserved_type"


@pytest.mark.parametrize("extra", [b"", b"\x00", b"hello world", bytes(64)])
def test_decode_of_a_freeform_frame_reports_reserved_not_unknown(extra):
    frame = struct.pack(">BBH", int(MessageType.FREEFORM), 5, 99) + extra
    with pytest.raises(ReservedMessageType) as exc:
        decode(frame)
    # The distinction is the point: "we cannot handle this yet" and "we have
    # never heard of this" call for different operator responses.
    assert not isinstance(exc.value, UnknownMessageType)
    assert exc.value.kind == "reserved_type"


def test_reserved_is_reported_even_for_a_truncated_freeform_frame():
    # Tag identification must not depend on the rest of the frame being sane.
    frame = struct.pack(">BBH", int(MessageType.FREEFORM), 1, 1)
    with pytest.raises(ReservedMessageType):
        decode(frame)


def test_freeform_is_the_only_reserved_tag_and_has_no_layout():
    assert MessageType.FREEFORM not in {cls.TAG for cls in BUILT}
    assert MessageType.FREEFORM == 0x07


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


@pytest.mark.parametrize("cls", BUILT, ids=lambda c: c.__name__)
def test_trailing_bytes_are_rejected_rather_than_ignored(cls):
    packet = encode(_extreme_instances(cls)[0])
    for extra in [b"\x00", b"\xff", b"more", bytes(200)]:
        with pytest.raises(TrailingBytes) as exc:
            decode(packet + extra)
        assert exc.value.kind == "trailing"


@pytest.mark.parametrize("cls", BUILT, ids=lambda c: c.__name__)
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
        ">HHHBH", 0, 0, 0, 200, 0
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
    msg = Heartbeat(
        src=1, seq=1, cap_mask=0, grid_row=3.7, grid_col=0, battery=50, cur_task=0
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


def test_ttl_is_unquantized_and_exact_to_the_second():
    for ttl in [0, 1, 59, 60, 3599, 3600, 65534, 65535]:
        msg = Hazard(
            src=1,
            seq=1,
            hazard_class=HazardClass.OBSTACLE,
            grid_row=0,
            grid_col=0,
            radius=0,
            confidence=0,
            ttl_s=ttl,
        )
        assert decode(encode(msg)).ttl_s == ttl


# --------------------------------------------------------------------------
# Capability mask
# --------------------------------------------------------------------------


def test_cap_mask_and_has_capability_are_inverses():
    caps = [Capability.DRIVE, Capability.HARVEST, Capability.RELAY]
    mask = cap_mask(*caps)
    for capability in Capability:
        assert has_capability(mask, capability) == (capability in caps)


def test_cap_mask_survives_a_heartbeat_round_trip():
    mask = cap_mask(*Capability)
    msg = Heartbeat(
        src=9, seq=9, cap_mask=mask, grid_row=1, grid_col=1, battery=100, cur_task=0
    )
    assert set(capabilities_in(decode(encode(msg)).cap_mask)) == set(Capability)


def test_a_task_requirement_can_be_tested_against_an_advertised_mask():
    # The one operation the arbiter needs, and the reason capabilities are bit
    # indices rather than mask values: the two message types interoperate.
    heartbeat = Heartbeat(
        src=2,
        seq=1,
        cap_mask=cap_mask(Capability.DRIVE, Capability.SPRAY),
        grid_row=0,
        grid_col=0,
        battery=50,
        cur_task=0,
    )
    announce = TaskAnnounce(
        src=1,
        seq=1,
        task_id=1,
        req_capability=Capability.SPRAY,
        grid_row=0,
        grid_col=0,
        priority=1,
        reason_code=ReasonCode.SCHEDULED,
    )
    assert has_capability(
        decode(encode(heartbeat)).cap_mask, decode(encode(announce)).req_capability
    )


def test_every_capability_index_fits_the_advertised_mask_width():
    for capability in Capability:
        assert 0 <= int(capability) < 16


def test_capability_indices_are_unique():
    values = [int(c) for c in Capability]
    assert len(values) == len(set(values))


# --------------------------------------------------------------------------
# Vocabulary integrity
# --------------------------------------------------------------------------


def test_every_built_type_has_a_distinct_tag():
    tags = [cls.TAG for cls in BUILT]
    assert len(tags) == len(set(tags))


def test_the_six_built_types_are_exactly_the_ones_specified():
    assert {cls.TAG for cls in BUILT} == {
        MessageType.HEARTBEAT,
        MessageType.TASK_ANNOUNCE,
        MessageType.BID,
        MessageType.GRANT,
        MessageType.ACK,
        MessageType.HAZARD,
    }


def test_message_type_values_match_the_wire_contract():
    # Renumbering a shipped tag silently breaks every peer, so the numbers are
    # pinned here rather than merely being whatever the enum happens to say.
    assert [int(t) for t in MessageType] == [1, 2, 3, 4, 5, 6, 7]
