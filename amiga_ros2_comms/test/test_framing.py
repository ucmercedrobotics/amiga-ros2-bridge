#!/usr/bin/env python3
"""Unit tests for COBS framing and CRC-8. No ROS, no serial port."""

import os
import random
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.lora.framing import (  # noqa: E402
    DELIMITER,
    MAX_BODY_BYTES,
    FrameError,
    FrameParser,
    cobs_decode,
    cobs_encode,
    crc8,
    decode_frame,
    encode_frame,
    max_wire_frame_bytes,
)


def test_crc8_check_value():
    # The published check value for CRC-8/ATM. Firmware must reproduce this, so
    # it is the one number in the contract worth pinning in a test.
    assert crc8(b"123456789") == 0xF4


def test_crc8_detects_every_single_bit_flip_in_short_payloads():
    # A CRC-8 cannot catch everything, but its whole job here is single-bit
    # UART errors, and for those it must be exact.
    payload = bytes(range(32))
    baseline = crc8(payload)
    for index in range(len(payload)):
        for bit in range(8):
            corrupted = bytearray(payload)
            corrupted[index] ^= 1 << bit
            assert crc8(bytes(corrupted)) != baseline


@pytest.mark.parametrize(
    "data",
    [
        b"",
        b"\x00",
        b"\x00\x00\x00",
        b"hello",
        b"\x01\x00\x02\x00\x03",
        bytes(254),
        bytes(range(256)) * 2,
        b"\xff" * 300,
    ],
)
def test_cobs_round_trip(data):
    encoded = cobs_encode(data)
    assert DELIMITER not in encoded, "encoded form must be zero-free"
    assert cobs_decode(encoded) == data


def test_cobs_round_trip_random():
    rng = random.Random(1234)
    for _ in range(500):
        length = rng.randrange(0, 600)
        data = bytes(rng.randrange(0, 256) for _ in range(length))
        assert cobs_decode(cobs_encode(data)) == data


def test_cobs_rejects_malformed():
    with pytest.raises(FrameError):
        cobs_decode(b"\x00")  # zero code byte
    with pytest.raises(FrameError):
        cobs_decode(b"\x05ab")  # code overruns the frame


@pytest.mark.parametrize("length", [0, 1, 2, 100, 200, 253, 254, 255])
def test_frame_round_trip(length):
    payload = bytes((i * 7 + 3) % 256 for i in range(length))
    wire = encode_frame(payload)
    assert wire[-1] == DELIMITER
    assert DELIMITER not in wire[:-1]
    assert decode_frame(wire[:-1], MAX_BODY_BYTES, False).payload == payload


def test_wire_size_bound_holds():
    for length in (0, 1, 253, 254, 255):
        assert len(encode_frame(bytes(length))) <= max_wire_frame_bytes(MAX_BODY_BYTES)


def test_encode_rejects_oversize_body():
    with pytest.raises(FrameError):
        encode_frame(bytes(MAX_BODY_BYTES + 1))


# ----------------------------------------------------------------------
# Corruption: acceptance test 2, at the framing level
# ----------------------------------------------------------------------


def _frame_with_corrupt_payload(payload, byte_index, bit):
    """Build a frame whose CRC is correct, then flip a payload bit.

    Flipping after the CRC is computed but before COBS encoding is what puts the
    fault squarely in the CRC's path, rather than in COBS structure checking.
    """
    body = bytearray(payload)
    logical = bytearray([len(body)]) + body + bytearray([crc8(bytes(body))])
    logical[1 + byte_index] ^= 1 << bit
    return cobs_encode(bytes(logical)) + bytes([DELIMITER])


def test_payload_bit_flip_is_caught_by_crc():
    payload = b"the quick brown fox jumps over the lazy dog"
    parser = FrameParser()
    for index in range(len(payload)):
        for bit in range(8):
            frames = parser.feed(_frame_with_corrupt_payload(payload, index, bit))
            assert frames == [], "a corrupt frame must never be surfaced"
    assert parser.stats.crc_errors == len(payload) * 8
    assert parser.stats.frames == 0


def test_length_byte_corruption_is_caught():
    # The length byte is deliberately outside the CRC; the decoded-size
    # consistency check is what covers it.
    wire = bytearray(encode_frame(b"payload bytes"))
    logical = bytearray(cobs_decode(bytes(wire[:-1])))
    logical[0] ^= 0x10
    corrupted = cobs_encode(bytes(logical)) + bytes([DELIMITER])

    parser = FrameParser()
    assert parser.feed(corrupted) == []
    assert parser.stats.length_errors == 1


def test_corrupt_frame_does_not_poison_the_next_one():
    parser = FrameParser()
    good = b"i should still arrive"
    stream = _frame_with_corrupt_payload(b"i am broken", 3, 2) + encode_frame(good)
    frames = parser.feed(stream)
    assert [f.payload for f in frames] == [good]
    assert parser.stats.crc_errors == 1


def test_oversize_frame_rejected_against_max_payload():
    parser = FrameParser(max_payload_bytes=16)
    assert parser.feed(encode_frame(bytes(32))) == []
    assert parser.stats.oversize_errors == 1
    assert parser.feed(encode_frame(bytes(16))) != []


# ----------------------------------------------------------------------
# Stream behaviour
# ----------------------------------------------------------------------


def test_parser_reassembles_across_arbitrary_chunk_boundaries():
    rng = random.Random(99)
    payloads = [
        bytes(rng.randrange(256) for _ in range(rng.randrange(0, 200)))
        for _ in range(40)
    ]
    stream = b"".join(encode_frame(p) for p in payloads)

    parser = FrameParser()
    received = []
    cursor = 0
    while cursor < len(stream):
        step = rng.randrange(1, 37)
        received += parser.feed(stream[cursor : cursor + step])
        cursor += step

    assert [f.payload for f in received] == payloads
    assert parser.stats.drops == 0


def test_parser_resynchronises_after_joining_mid_stream():
    payloads = [b"first", b"second", b"third"]
    stream = b"".join(encode_frame(p) for p in payloads)
    # Start reading partway into the first frame, as if we opened the port while
    # the Arduino was already transmitting.
    truncated = stream[4:]

    parser = FrameParser()
    frames = parser.feed(truncated)
    # The frame we joined halfway through is lost; every later one survives.
    assert [f.payload for f in frames] == payloads[1:]


def test_parser_tolerates_repeated_delimiters():
    parser = FrameParser()
    stream = b"\x00\x00" + encode_frame(b"abc") + b"\x00\x00\x00" + encode_frame(b"de")
    assert [f.payload for f in parser.feed(stream)] == [b"abc", b"de"]
    assert parser.stats.drops == 0


def test_parser_discards_a_run_with_no_delimiter_and_recovers():
    parser = FrameParser(max_payload_bytes=32)
    noise = b"\xaa" * (max_wire_frame_bytes(MAX_BODY_BYTES) * 3)
    assert parser.feed(noise) == []
    assert parser.stats.resyncs >= 1
    # Buffer never grew without bound, and a real frame still gets through.
    assert [f.payload for f in parser.feed(b"\x00" + encode_frame(b"ok"))] == [b"ok"]


# ----------------------------------------------------------------------
# Optional link stats
# ----------------------------------------------------------------------


def test_link_stats_parsed_when_firmware_sends_them():
    parser = FrameParser(max_payload_bytes=64, with_link_stats=True)
    wire = encode_frame(b"telemetry", link_stats=(-97, -7.25))
    (frame,) = parser.feed(wire)
    assert frame.payload == b"telemetry"
    assert frame.has_link_stats
    assert frame.rssi == -97
    assert frame.snr == pytest.approx(-7.25)


def test_link_stats_absent_by_default():
    parser = FrameParser(max_payload_bytes=64)
    (frame,) = parser.feed(encode_frame(b"telemetry"))
    assert frame.payload == b"telemetry"
    assert not frame.has_link_stats
    assert frame.rssi is None and frame.snr is None
