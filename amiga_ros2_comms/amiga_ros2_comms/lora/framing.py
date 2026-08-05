#!/usr/bin/env python3
"""COBS framing and CRC-8 for the LoRa serial link.

Implements the host side of docs/lora_frame_contract.md. Pure Python with no
ROS dependency, so it can be unit-tested on its own.

Payloads are opaque. Nothing here knows or cares what the bytes mean.
"""

from dataclasses import dataclass, field
from typing import List, Optional

# CRC-8/ATM: poly 0x07, init 0x00, no reflection, no final XOR.
_CRC8_POLY = 0x07

# Frame delimiter. COBS guarantees the encoded body never contains one.
DELIMITER = 0x00

# The length field is a single byte, so this is the absolute ceiling on a frame
# body regardless of how max_payload_bytes is configured.
MAX_BODY_BYTES = 255

# Wire size of the link-stats header, when firmware prepends one.
LINK_STATS_BYTES = 3


def _build_crc8_table() -> bytes:
    table = bytearray(256)
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ _CRC8_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table[i] = crc
    return bytes(table)


_CRC8_TABLE = _build_crc8_table()


def crc8(data: bytes) -> int:
    """CRC-8/ATM over ``data``. Check value for b"123456789" is 0xF4."""
    crc = 0x00
    for byte in data:
        crc = _CRC8_TABLE[crc ^ byte]
    return crc


class FrameError(ValueError):
    """A frame was malformed or failed validation. Always means "drop it"."""

    def __init__(self, reason: str, kind: str):
        super().__init__(reason)
        # One of: cobs, crc, length, oversize. Used to pick a counter.
        self.kind = kind


def cobs_encode(data: bytes) -> bytes:
    """Consistent Overhead Byte Stuffing. Output contains no zero bytes."""
    out = bytearray(b"\x00")  # placeholder for the first code byte
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Inverse of :func:`cobs_encode`. Raises FrameError on malformed input."""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise FrameError("zero code byte inside frame", "cobs")
        i += 1
        block_end = i + code - 1
        if block_end > n:
            raise FrameError("code byte overruns frame", "cobs")
        out += data[i:block_end]
        i = block_end
        # A 0xFF block is a continuation, not a zero that was stuffed out. A
        # block that ends exactly at the frame end has no zero after it either.
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def max_wire_frame_bytes(max_body_bytes: int = MAX_BODY_BYTES) -> int:
    """Largest number of wire bytes one frame can occupy, delimiter included."""
    logical = max_body_bytes + 2  # length byte + body + crc
    return logical + (logical // 254 + 1) + 1


def encode_frame(payload: bytes, link_stats: Optional[tuple] = None) -> bytes:
    """Build one delimited wire frame from an opaque payload.

    ``link_stats``, if given, is an ``(rssi_dbm, snr_db)`` pair prepended to the
    body. Only firmware sends those; the host passes None. It exists here so
    tests can synthesise inbound frames without firmware.
    """
    body = bytes(payload)
    if link_stats is not None:
        rssi, snr = link_stats
        body = (
            int(rssi).to_bytes(2, "little", signed=True)
            + int(round(snr * 4)).to_bytes(1, "little", signed=True)
            + body
        )
    if len(body) > MAX_BODY_BYTES:
        raise FrameError(f"body of {len(body)} exceeds {MAX_BODY_BYTES}", "oversize")
    logical = bytes([len(body)]) + body + bytes([crc8(body)])
    return cobs_encode(logical) + bytes([DELIMITER])


@dataclass
class Frame:
    """One validated inbound frame."""

    payload: bytes
    rssi: Optional[int] = None
    snr: Optional[float] = None

    @property
    def has_link_stats(self) -> bool:
        return self.rssi is not None


def decode_frame(
    encoded: bytes, max_payload_bytes: int, with_link_stats: bool
) -> Frame:
    """Validate and unpack one COBS-encoded frame (delimiter already stripped).

    Raises FrameError with a ``kind`` naming which check failed.
    """
    logical = cobs_decode(encoded)
    if len(logical) < 2:
        raise FrameError("frame shorter than length+crc", "length")

    declared = logical[0]
    body = logical[1:-1]
    if declared != len(body):
        raise FrameError(f"length byte says {declared}, decoded {len(body)}", "length")
    if crc8(body) != logical[-1]:
        raise FrameError("crc mismatch", "crc")

    rssi = snr = None
    if with_link_stats:
        if len(body) < LINK_STATS_BYTES:
            raise FrameError("frame too short to hold link stats", "length")
        rssi = int.from_bytes(body[0:2], "little", signed=True)
        snr = int.from_bytes(body[2:3], "little", signed=True) / 4.0
        body = body[LINK_STATS_BYTES:]

    if len(body) > max_payload_bytes:
        raise FrameError(
            f"payload of {len(body)} exceeds max_payload_bytes={max_payload_bytes}",
            "oversize",
        )
    return Frame(payload=body, rssi=rssi, snr=snr)


@dataclass
class ParserStats:
    """Counters for everything the parser has seen. Diagnostics only."""

    frames: int = 0
    crc_errors: int = 0
    cobs_errors: int = 0
    length_errors: int = 0
    oversize_errors: int = 0
    resyncs: int = 0

    def as_dict(self) -> dict:
        counters = dict(self.__dict__)
        counters["drops"] = self.drops
        return counters

    @property
    def drops(self) -> int:
        return (
            self.crc_errors
            + self.cobs_errors
            + self.length_errors
            + self.oversize_errors
            + self.resyncs
        )


@dataclass
class FrameParser:
    """Incremental parser: feed it arbitrary byte chunks, get whole frames out.

    Self-resynchronising. A corrupt or truncated frame costs at most the frames
    between two delimiters; parsing picks straight back up at the next one.
    """

    max_payload_bytes: int = MAX_BODY_BYTES
    with_link_stats: bool = False
    stats: ParserStats = field(default_factory=ParserStats)
    _buffer: bytearray = field(default_factory=bytearray, repr=False)

    def __post_init__(self):
        # Sized to the protocol ceiling, not to max_payload_bytes. The guard
        # exists to bound memory against a line that never delimits, and 260
        # bytes is already negligible. Sizing it to the configured limit instead
        # would shred an oversized-but-well-formed frame into a resync, hiding
        # the very mismatch oversize_errors is there to report.
        self._max_wire = max_wire_frame_bytes(MAX_BODY_BYTES)

    def feed(self, chunk: bytes) -> List[Frame]:
        """Consume bytes off the wire, returning every frame they completed."""
        frames: List[Frame] = []
        for byte in chunk:
            if byte != DELIMITER:
                self._buffer.append(byte)
                # A line that never delimits would otherwise grow without bound.
                # Anything this long cannot become a valid frame, so drop it and
                # wait for the next delimiter to re-anchor us.
                if len(self._buffer) > self._max_wire:
                    self._buffer.clear()
                    self.stats.resyncs += 1
                continue

            if not self._buffer:
                # Repeated delimiters are legal padding, not an error.
                continue
            encoded = bytes(self._buffer)
            self._buffer.clear()
            try:
                frames.append(
                    decode_frame(encoded, self.max_payload_bytes, self.with_link_stats)
                )
                self.stats.frames += 1
            except FrameError as exc:
                setattr(
                    self.stats,
                    f"{exc.kind}_errors",
                    getattr(self.stats, f"{exc.kind}_errors") + 1,
                )
        return frames
