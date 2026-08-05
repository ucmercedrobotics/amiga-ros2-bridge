#!/usr/bin/env python3
"""LoRa time-on-air, derived from the modem settings.

Semtech's formula (SX1276 datasheet 4.1.1.7, AN1200.13). Pure arithmetic: no
ROS, no serial port, no radio. Two things use it.

The simulator holds a frame in the air for as long as the real modem would, so
sim traffic is paced in hundreds of milliseconds rather than microseconds and
concurrent senders actually collide.

:func:`max_payload_for_dwell` answers the question docs/lora_frame_contract.md
had to leave open: how many payload bytes fit inside the FCC dwell budget at a
given spreading factor. That number, not the 255-byte length field and not the
radio FIFO, is what really caps ``max_payload_bytes``.
"""

import math
from dataclasses import dataclass

# FCC Part 15.247 hybrid-mode dwell limit: a single transmission on one channel
# may not exceed 400 ms. This is the binding constraint on payload size in the
# US 915 MHz ISM band, and it tightens fast as the spreading factor goes up.
DWELL_LIMIT_SEC = 0.4

# The frame's length byte caps a body at 255, so no payload can exceed it.
PAYLOAD_CEILING_BYTES = 255


@dataclass(frozen=True)
class RadioConfig:
    """Modem settings. Defaults are a common SF7/125 kHz starting point."""

    spreading_factor: int = 7
    bandwidth_hz: int = 125_000
    # Denominator of the 4/N coding rate, so 5..8 for 4/5..4/8.
    coding_rate: int = 5
    preamble_symbols: int = 8
    explicit_header: bool = True
    payload_crc: bool = True

    def __post_init__(self):
        if not 6 <= self.spreading_factor <= 12:
            raise ValueError(
                f"spreading_factor must be 6..12, got {self.spreading_factor}"
            )
        if self.bandwidth_hz <= 0:
            raise ValueError(f"bandwidth_hz must be positive, got {self.bandwidth_hz}")
        if not 5 <= self.coding_rate <= 8:
            raise ValueError(
                f"coding_rate is the 4/N denominator and must be 5..8, "
                f"got {self.coding_rate}"
            )
        if self.preamble_symbols < 0:
            raise ValueError(
                f"preamble_symbols must be >= 0, got {self.preamble_symbols}"
            )

    def describe(self) -> str:
        return (
            f"SF{self.spreading_factor}/BW{self.bandwidth_hz // 1000}k/"
            f"CR4-{self.coding_rate}"
        )


DEFAULT_RADIO = RadioConfig()


def symbol_time_sec(radio: RadioConfig = DEFAULT_RADIO) -> float:
    """Duration of one chirp symbol."""
    return (1 << radio.spreading_factor) / radio.bandwidth_hz


def low_data_rate_optimize(radio: RadioConfig = DEFAULT_RADIO) -> bool:
    """Whether the modem must enable LDRO.

    Mandatory once a symbol runs longer than 16 ms (SF11 and SF12 at 125 kHz),
    where crystal drift across a single symbol would break demodulation. It
    costs throughput, which is why it belongs in the airtime calculation rather
    than being ignored.
    """
    return symbol_time_sec(radio) > 0.016


def preamble_time_sec(radio: RadioConfig = DEFAULT_RADIO) -> float:
    """Preamble duration, including the 4.25-symbol sync word."""
    return (radio.preamble_symbols + 4.25) * symbol_time_sec(radio)


def payload_symbols(payload_bytes: int, radio: RadioConfig = DEFAULT_RADIO) -> int:
    """Number of symbols the payload occupies, after coding and padding."""
    if payload_bytes < 0:
        raise ValueError(f"payload_bytes must be >= 0, got {payload_bytes}")
    sf = radio.spreading_factor
    de = 1 if low_data_rate_optimize(radio) else 0
    numerator = (
        8 * payload_bytes
        - 4 * sf
        + 28
        + (16 if radio.payload_crc else 0)
        - (0 if radio.explicit_header else 20)
    )
    denominator = 4 * (sf - 2 * de)
    return 8 + max(math.ceil(numerator / denominator) * radio.coding_rate, 0)


def airtime_sec(payload_bytes: int, radio: RadioConfig = DEFAULT_RADIO) -> float:
    """Total time on air for one frame carrying ``payload_bytes``."""
    return preamble_time_sec(radio) + payload_symbols(
        payload_bytes, radio
    ) * symbol_time_sec(radio)


def max_payload_for_dwell(
    radio: RadioConfig = DEFAULT_RADIO,
    dwell_sec: float = DWELL_LIMIT_SEC,
    ceiling: int = PAYLOAD_CEILING_BYTES,
) -> int:
    """Largest payload whose airtime fits in ``dwell_sec``. 0 if none does.

    Searched rather than solved: the symbol count is a ceiling division, so the
    closed form needs care and this runs 256 times at startup, once.
    """
    best = 0
    for size in range(ceiling + 1):
        if airtime_sec(size, radio) > dwell_sec:
            break
        best = size
    return best
