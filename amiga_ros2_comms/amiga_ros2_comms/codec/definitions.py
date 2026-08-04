#!/usr/bin/env python3
"""Shared wire vocabulary for the coordination codec.

Every tag, enum, sentinel and quantization constant lives here and nowhere else.
Both the encoder and the decoder import from this module, which is the only
reason they cannot drift apart. A firmware or non-Python peer implementing the
same protocol should treat this file as the normative table.

Pure data. No ROS, no serial, no I/O.
"""

from enum import IntEnum
from typing import Iterable

# --------------------------------------------------------------------------
# Framing-independent wire conventions
# --------------------------------------------------------------------------

# Big-endian / network order, everywhere, no exceptions. Note this deliberately
# differs from the little-endian link-stats header in lora/framing.py: that
# header is dictated by the modem firmware contract, while this vocabulary is
# ours to define and network order is the portable default for a protocol other
# implementations will have to read.
BYTE_ORDER = ">"

# tag:1, src:1, seq:2
HEADER_FORMAT = BYTE_ORDER + "BBH"
HEADER_BYTES = 4

# Budget for one encoded message. Every built type must fit in a single LoRa
# payload -- there is no fragmentation at this layer and never will be, so a
# message that does not fit is a design error rather than something to split.
# The bridge's own max_payload_bytes (200 by default) is the transport ceiling;
# this much tighter number is the design rule that keeps messages single-packet
# even at a high spreading factor.
DEFAULT_MAX_PAYLOAD_BYTES = 50


class MessageType(IntEnum):
    """The tag byte. Values are frozen once shipped -- never renumber."""

    HEARTBEAT = 0x01
    TASK_ANNOUNCE = 0x02
    BID = 0x03
    GRANT = 0x04
    ACK = 0x05
    HAZARD = 0x06
    # Claimed but unbuilt. A free-text message cannot be bounded to one packet,
    # so it needs fragmentation, which belongs to the reliability layer that
    # does not exist yet. The tag is reserved now so nothing else takes 0x07 in
    # the meantime, and so a peer that gains it early is rejected as
    # "unsupported" rather than "unknown".
    FREEFORM = 0x07


#: Tags that are allocated but not implemented. Encoding one is refused;
#: decoding one is a distinct error from an unallocated tag.
RESERVED_TYPES = frozenset({MessageType.FREEFORM})


class Capability(IntEnum):
    """What a robot can do.

    Values are *bit indices* into a 16-bit ``cap_mask``, not mask values. That
    is what lets HEARTBEAT advertise a set in 2 bytes while TASK_ANNOUNCE names
    a single requirement in 1, with a one-line test between them::

        has_capability(heartbeat.cap_mask, announce.req_capability)

    Adding a capability means appending an index, never renumbering one.
    """

    DRIVE = 0
    INSPECT = 1
    SPRAY = 2
    HARVEST = 3
    MANIPULATE = 4
    TRANSPORT = 5
    SURVEY = 6
    CHARGE_HOST = 7
    RELAY = 8


#: Width of ``cap_mask`` in bits. Bounds every legal Capability index.
CAP_MASK_BITS = 16


class ReasonCode(IntEnum):
    """Why a task is being announced. Diagnostics and operator display."""

    UNSPECIFIED = 0
    OPERATOR_REQUEST = 1
    SCHEDULED = 2
    BATTERY_LOW = 3
    TASK_FAILED = 4
    CAPABILITY_MISSING = 5
    HAZARD_BLOCKED = 6
    PREEMPTED = 7
    TIMEOUT = 8


class HazardClass(IntEnum):
    """What kind of thing is in the way."""

    UNKNOWN = 0
    OBSTACLE = 1
    HUMAN = 2
    ANIMAL = 3
    TERRAIN = 4
    EQUIPMENT = 5
    DISABLED_ROBOT = 6
    CHEMICAL = 7
    FIRE = 8


# --------------------------------------------------------------------------
# Sentinels
# --------------------------------------------------------------------------

#: ``cur_task``/``task_id`` of 0 means "none" -- idle, or a task nobody owns.
TASK_NONE = 0

#: Robot ID 0 means "no robot". Real IDs are 1..255. The codec packs whatever it
#: is given; this is a fleet convention, not something the codec enforces.
ROBOT_ID_NONE = 0


# --------------------------------------------------------------------------
# Field ranges and quantization
# --------------------------------------------------------------------------

# Sender ID and sequence number. (src, seq) is the globally unique message ID.
# The codec packs and unpacks them and does nothing else with them -- dedup,
# ACK tracking and retransmit are all the reliability layer's business.
SRC_MAX = 0xFF
SEQ_MAX = 0xFFFF

# Local orchard grid indices, never lat/lon. Unsigned to match the
# ROW_INDEX/COL_INDEX convention already used by amiga_interfaces/GetTreeInfo,
# which indexes rows and columns from a corner origin. That service uses uint8;
# 16 bits here is deliberate headroom, not a different coordinate system.
GRID_MAX = 0xFFFF

TASK_ID_MAX = 0xFFFF

#: Battery charge, whole percent. 0..100, not 0..255 -- a percent that can read
#: 200 is a percent nobody can sanity-check.
BATTERY_MAX = 100

#: Detection confidence, whole percent, same reasoning as battery.
CONFIDENCE_MAX = 100

#: Task priority. Unitless, higher is more urgent. Comparison only; the codec
#: assigns no meaning to any particular value.
PRIORITY_MAX = 0xFF

#: Bid cost. Unitless, *lower is better*. Whatever scalar the bidder minimises
#: (time, energy, detour); only comparable between bids on the same task.
COST_MAX = 0xFF

#: Hazard radius, in grid cells -- same unit as grid_row/grid_col, so a hazard
#: is a disc in the grid the coordinates already live in.
RADIUS_MAX = 0xFF

# ETA carried in one byte. At 1 s/LSB that would cap at 255 s, which is under
# five minutes and shorter than a great many orchard traverses -- a robot would
# have to either lie or overflow. 4 s/LSB buys 17 minutes for the same byte, and
# 4 s of slop on an ETA that size is not a number anyone was going to act on.
# Encoding rounds half-up; worst-case error is ETA_RESOLUTION_S / 2 = 2 s.
ETA_RESOLUTION_S = 4
ETA_MAX_S = 0xFF * ETA_RESOLUTION_S  # 1020 s == 17 min

# TTL gets two bytes, so it needs no coarsening: 1 s/LSB reaches 18 hours and
# round-trips exactly. The asymmetry with eta_s is a consequence of field width,
# not of the two meaning different things.
TTL_RESOLUTION_S = 1
TTL_MAX_S = 0xFFFF  # 65535 s == 18h 12m


# --------------------------------------------------------------------------
# Capability mask helpers
# --------------------------------------------------------------------------


def cap_mask(*capabilities: Iterable[int]) -> int:
    """Build a ``cap_mask`` from Capability members.

    >>> cap_mask(Capability.DRIVE, Capability.SPRAY) == 0b101
    True
    """
    mask = 0
    for capability in capabilities:
        index = int(capability)
        if not 0 <= index < CAP_MASK_BITS:
            raise ValueError(f"capability index {index} outside 0..{CAP_MASK_BITS - 1}")
        mask |= 1 << index
    return mask


def has_capability(mask: int, capability: int) -> bool:
    """True if ``mask`` advertises ``capability``.

    This is the whole point of capabilities being bit indices: it is the one
    operation the arbiter needs, and it is a single shift.
    """
    index = int(capability)
    if not 0 <= index < CAP_MASK_BITS:
        raise ValueError(f"capability index {index} outside 0..{CAP_MASK_BITS - 1}")
    return bool(mask & (1 << index))


def capabilities_in(mask: int) -> "list[Capability]":
    """Every known Capability present in ``mask``. Unknown bits are ignored."""
    return [c for c in Capability if mask & (1 << int(c))]
