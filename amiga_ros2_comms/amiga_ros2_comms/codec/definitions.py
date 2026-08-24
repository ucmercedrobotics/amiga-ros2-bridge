#!/usr/bin/env python3
"""Shared wire vocabulary for the coordination codec.

Every tag, enum, sentinel and quantization constant lives here and nowhere else.
Both the encoder and the decoder import from this module, which is the only
reason they cannot drift apart. A firmware or non-Python peer implementing the
same protocol should treat this file as the normative table.

**Where the vocabulary comes from.** Not from a guess about what an orchard
robot might do. ``Capability`` is the ActionGroup of
``amiga_ros2_behavior_tree/schemas/amiga_btcpp.xsd`` -- the schema the mission
planner writes against, the arbiter validates against, and ``bt_runner`` refuses
a mission for violating. ``Target`` is the set of places a BT leaf can actually
name. If the schema gains an action, this file gains a bit; if it does not, the
fleet cannot announce work nobody can execute.

Pure data. No ROS, no serial, no I/O -- reading the schema off disk belongs to
whoever configures a robot, not here.
"""

from dataclasses import dataclass
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
#
# FREEFORM does not break the rule, it obeys it per fragment: one *fragment* is
# one message is one packet, and the splitting happens a layer up in
# reliability/notes.py. Nothing here ever produces two packets from one call.
DEFAULT_MAX_PAYLOAD_BYTES = 50


class MessageType(IntEnum):
    """The tag byte. Values are frozen once shipped -- never renumber."""

    HEARTBEAT = 0x01
    TASK_ANNOUNCE = 0x02
    BID = 0x03
    GRANT = 0x04
    ACK = 0x05
    # 0x06 was HAZARD. Retired, and deliberately not reused: nothing in this
    # system ever detected or published a hazard, so the type described a
    # capability the robot does not have. Decoding 0x06 now reports an
    # unallocated tag, which is the truth.
    #
    # One fragment of a *note*: free text bound to a task_id. The only type
    # with a variable-length field, and the only one a sender may emit more
    # than one of per logical message. See messages.Freeform.
    FREEFORM = 0x07


#: Tags that are allocated but not implemented. Encoding one is refused;
#: decoding one is a distinct error from an unallocated tag.
#:
#: Empty now that FREEFORM is built. Kept, along with ReservedMessageType, as
#: the mechanism rather than as a list with one entry: the next tag we allocate
#: ahead of implementing it wants exactly this treatment, and a peer running
#: ahead of us should hear "not yet" instead of "never heard of it".
RESERVED_TYPES: frozenset = frozenset()


class Capability(IntEnum):
    """What a robot can do: one bit index per behaviour-tree action type.

    These are exactly the elements of ``<xs:group name="ActionGroup">`` in
    amiga_btcpp.xsd, in schema order. That is what makes a capability claim
    checkable rather than asserted -- a robot advertises the actions its own
    mission schema permits, and ``capabilities_from_xsd`` in the coordinator
    derives the mask from the installed file instead of a launch parameter
    somebody typed.

    ``DetectObject``, ``AssertTrue`` and ``CheckValue`` are registered in
    bt.cpp but commented out of the schema, so they are not here: a mission
    containing one would be rejected before it ever ran, which makes
    advertising it a lie.

    Values are *bit indices* into a 16-bit ``cap_mask``, not mask values. That
    is what lets HEARTBEAT advertise a whole action set in 2 bytes and
    TASK_ANNOUNCE state a whole requirement in 2 more, with a one-line test
    between them::

        has_capabilities(heartbeat.cap_mask, announce.req_cap_mask)

    Adding a capability means appending an index, never renumbering one.
    """

    MOVE_TO_TREE_ID = 0
    MOVE_TO_AISLE_HEAD = 1
    MOVE_TO_GPS_LOCATION = 2
    APPROACH_GPS_WAYPOINT = 3
    MOVE_TO_RELATIVE_LOCATION = 4
    ORIENT_ROBOT_HEADING = 5
    FOLLOW_PERSON = 6
    SAMPLE_LEAF = 7
    MOVE_ARM_TO_POSITION = 8
    WAIT = 9


#: Capability -> the XML element name it stands for. The mapping is explicit
#: rather than derived from the enum name by rule, because the schema is the
#: authority on spelling and a rule would quietly invent "MoveToGpsLocation".
XML_ELEMENT = {
    Capability.MOVE_TO_TREE_ID: "MoveToTreeID",
    Capability.MOVE_TO_AISLE_HEAD: "MoveToAisleHead",
    Capability.MOVE_TO_GPS_LOCATION: "MoveToGPSLocation",
    Capability.APPROACH_GPS_WAYPOINT: "ApproachGPSWaypoint",
    Capability.MOVE_TO_RELATIVE_LOCATION: "MoveToRelativeLocation",
    Capability.ORIENT_ROBOT_HEADING: "OrientRobotHeading",
    Capability.FOLLOW_PERSON: "FollowPerson",
    Capability.SAMPLE_LEAF: "SampleLeaf",
    Capability.MOVE_ARM_TO_POSITION: "MoveArmToPosition",
    Capability.WAIT: "Wait",
}

#: XML element name -> Capability. The direction the mission-XML side needs.
CAPABILITY_BY_ELEMENT = {name: cap for cap, name in XML_ELEMENT.items()}

#: Width of ``cap_mask`` in bits. Bounds every legal Capability index.
CAP_MASK_BITS = 16


class ReasonCode(IntEnum):
    """Why a task is being announced. Diagnostics and operator display.

    Every value is something this system can actually observe: a behaviour-tree
    node returned FAILURE, the battery is low, navigation found no route, this
    robot's schema has no such action, an operator asked, or something ran out
    of time. Nothing here is aspirational -- a code no code path can set is a
    code that only ever misleads whoever reads the log.

    This is also the *only* definition. The triage agent builds its label table
    from this enum rather than restating it; two hand-written tables is exactly
    how ``low_battery`` came to travel as ``OPERATOR_REQUEST``.
    """

    UNSPECIFIED = 0
    #: A BT node returned FAILURE and local replanning could not recover it.
    TASK_FAILED = 1
    BATTERY_LOW = 2
    #: No route, or the orchard model has no such tree. Navigation's answer.
    UNREACHABLE = 3
    #: This robot's mission schema does not contain the required action.
    CAPABILITY_MISSING = 4
    OPERATOR_REQUEST = 5
    TIMEOUT = 6


# --------------------------------------------------------------------------
# Sentinels
# --------------------------------------------------------------------------

#: ``cur_task``/``task_id`` of 0 means "none" -- idle, or a task nobody owns.
TASK_NONE = 0

#: Robot ID 0 means "no robot". Real IDs are 1..255. The codec packs whatever it
#: is given; this is a fleet convention, not something the codec enforces.
ROBOT_ID_NONE = 0


# --------------------------------------------------------------------------
# Where work happens
# --------------------------------------------------------------------------


class TargetKind(IntEnum):
    """How a place is named, matching how the behaviour tree names places.

    There is no single coordinate system here because the tree does not have
    one. ``MoveToTreeID`` names a tree index, ``MoveToAisleHead`` an aisle
    index, the two GPS actions a latitude and longitude -- and ``SampleLeaf``
    and ``FollowPerson`` name nothing at all, because they happen wherever the
    robot already is. GetTreeInfo converts between the first three on demand;
    flattening them into one at announce time would throw away the very thing
    the receiving robot needs to rebuild the action node.
    """

    #: Work with no place of its own. Only meaningful next to work that has one.
    NONE = 0
    #: ``a`` is a tree index -- GetTreeInfo TREE_INDEX, MoveToTreeID's ``id``.
    TREE = 1
    #: ``a`` is an aisle index -- GetTreeInfo AISLE_INDEX.
    AISLE = 2
    #: ``a``/``b`` are latitude/longitude, scaled by GPS_SCALE.
    GPS = 3


#: Degrees per LSB is 1/GPS_SCALE: 1e-7 deg, about 1.1 cm at the equator, which
#: is finer than any orchard waypoint needs and still leaves int32 covering the
#: full +/-180 range with room to spare.
GPS_SCALE = 10_000_000

#: Bounds for the two scaled target words. Signed, because longitude is.
TARGET_WORD_MAX = 2**31 - 1
TARGET_WORD_MIN = -(2**31)

#: Tree and aisle indices are unsigned and small. Bounded separately from the
#: word range so a negative tree index is refused at construction rather than
#: encoded happily and puzzled over later.
INDEX_MAX = 0xFFFF


@dataclass(frozen=True)
class Target:
    """Where a task happens, in the terms the behaviour tree uses.

    Two integer words behind a kind byte. Degrees are converted at the boundary
    -- ``Target.gps()`` in, ``lat_deg``/``lon_deg`` out -- so everything from
    here down the stack is integers and the codec needs no fractional scaling.
    """

    kind: TargetKind
    #: tree index | aisle index | latitude * GPS_SCALE. Unused when NONE.
    a: int = 0
    #: longitude * GPS_SCALE for GPS. Unused otherwise.
    b: int = 0

    def __post_init__(self):
        kind = TargetKind(int(self.kind))
        object.__setattr__(self, "kind", kind)
        for name in ("a", "b"):
            value = int(getattr(self, name))
            if not TARGET_WORD_MIN <= value <= TARGET_WORD_MAX:
                raise ValueError(
                    f"target.{name}={value} outside "
                    f"{TARGET_WORD_MIN}..{TARGET_WORD_MAX}"
                )
            object.__setattr__(self, name, value)
        if kind in (TargetKind.TREE, TargetKind.AISLE):
            if not 0 <= self.a <= INDEX_MAX:
                raise ValueError(f"{kind.name} index {self.a} outside 0..{INDEX_MAX}")
            if self.b:
                raise ValueError(f"{kind.name} target does not use b, got {self.b}")
        elif kind is TargetKind.NONE and (self.a or self.b):
            raise ValueError("a NONE target carries no coordinates")

    # -- constructors -------------------------------------------------------

    @classmethod
    def none(cls) -> "Target":
        return cls(TargetKind.NONE)

    @classmethod
    def tree(cls, tree_index: int) -> "Target":
        return cls(TargetKind.TREE, int(tree_index))

    @classmethod
    def aisle(cls, aisle_index: int) -> "Target":
        return cls(TargetKind.AISLE, int(aisle_index))

    @classmethod
    def gps(cls, lat_deg: float, lon_deg: float) -> "Target":
        if not -90.0 <= float(lat_deg) <= 90.0:
            raise ValueError(f"latitude {lat_deg} outside -90..90")
        if not -180.0 <= float(lon_deg) <= 180.0:
            raise ValueError(f"longitude {lon_deg} outside -180..180")
        return cls(
            TargetKind.GPS,
            round(float(lat_deg) * GPS_SCALE),
            round(float(lon_deg) * GPS_SCALE),
        )

    # -- accessors ----------------------------------------------------------

    @property
    def lat_deg(self) -> float:
        return self.a / GPS_SCALE

    @property
    def lon_deg(self) -> float:
        return self.b / GPS_SCALE

    @property
    def placed(self) -> bool:
        """Whether this target names somewhere another robot could go."""
        return self.kind is not TargetKind.NONE

    def __str__(self) -> str:
        if self.kind is TargetKind.TREE:
            return f"tree {self.a}"
        if self.kind is TargetKind.AISLE:
            return f"aisle {self.a}"
        if self.kind is TargetKind.GPS:
            return f"({self.lat_deg:.7f}, {self.lon_deg:.7f})"
        return "here"


# --------------------------------------------------------------------------
# Field ranges and quantization
# --------------------------------------------------------------------------

# Sender ID and sequence number. (src, seq) is the globally unique message ID.
# The codec packs and unpacks them and does nothing else with them -- dedup,
# ACK tracking and retransmit are all the reliability layer's business.
SRC_MAX = 0xFF
SEQ_MAX = 0xFFFF

TASK_ID_MAX = 0xFFFF

#: Battery charge, whole percent. 0..100, not 0..255 -- a percent that can read
#: 200 is a percent nobody can sanity-check.
BATTERY_MAX = 100

#: Task priority. Unitless, higher is more urgent. Comparison only; the codec
#: assigns no meaning to any particular value.
PRIORITY_MAX = 0xFF

#: Bid cost. Unitless, *lower is better*. Whatever scalar the bidder minimises
#: (time, energy, detour); only comparable between bids on the same task.
COST_MAX = 0xFF

# ETA carried in one byte. At 1 s/LSB that would cap at 255 s, which is under
# five minutes and shorter than a great many orchard traverses -- a robot would
# have to either lie or overflow. 4 s/LSB buys 17 minutes for the same byte, and
# 4 s of slop on an ETA that size is not a number anyone was going to act on.
# Encoding rounds half-up; worst-case error is ETA_RESOLUTION_S / 2 = 2 s.
ETA_RESOLUTION_S = 4
ETA_MAX_S = 0xFF * ETA_RESOLUTION_S  # 1020 s == 17 min


# --------------------------------------------------------------------------
# Notes: free text bound to a task
# --------------------------------------------------------------------------

#: Distinguishes two notes about the same task from the same sender, so their
#: fragments cannot interleave into one plausible-looking sentence. Wraps; the
#: reassembler only needs consecutive notes to differ, not all-time uniqueness.
NOTE_ID_MAX = 0xFF

#: Fragment index and count both live in one byte, but the real cap is
#: MAX_NOTE_FRAGMENTS below.
FRAG_MAX = 0xFF

#: How many fragments one note may be split into.
#:
#: A bound, not an airtime budget -- nothing here paces or meters the link. It
#: exists so a single note cannot become a hundred-packet monologue that starves
#: an auction it was supposed to inform, and so a receiver's reassembly buffer
#: has a size that can be stated. At the 50-byte design budget a fragment
#: carries 41 text bytes, so 8 fragments is about 328 characters: a couple of
#: sentences, which is the length the examples driving this feature actually
#: are.
MAX_NOTE_FRAGMENTS = 8


# --------------------------------------------------------------------------
# Capability mask helpers
# --------------------------------------------------------------------------


def cap_mask(*capabilities: Iterable[int]) -> int:
    """Build a ``cap_mask`` from Capability members.

    >>> cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF) == 0b10000001
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


def has_capabilities(mask: int, required: int) -> bool:
    """True if ``mask`` advertises *every* action in ``required``.

    A task is a behaviour-tree subtree and a subtree uses a set of actions, not
    one: sampling a tree is ``MoveToTreeID`` *and* ``SampleLeaf``, which is
    exactly the pairing the arbiter's orphaned-SampleLeaf check enforces. A
    robot with the arm but no tree navigation would pass a single-action test
    and then be unable to place the work it just won.

    An empty requirement is satisfied by anything, which is the right reading:
    a task nobody needs a particular action for is a task anybody can take.
    """
    return (int(mask) & int(required)) == int(required)


def capabilities_in(mask: int) -> "list[Capability]":
    """Every known Capability present in ``mask``. Unknown bits are ignored."""
    return [c for c in Capability if mask & (1 << int(c))]
