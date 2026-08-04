#!/usr/bin/env python3
"""Compact binary message vocabulary for robot-to-robot coordination.

A pure serialization library: ``encode(msg) -> bytes``, ``decode(bytes) -> msg``.
It sits above the LoRa bridge's opaque framing and below the reliability layer
that will eventually own IDs, ACKs, dedup, retransmit and fragmentation.

    firmware -> serial bridge -> codec -> reliability -> coordinator

No ROS, no serial, no radio -- import it and unit-test it anywhere.

The wire format is specified in ``docs/codec_message_vocabulary.md``.
"""

from .codec import (
    MAX_MESSAGE_BYTES,
    MESSAGE_SIZES,
    CodecError,
    FieldRangeError,
    PayloadTooLarge,
    ReservedMessageType,
    TrailingBytes,
    TruncatedMessage,
    UnknownMessageType,
    decode,
    encode,
)
from .definitions import (
    BATTERY_MAX,
    CAP_MASK_BITS,
    CONFIDENCE_MAX,
    COST_MAX,
    DEFAULT_MAX_PAYLOAD_BYTES,
    ETA_MAX_S,
    ETA_RESOLUTION_S,
    GRID_MAX,
    HEADER_BYTES,
    PRIORITY_MAX,
    RADIUS_MAX,
    RESERVED_TYPES,
    ROBOT_ID_NONE,
    SEQ_MAX,
    SRC_MAX,
    TASK_ID_MAX,
    TASK_NONE,
    TTL_MAX_S,
    Capability,
    HazardClass,
    MessageType,
    ReasonCode,
    cap_mask,
    capabilities_in,
    has_capability,
)
from .messages import (
    BUILT_MESSAGES,
    Ack,
    Bid,
    Grant,
    Hazard,
    Heartbeat,
    Message,
    TaskAnnounce,
)

__all__ = [
    # API
    "encode",
    "decode",
    # Messages
    "Message",
    "Heartbeat",
    "TaskAnnounce",
    "Bid",
    "Grant",
    "Ack",
    "Hazard",
    "BUILT_MESSAGES",
    # Vocabulary
    "MessageType",
    "Capability",
    "ReasonCode",
    "HazardClass",
    "RESERVED_TYPES",
    "TASK_NONE",
    "ROBOT_ID_NONE",
    # Bounds of the (src, seq) message-ID space the reliability layer owns.
    "SRC_MAX",
    "SEQ_MAX",
    "cap_mask",
    "has_capability",
    "capabilities_in",
    # Sizes and limits
    "HEADER_BYTES",
    "MESSAGE_SIZES",
    "MAX_MESSAGE_BYTES",
    "DEFAULT_MAX_PAYLOAD_BYTES",
    "BATTERY_MAX",
    "CONFIDENCE_MAX",
    "CAP_MASK_BITS",
    "ETA_MAX_S",
    "ETA_RESOLUTION_S",
    "TTL_MAX_S",
    # Field bounds the coordinator checks against before it builds a message,
    # so an out-of-range task or grid index is a rejected decision rather than
    # a FieldRangeError from inside the encoder.
    "GRID_MAX",
    "TASK_ID_MAX",
    "PRIORITY_MAX",
    "COST_MAX",
    "RADIUS_MAX",
    # Errors
    "CodecError",
    "UnknownMessageType",
    "ReservedMessageType",
    "TruncatedMessage",
    "TrailingBytes",
    "PayloadTooLarge",
    "FieldRangeError",
]
