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
    target_fields,
    target_of,
)
from .definitions import (
    BATTERY_MAX,
    CAP_MASK_BITS,
    CAPABILITY_BY_ELEMENT,
    COST_MAX,
    DEFAULT_MAX_PAYLOAD_BYTES,
    ETA_MAX_S,
    ETA_RESOLUTION_S,
    GPS_SCALE,
    HEADER_BYTES,
    INDEX_MAX,
    PRIORITY_MAX,
    RESERVED_TYPES,
    ROBOT_ID_NONE,
    SEQ_MAX,
    SRC_MAX,
    TARGET_WORD_MAX,
    TARGET_WORD_MIN,
    TASK_ID_MAX,
    TASK_NONE,
    XML_ELEMENT,
    Capability,
    MessageType,
    ReasonCode,
    Target,
    TargetKind,
    cap_mask,
    capabilities_in,
    has_capabilities,
    has_capability,
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
    "BUILT_MESSAGES",
    # Vocabulary
    "MessageType",
    "Capability",
    "ReasonCode",
    "RESERVED_TYPES",
    "TASK_NONE",
    "ROBOT_ID_NONE",
    # Bounds of the (src, seq) message-ID space the reliability layer owns.
    "SRC_MAX",
    "SEQ_MAX",
    "cap_mask",
    "has_capability",
    "has_capabilities",
    "capabilities_in",
    # The behaviour-tree side of the capability vocabulary: these are what let
    # mission XML be read into a cap_mask and a cap_mask be shown to a person.
    "XML_ELEMENT",
    "CAPABILITY_BY_ELEMENT",
    # Where work happens, in the terms the behaviour tree uses.
    "Target",
    "TargetKind",
    "target_of",
    "target_fields",
    "GPS_SCALE",
    "INDEX_MAX",
    "TARGET_WORD_MAX",
    "TARGET_WORD_MIN",
    # Sizes and limits
    "HEADER_BYTES",
    "MESSAGE_SIZES",
    "MAX_MESSAGE_BYTES",
    "DEFAULT_MAX_PAYLOAD_BYTES",
    "BATTERY_MAX",
    "CAP_MASK_BITS",
    "ETA_MAX_S",
    "ETA_RESOLUTION_S",
    # Field bounds the coordinator checks against before it builds a message,
    # so an out-of-range task or tree index is a rejected decision rather than
    # a FieldRangeError from inside the encoder.
    "TASK_ID_MAX",
    "PRIORITY_MAX",
    "COST_MAX",
    # Errors
    "CodecError",
    "UnknownMessageType",
    "ReservedMessageType",
    "TruncatedMessage",
    "TrailingBytes",
    "PayloadTooLarge",
    "FieldRangeError",
]
