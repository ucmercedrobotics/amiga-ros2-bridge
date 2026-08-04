#!/usr/bin/env python3
"""Make bytes arrive, and arrive once.

The layer that turns a fire-and-forget radio into something task ownership can
be built on. It sits between the codec and the coordinator::

    firmware -> serial bridge -> codec -> reliability -> coordinator

and offers the layer above three things: send this reliably, send this
best-effort, and here is an inbound message you have not already been given.

Its one organising rule is that **reliability follows addressing**. A message
addressed to a single robot is ACKed and retransmitted until it lands or the
budget runs out; a message shouted at the whole fleet is sent once, because
there is no set of ACKs whose absence would mean anything. Broadcast
reliability is repetition by the coordinator plus dedup here.

What it will not do is decide anything -- no bidding, no arbitration, no
ownership, no LLM -- and it never fragments: every built message is one packet
by construction.

``ReliabilitySession`` is the engine and has no ROS in it, which is why the
acceptance tests drive whole retransmit campaigns over a lossy in-memory link
with no executor and no sleeping. ``ReliabilityNode`` is the thin wiring to
/lora/tx and /lora/rx.

Specified in ``docs/reliability_layer.md``.
"""

from .addressing import (
    BROADCAST,
    MODES,
    UNICAST,
    addressee_of,
    addressing_of,
    is_reliable,
    is_unicast,
    unicast_types,
)
from .dedup import DedupCache
from .session import (
    Outcome,
    ReliabilityError,
    ReliabilityParams,
    ReliabilitySession,
)

__all__ = [
    # Engine
    "ReliabilitySession",
    "ReliabilityParams",
    "Outcome",
    "ReliabilityError",
    "DedupCache",
    # Addressing
    "UNICAST",
    "BROADCAST",
    "MODES",
    "addressing_of",
    "addressee_of",
    "is_unicast",
    "is_reliable",
    "unicast_types",
]
