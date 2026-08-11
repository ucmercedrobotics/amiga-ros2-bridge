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

It also classifies what it sends into two transmit-ordering classes, so that a
busy radio drains ACKs and GRANTs ahead of everything else (see priority.py).
That is ordering only -- nothing here paces, budgets or rate-limits the link.

What it will not do is decide anything -- no bidding, no arbitration, no
ownership, no LLM.

It does fragment exactly one thing: a **note**, free text bound to a task id
(notes.py). Notes are broadcast, therefore unACKed, therefore a note missing a
fragment is dropped rather than repaired -- which is only acceptable because the
announcement a note accompanies carries the machine-readable requirement by
itself, so losing the text costs a bidder context and costs the auction nothing.
Every other built message is still one packet by construction.

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
from .notes import (
    CompletedNote,
    NoteReassembler,
    NoteTooLong,
    split_note,
    text_bytes_per_fragment,
)
from .priority import (
    BULK,
    CLASS_NAME,
    CLASSES,
    URGENT,
    is_urgent,
    priority_of,
    urgent_types,
)
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
    # Transmit priority
    "URGENT",
    "BULK",
    "CLASSES",
    "CLASS_NAME",
    "priority_of",
    "is_urgent",
    "urgent_types",
    # Notes: the one thing here that spans several packets.
    "CompletedNote",
    "NoteReassembler",
    "NoteTooLong",
    "split_note",
    "text_bytes_per_fragment",
]
