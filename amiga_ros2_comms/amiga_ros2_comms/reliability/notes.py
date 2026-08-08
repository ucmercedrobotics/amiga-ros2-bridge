#!/usr/bin/env python3
"""Cutting a note into packets, and putting the packets back into a note.

A *note* is free text about an announced task -- "soft ground past the gate,
the arm needs the long reach". It is the one thing in this vocabulary that does
not fit in a packet, so it is the one thing that gets split, and this is where
both halves of that live. The codec below stays what it was: one message, one
packet, no memory between calls.

**A note never carries load.** The TASK_ANNOUNCE it accompanies still holds the
whole machine-readable requirement -- capabilities, place, priority -- so losing
every fragment of a note costs a bidder some context and costs the auction
nothing. That is deliberate, and it is what makes the loss behaviour here
acceptable: fragments are broadcast, therefore unacknowledged, therefore a note
with a hole in it is simply dropped. There is no retransmit, no NACK and no
erasure coding, because the alternative -- text that can invalidate an auction
by going missing -- would be a worse design than not carrying text at all.

Bounded in the same two directions as dedup.py, for the same reason: this runs
forever on a robot, and a peer emitting fragments of notes it never finishes
must cost memory in proportion to the buffer rather than to its output.

Pure Python. No ROS, no lock, no clock of its own -- ``now`` is passed in, and
the caller holds whatever lock it holds.
"""

from collections import OrderedDict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from ..codec import (
    DEFAULT_MAX_PAYLOAD_BYTES,
    MAX_NOTE_FRAGMENTS,
    NOTE_HEADER_BYTES,
    NOTE_ID_MAX,
    TASK_ID_MAX,
    Freeform,
)

#: (src, task_id, note_id) -- one note in flight from one sender about one task.
FragmentKey = Tuple[int, int, int]

#: (src, task_id) -- how a *completed* note is looked up, because that is what
#: the receiver knows at the moment a TASK_ANNOUNCE arrives.
NoteKey = Tuple[int, int]


class NoteTooLong(ValueError):
    """The text needs more fragments than the cap allows.

    Raised by the splitter rather than silently truncating. Where to cut a
    sentence that is too long is a decision with meaning -- and the caller,
    which knows what the note was for, is the only thing in a position to make
    it.
    """


def text_bytes_per_fragment(max_payload_bytes: int = DEFAULT_MAX_PAYLOAD_BYTES) -> int:
    """How much UTF-8 one fragment can carry under a given payload budget.

    The number that makes notes expensive. At the 50-byte design budget it is
    41; under the 400 ms FCC dwell limit at SF10, where a whole payload is 24
    bytes, it is 15.
    """
    return int(max_payload_bytes) - NOTE_HEADER_BYTES


def split_note(
    task_id: int,
    note_id: int,
    text: str,
    max_payload_bytes: int = DEFAULT_MAX_PAYLOAD_BYTES,
    max_fragments: int = MAX_NOTE_FRAGMENTS,
) -> List[Freeform]:
    """Cut ``text`` into the fragments that carry it.

    ``src`` and ``seq`` are left at 0. The session owns sequence allocation and
    stamps them as each fragment goes out, exactly as it does for every other
    message -- a splitter that assigned its own would be a second allocator of
    the one number the dedup cache keys on.

    The cut is made on the *UTF-8 bytes*, not on characters, because the budget
    is a byte budget. A multi-byte character can therefore be split across two
    fragments, and only the reassembled whole is decodable text. That is why
    ``Freeform.text`` is bytes and why nothing tries to decode a fragment.
    """
    if not 0 < int(task_id) <= TASK_ID_MAX:
        raise ValueError(f"task_id must be 1..{TASK_ID_MAX}, got {task_id}")
    if not 0 <= int(note_id) <= NOTE_ID_MAX:
        raise ValueError(f"note_id must be 0..{NOTE_ID_MAX}, got {note_id}")
    if max_fragments < 1:
        raise ValueError(f"max_fragments must be >= 1, got {max_fragments}")

    per_fragment = text_bytes_per_fragment(max_payload_bytes)
    if per_fragment < 1:
        raise ValueError(
            f"max_payload_bytes={max_payload_bytes} leaves no room for text "
            f"after the {NOTE_HEADER_BYTES}-byte fragment header"
        )

    blob = text.encode("utf-8")
    if not blob:
        # A note that says nothing would still cost a packet and still make
        # every receiver run an interpretation over it.
        raise ValueError("a note needs text")

    chunks = [blob[i : i + per_fragment] for i in range(0, len(blob), per_fragment)]
    if len(chunks) > max_fragments:
        raise NoteTooLong(
            f"note is {len(blob)} bytes, which needs {len(chunks)} fragments "
            f"of {per_fragment}; the cap is {max_fragments} "
            f"({max_fragments * per_fragment} bytes)"
        )

    count = len(chunks)
    return [
        Freeform(
            src=0,
            seq=0,
            task_id=int(task_id),
            note_id=int(note_id),
            frag_index=index,
            frag_count=count,
            text=chunk,
        )
        for index, chunk in enumerate(chunks)
    ]


@dataclass(frozen=True)
class CompletedNote:
    """A note that arrived whole."""

    src: int
    task_id: int
    note_id: int
    text: str
    #: When the last missing fragment landed, on the caller's clock.
    completed_at: float


@dataclass
class _Partial:
    """Fragments of one note collected so far."""

    frag_count: int
    fragments: Dict[int, bytes]
    expires_at: float

    @property
    def complete(self) -> bool:
        return len(self.fragments) == self.frag_count


class NoteReassembler:
    """Collects fragments into notes, and holds finished notes briefly.

    Two caches, because there are two questions:

    * **Partials**, keyed ``(src, task_id, note_id)`` -- what is still arriving.
    * **Completed**, keyed ``(src, task_id)`` -- what arrived, kept for a short
      while so the TASK_ANNOUNCE that follows a note can find it.

    The second cache is the whole reason notes are sent *before* the
    announcement they annotate. A bidder computes fitness the instant an
    announcement decodes, so text that arrives afterwards has missed the only
    synchronous moment that matters. Sending the note first turns "did the text
    get here in time" into a cache lookup.

    Keyed on ``(src, task_id)`` without the note id, so a second note about the
    same task from the same sender replaces the first. Two notes in flight
    about one task is a sender correcting itself, and the correction is the one
    worth having.
    """

    def __init__(
        self,
        ttl_sec: float = 30.0,
        completed_ttl_sec: float = 60.0,
        max_partial: int = 16,
        max_completed: int = 32,
        max_fragments: int = MAX_NOTE_FRAGMENTS,
    ):
        if ttl_sec <= 0:
            raise ValueError(f"ttl_sec must be positive, got {ttl_sec}")
        if completed_ttl_sec <= 0:
            raise ValueError(
                f"completed_ttl_sec must be positive, got {completed_ttl_sec}"
            )
        if max_partial < 1:
            raise ValueError(f"max_partial must be >= 1, got {max_partial}")
        if max_completed < 1:
            raise ValueError(f"max_completed must be >= 1, got {max_completed}")
        if max_fragments < 1:
            raise ValueError(f"max_fragments must be >= 1, got {max_fragments}")

        self._ttl = float(ttl_sec)
        self._completed_ttl = float(completed_ttl_sec)
        self._max_partial = int(max_partial)
        self._max_completed = int(max_completed)
        self._max_fragments = int(max_fragments)

        # Both ordered by expiry, which -- since every insert uses a fixed TTL
        # and moves its key to the end -- is insertion order, so pruning is a
        # walk from the front that stops at the first live entry.
        self._partial: "OrderedDict[FragmentKey, _Partial]" = OrderedDict()
        self._completed: "OrderedDict[NoteKey, Tuple[CompletedNote, float]]" = (
            OrderedDict()
        )

        self._counters = {
            "note_fragments": 0,
            "note_fragments_duplicate": 0,
            "note_fragments_refused": 0,
            "notes_complete": 0,
            "notes_abandoned": 0,
            "notes_evicted": 0,
            "notes_undecodable": 0,
        }

    # -- inbound ------------------------------------------------------------

    def add(self, msg: Freeform, now: float) -> Optional[CompletedNote]:
        """Take one fragment. Returns the note if that was the last one.

        Never raises on a bad fragment -- this is one call away from the radio,
        and a peer with a bug in its splitter must not be able to stop us
        reassembling anybody else's notes. A refused fragment is counted and
        dropped.
        """
        self.prune(now)

        count = int(msg.frag_count)
        index = int(msg.frag_index)
        if not 1 <= count <= self._max_fragments or not 0 <= index < count:
            # Covers a count of zero, an index past the end, and a peer
            # configured to send longer notes than we will reassemble. The
            # codec passes the full byte range through so that this check can
            # be here, where the count can be reported, rather than in a decode
            # that would lose the fragments we already hold.
            self._counters["note_fragments_refused"] += 1
            return None

        self._counters["note_fragments"] += 1
        key: FragmentKey = (int(msg.src), int(msg.task_id), int(msg.note_id))
        partial = self._partial.get(key)

        if partial is not None and partial.frag_count != count:
            # The same note described with two different lengths. One of the
            # two is wrong and there is no way to tell which, so start again
            # from this fragment rather than assemble a note that is part one
            # sentence and part another.
            self._counters["note_fragments_refused"] += 1
            partial = None

        if partial is None:
            partial = _Partial(
                frag_count=count, fragments={}, expires_at=now + self._ttl
            )
            self._partial[key] = partial
        elif index in partial.fragments:
            self._counters["note_fragments_duplicate"] += 1

        partial.fragments[index] = bytes(msg.text)
        partial.expires_at = now + self._ttl
        self._partial.move_to_end(key)

        while len(self._partial) > self._max_partial:
            self._partial.popitem(last=False)
            self._counters["notes_evicted"] += 1

        if not partial.complete:
            return None

        del self._partial[key]
        blob = b"".join(partial.fragments[i] for i in range(count))
        try:
            text = blob.decode("utf-8")
        except UnicodeDecodeError:
            # Every fragment passed a CRC on the way in, so this is not line
            # noise -- it is a sender that split somebody's bytes wrongly, or
            # is not sending UTF-8 at all. Counted and dropped, because a note
            # repaired with replacement characters is a note whose meaning we
            # made up.
            self._counters["notes_undecodable"] += 1
            return None

        note = CompletedNote(
            src=int(msg.src),
            task_id=int(msg.task_id),
            note_id=int(msg.note_id),
            text=text,
            completed_at=now,
        )
        self._counters["notes_complete"] += 1
        self._remember(note, now)
        return note

    def _remember(self, note: CompletedNote, now: float) -> None:
        key: NoteKey = (note.src, note.task_id)
        self._completed[key] = (note, now + self._completed_ttl)
        self._completed.move_to_end(key)
        while len(self._completed) > self._max_completed:
            self._completed.popitem(last=False)

    def completed(self, src: int, task_id: int, now: float) -> Optional[CompletedNote]:
        """The note this sender most recently finished about this task, if any.

        A lookup rather than a take: an announcement repeats while its window
        is open, and each repeat has to see the same note as the first.
        """
        self.prune(now)
        entry = self._completed.get((int(src), int(task_id)))
        return entry[0] if entry is not None else None

    def prune(self, now: float) -> None:
        """Drop partials that ran out of time, and completed notes past TTL."""
        for key, partial in list(self._partial.items()):
            if partial.expires_at > now:
                break
            del self._partial[key]
            # Distinct from an eviction: this note had time and never
            # completed, which means fragments were lost on the air. It is the
            # number the loss experiment is actually about.
            self._counters["notes_abandoned"] += 1

        for key, (_, expiry) in list(self._completed.items()):
            if expiry > now:
                break
            del self._completed[key]

    def stats(self) -> dict:
        return {
            **self._counters,
            "notes_partial": len(self._partial),
            "notes_held": len(self._completed),
        }
