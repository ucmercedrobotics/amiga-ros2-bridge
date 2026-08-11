#!/usr/bin/env python3
"""Unit tests for splitting a note and putting it back together.

No ROS, no radio, no session. The reassembler takes ``now`` as an argument, so
every TTL and eviction case runs in microseconds with nothing sleeping.

The property most of this file is really about: **a note that lost a fragment
is dropped, never partially delivered.** Half a sentence that reads like a whole
one is worse than no sentence at all, because the auction it informs cannot tell
the difference.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import (  # noqa: E402
    DEFAULT_MAX_PAYLOAD_BYTES,
    MAX_NOTE_FRAGMENTS,
    NOTE_HEADER_BYTES,
    Freeform,
    decode,
    encode,
)
from amiga_ros2_comms.reliability.notes import (  # noqa: E402
    NoteReassembler,
    NoteTooLong,
    split_note,
    text_bytes_per_fragment,
)

SENDER = 5
TASK = 77

SHORT = "soft ground"
LONG = "soft ground past the gate, the row is muddy after rain and the arm needs reach"
UNICODE = "muddy \U0001f327\U0001f327\U0001f327 after rain, mind the étagère "


def sent(text, src=SENDER, task_id=TASK, note_id=1, **kwargs):
    """Fragments as they would arrive: split, then stamped with a src."""
    fragments = split_note(task_id, note_id, text, **kwargs)
    for index, fragment in enumerate(fragments):
        fragment.src = src
        fragment.seq = index
    return fragments


def feed(reassembler, fragments, now=0.0):
    """Push fragments in. Returns the completed note, or None."""
    out = None
    for fragment in fragments:
        result = reassembler.add(fragment, now)
        if result is not None:
            out = result
    return out


# ==========================================================================
# Splitting
# ==========================================================================


def test_the_budget_is_the_payload_minus_the_fragment_header():
    assert text_bytes_per_fragment() == DEFAULT_MAX_PAYLOAD_BYTES - NOTE_HEADER_BYTES
    # The numbers this design is actually constrained by: 41 bytes of text per
    # packet at the design budget, 15 under the FCC dwell limit at SF10.
    assert text_bytes_per_fragment() == 41
    assert text_bytes_per_fragment(24) == 15


def test_a_short_note_is_one_fragment():
    fragments = split_note(TASK, 1, SHORT)
    assert len(fragments) == 1
    assert fragments[0].frag_count == 1
    assert fragments[0].frag_index == 0
    assert fragments[0].text == SHORT.encode("utf-8")


def test_every_fragment_carries_the_count_not_just_the_first():
    # On a lossy broadcast the fragment that would have told you the length is
    # exactly as likely to be lost as any other.
    fragments = split_note(TASK, 1, LONG)
    assert len(fragments) > 1
    assert {f.frag_count for f in fragments} == {len(fragments)}
    assert [f.frag_index for f in fragments] == list(range(len(fragments)))


def test_every_fragment_fits_one_packet():
    for payload_budget in (24, 32, DEFAULT_MAX_PAYLOAD_BYTES, 200):
        for fragment in split_note(
            TASK, 1, LONG * 2, max_payload_bytes=payload_budget, max_fragments=64
        ):
            assert len(encode(fragment, max_payload_bytes=payload_budget)) <= (
                payload_budget
            )


def test_the_splitter_does_not_assign_sequence_numbers():
    # The session owns seq allocation. A second allocator here would be a
    # second writer of the one number dedup keys on.
    for fragment in split_note(TASK, 1, LONG):
        assert (fragment.src, fragment.seq) == (0, 0)


def test_a_note_too_long_for_the_cap_is_refused_not_truncated():
    per = text_bytes_per_fragment()
    with pytest.raises(NoteTooLong):
        split_note(TASK, 1, "x" * (per * MAX_NOTE_FRAGMENTS + 1))
    # And the largest note that does fit is accepted, so the boundary is where
    # the message says it is.
    assert (
        len(split_note(TASK, 1, "x" * (per * MAX_NOTE_FRAGMENTS))) == MAX_NOTE_FRAGMENTS
    )


def test_an_empty_note_is_refused():
    with pytest.raises(ValueError):
        split_note(TASK, 1, "")


def test_a_payload_budget_with_no_room_for_text_is_refused():
    with pytest.raises(ValueError):
        split_note(TASK, 1, SHORT, max_payload_bytes=NOTE_HEADER_BYTES)


@pytest.mark.parametrize("task_id", [0, 65536, -1])
def test_a_task_id_off_the_wire_range_is_refused(task_id):
    with pytest.raises(ValueError):
        split_note(task_id, 1, SHORT)


# ==========================================================================
# Reassembly
# ==========================================================================


@pytest.mark.parametrize("text", [SHORT, LONG, UNICODE, UNICODE * 3, "x", "é"])
def test_a_note_round_trips_through_split_and_reassemble(text):
    note = feed(NoteReassembler(), sent(text, max_fragments=64))
    assert note is not None
    assert note.text == text
    assert (note.src, note.task_id) == (SENDER, TASK)


def test_a_multi_byte_character_split_across_fragments_survives():
    # The cut is made on UTF-8 bytes because the budget is a byte budget, so a
    # character straddling a boundary is normal rather than exceptional. Only
    # the reassembled whole is decodable, which is why Freeform.text is bytes.
    text = "\U0001f327" * 20
    fragments = sent(text, max_payload_bytes=NOTE_HEADER_BYTES + 3, max_fragments=64)
    assert len(fragments) > 1
    straddled = [f for f in fragments if len(f.text) % 4]
    assert straddled, "expected at least one fragment ending mid-character"
    assert feed(NoteReassembler(max_fragments=64), fragments).text == text


def test_fragments_arriving_out_of_order_reassemble():
    fragments = sent(LONG)
    assert len(fragments) > 1
    assert feed(NoteReassembler(), list(reversed(fragments))).text == LONG


def test_a_duplicate_fragment_is_counted_and_harmless():
    reassembler = NoteReassembler()
    fragments = sent(LONG)
    assert reassembler.add(fragments[0], 0.0) is None
    assert reassembler.add(fragments[0], 0.0) is None
    assert reassembler.stats()["note_fragments_duplicate"] == 1
    assert feed(reassembler, fragments[1:]).text == LONG


def test_nothing_is_delivered_until_the_last_fragment_lands():
    reassembler = NoteReassembler()
    fragments = sent(LONG)
    for fragment in fragments[:-1]:
        assert reassembler.add(fragment, 0.0) is None
    assert reassembler.add(fragments[-1], 0.0) is not None


@pytest.mark.parametrize("lost", [0, 1, -1])
def test_a_note_missing_a_fragment_is_dropped_never_half_delivered(lost):
    # The property the whole design rests on. There is no retransmit and no
    # erasure coding for a broadcast, so this is what loss looks like.
    reassembler = NoteReassembler(ttl_sec=5.0)
    fragments = sent(LONG)
    assert len(fragments) > 1
    kept = [f for i, f in enumerate(fragments) if i != lost % len(fragments)]

    assert feed(reassembler, kept) is None
    assert reassembler.completed(SENDER, TASK, 0.0) is None

    reassembler.prune(10.0)
    assert reassembler.stats()["notes_abandoned"] == 1
    assert reassembler.stats()["notes_complete"] == 0


def test_two_notes_about_one_task_do_not_interleave():
    # Without note_id, fragments of a correction and fragments of the thing it
    # corrects would assemble into a sentence neither of them said.
    reassembler = NoteReassembler()
    first = sent("the gate is open and the ground past it is soft", note_id=1)
    second = sent(
        "ignore that, the gate is locked after all and the key is gone", note_id=2
    )
    assert len(first) > 1 and len(second) > 1

    interleaved = [f for pair in zip(first, second) for f in pair]
    texts = {
        note.text
        for note in (reassembler.add(f, 0.0) for f in interleaved)
        if note is not None
    }
    assert texts == {
        "the gate is open and the ground past it is soft",
        "ignore that, the gate is locked after all and the key is gone",
    }


def test_notes_from_different_senders_about_one_task_stay_separate():
    reassembler = NoteReassembler()
    mine = sent("the row is muddy after rain", src=5, note_id=1)
    theirs = sent("the row is dry, I came through it", src=6, note_id=1)

    interleaved = [f for pair in zip(mine, theirs) for f in pair]
    for fragment in interleaved:
        reassembler.add(fragment, 0.0)

    assert reassembler.completed(5, TASK, 0.0).text == "the row is muddy after rain"
    assert (
        reassembler.completed(6, TASK, 0.0).text == "the row is dry, I came through it"
    )


def test_a_second_note_about_a_task_replaces_the_first_in_the_lookup():
    # A sender putting out a second note about one task is correcting itself,
    # and the correction is the one worth having.
    reassembler = NoteReassembler()
    feed(reassembler, sent("the gate is open", note_id=1))
    feed(reassembler, sent("ignore that, it is locked", note_id=2))
    assert reassembler.completed(SENDER, TASK, 0.0).text == "ignore that, it is locked"


def test_the_lookup_is_not_a_take():
    # An announcement repeats while its window is open and each repeat has to
    # see the same note as the first.
    reassembler = NoteReassembler()
    feed(reassembler, sent(SHORT))
    for _ in range(3):
        assert reassembler.completed(SENDER, TASK, 0.0).text == SHORT


# ==========================================================================
# Bounds: this runs forever on a robot
# ==========================================================================


def test_a_completed_note_expires_from_the_lookup():
    reassembler = NoteReassembler(completed_ttl_sec=30.0)
    feed(reassembler, sent(SHORT))
    assert reassembler.completed(SENDER, TASK, 29.0) is not None
    assert reassembler.completed(SENDER, TASK, 31.0) is None


def test_partial_notes_are_bounded_by_count_not_by_sender_output():
    reassembler = NoteReassembler(max_partial=4)
    for note_id in range(10):
        reassembler.add(sent(LONG, note_id=note_id)[0], 0.0)
    assert reassembler.stats()["notes_partial"] == 4
    assert reassembler.stats()["notes_evicted"] == 6


def test_completed_notes_are_bounded_by_count():
    reassembler = NoteReassembler(max_completed=3)
    for task_id in range(1, 11):
        feed(reassembler, sent(SHORT, task_id=task_id))
    assert reassembler.stats()["notes_held"] == 3


def test_an_eviction_is_distinct_from_running_out_of_time():
    # Different diagnoses: eviction means the buffer is too small for the
    # traffic, while abandonment means fragments were lost on the air -- and
    # the second is the number the loss experiment is about.
    reassembler = NoteReassembler(ttl_sec=5.0)
    reassembler.add(sent(LONG)[0], 0.0)
    reassembler.prune(10.0)
    stats = reassembler.stats()
    assert (stats["notes_abandoned"], stats["notes_evicted"]) == (1, 0)


def test_a_fragment_refreshes_the_ttl_of_the_note_it_belongs_to():
    reassembler = NoteReassembler(ttl_sec=5.0)
    fragments = sent(LONG)
    reassembler.add(fragments[0], 0.0)
    reassembler.add(fragments[1], 4.0)
    reassembler.prune(6.0)
    assert reassembler.stats()["notes_abandoned"] == 0


# ==========================================================================
# A peer with a bug in its splitter must not stop us reassembling anyone else's
# ==========================================================================


@pytest.mark.parametrize(
    "index, count",
    [(0, 0), (3, 2), (200, 1), (0, MAX_NOTE_FRAGMENTS + 1)],
)
def test_an_impossible_fragment_is_counted_and_dropped_not_raised(index, count):
    reassembler = NoteReassembler()
    fragment = Freeform(
        src=SENDER,
        seq=0,
        task_id=TASK,
        note_id=1,
        frag_index=index,
        frag_count=count,
        text=b"nonsense",
    )
    assert reassembler.add(fragment, 0.0) is None  # must not raise
    assert reassembler.stats()["note_fragments_refused"] == 1


def test_a_note_described_with_two_different_lengths_starts_again():
    # One of the two counts is wrong and there is no way to tell which, so
    # assembling them together would produce a note that is part one sentence
    # and part another.
    reassembler = NoteReassembler()
    fragments = sent(LONG)
    reassembler.add(fragments[0], 0.0)

    liar = Freeform(
        src=SENDER,
        seq=99,
        task_id=TASK,
        note_id=1,
        frag_index=0,
        frag_count=1,
        text=b"totally different",
    )
    note = reassembler.add(liar, 0.0)
    assert note is not None
    assert note.text == "totally different"
    assert reassembler.stats()["note_fragments_refused"] == 1


def test_a_note_that_is_not_utf8_is_dropped_rather_than_repaired():
    # Every fragment passed a CRC, so this is a sender that is not sending
    # UTF-8 rather than line noise. A note repaired with replacement characters
    # is a note whose meaning we made up.
    reassembler = NoteReassembler()
    fragment = Freeform(
        src=SENDER,
        seq=0,
        task_id=TASK,
        note_id=1,
        frag_index=0,
        frag_count=1,
        text=b"\xff\xfe\xfd",
    )
    assert reassembler.add(fragment, 0.0) is None
    assert reassembler.stats()["notes_undecodable"] == 1
    assert reassembler.completed(SENDER, TASK, 0.0) is None


def test_fragments_survive_the_wire_before_being_reassembled():
    # The other tests hand the reassembler objects. This one proves the whole
    # path: split, encode, decode, reassemble.
    reassembler = NoteReassembler()
    note = None
    for fragment in sent(LONG):
        note = reassembler.add(decode(encode(fragment)), 0.0) or note
    assert note.text == LONG


@pytest.mark.parametrize("ttl", [0.0, -1.0])
def test_a_reassembler_with_a_nonsense_bound_is_refused(ttl):
    with pytest.raises(ValueError):
        NoteReassembler(ttl_sec=ttl)
    with pytest.raises(ValueError):
        NoteReassembler(completed_ttl_sec=ttl)


@pytest.mark.parametrize("cap", [0, -1])
def test_a_reassembler_with_a_nonsense_capacity_is_refused(cap):
    with pytest.raises(ValueError):
        NoteReassembler(max_partial=cap)
    with pytest.raises(ValueError):
        NoteReassembler(max_completed=cap)
