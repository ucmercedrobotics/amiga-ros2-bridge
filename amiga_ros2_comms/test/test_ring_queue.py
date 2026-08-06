#!/usr/bin/env python3
"""Unit tests for the bounded ring buffers' overflow and ordering policies."""

import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.ring_queue import (  # noqa: E402
    DROP_NEWEST,
    DROP_OLDEST,
    BoundedPriorityRingQueue,
    BoundedRingQueue,
)

# Band indices, named here rather than imported: reliability/priority.py decides
# which *messages* land in which band, and this file is testing the container,
# which has no idea messages exist.
URGENT = 0
BULK = 1


def _drain(queue):
    items = []
    while True:
        item = queue.get(timeout=0)
        if item is None:
            return items
        items.append(item)


def test_drop_oldest_keeps_the_freshest_items():
    queue = BoundedRingQueue(3, DROP_OLDEST)
    for i in range(10):
        queue.put(i)
    assert _drain(queue) == [7, 8, 9]
    assert queue.dropped == 7


def test_drop_newest_keeps_the_earliest_items():
    queue = BoundedRingQueue(3, DROP_NEWEST)
    for i in range(10):
        queue.put(i)
    assert _drain(queue) == [0, 1, 2]
    assert queue.dropped == 7


def test_put_reports_whether_it_cost_an_item():
    queue = BoundedRingQueue(2, DROP_OLDEST)
    assert queue.put("a") is True
    assert queue.put("b") is True
    assert queue.put("c") is False


def test_put_never_blocks_even_when_permanently_full():
    queue = BoundedRingQueue(4, DROP_OLDEST)
    start = time.monotonic()
    for i in range(200_000):
        queue.put(i)
    elapsed = time.monotonic() - start
    # Generous bound: this is a "did it block" assertion, not a benchmark.
    assert elapsed < 5.0, f"200k non-blocking puts took {elapsed:.2f}s"


def test_get_blocks_until_an_item_arrives():
    queue = BoundedRingQueue(4)
    received = []

    def consumer():
        received.append(queue.get(timeout=2.0))

    thread = threading.Thread(target=consumer)
    thread.start()
    time.sleep(0.1)
    queue.put("late")
    thread.join(timeout=2.0)
    assert received == ["late"]


def test_get_returns_none_on_timeout():
    assert BoundedRingQueue(2).get(timeout=0.05) is None


def test_close_wakes_a_waiting_consumer():
    queue = BoundedRingQueue(2)
    result = []

    def consumer():
        result.append(queue.get(timeout=5.0))

    thread = threading.Thread(target=consumer)
    thread.start()
    time.sleep(0.1)
    start = time.monotonic()
    queue.close()
    thread.join(timeout=2.0)
    assert result == [None]
    assert time.monotonic() - start < 1.0, "close() must not wait out the timeout"


def test_rejects_nonsense_configuration():
    with pytest.raises(ValueError):
        BoundedRingQueue(0)
    with pytest.raises(ValueError):
        BoundedRingQueue(4, "drop_something")


def test_concurrent_producers_never_exceed_capacity():
    queue = BoundedRingQueue(8, DROP_OLDEST)

    def producer(base):
        for i in range(5000):
            queue.put((base, i))

    threads = [threading.Thread(target=producer, args=(n,)) for n in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert len(queue) <= 8
    assert len(_drain(queue)) <= 8


# ==========================================================================
# BoundedPriorityRingQueue
#
# The property under test throughout is the one the LoRa tx path depends on:
# a burst of bulk traffic must not be able to delay or displace an urgent
# frame, because on a half-duplex radio that is how an ACK misses the sender's
# retransmit timer.
# ==========================================================================


def test_urgent_overtakes_queued_bulk():
    queue = BoundedPriorityRingQueue(8, DROP_OLDEST)
    for i in range(5):
        queue.put(f"bulk{i}", BULK)
    queue.put("ack", URGENT)

    # Enqueued last, drained first, without displacing anything: there was room.
    assert _drain(queue) == ["ack", "bulk0", "bulk1", "bulk2", "bulk3", "bulk4"]
    assert queue.dropped == 0


def test_fifo_holds_within_a_band():
    """Bands are ordered against each other, never two items in one.

    A retransmit must not overtake the original it repeats, and fragments of one
    message must leave in the order they were split.
    """
    queue = BoundedPriorityRingQueue(8, DROP_OLDEST)
    for i in range(3):
        queue.put(f"u{i}", URGENT)
        queue.put(f"b{i}", BULK)
    assert _drain(queue) == ["u0", "u1", "u2", "b0", "b1", "b2"]


def test_urgent_evicts_bulk_when_full():
    queue = BoundedPriorityRingQueue(3, DROP_OLDEST)
    for i in range(3):
        queue.put(f"bulk{i}", BULK)
    assert len(queue) == 3

    assert queue.put("ack", URGENT) is False  # something had to go
    assert _drain(queue) == ["ack", "bulk1", "bulk2"]
    assert queue.dropped_by_priority == (0, 1)


def test_bulk_never_displaces_urgent():
    """The guarantee the whole class exists for."""
    queue = BoundedPriorityRingQueue(3, DROP_OLDEST)
    for i in range(3):
        queue.put(f"ack{i}", URGENT)

    assert queue.put("chatter", BULK) is False
    # The newcomer was refused; not one urgent item moved.
    assert _drain(queue) == ["ack0", "ack1", "ack2"]
    assert queue.dropped_by_priority == (0, 1)


def test_bulk_never_displaces_urgent_under_drop_newest_either():
    queue = BoundedPriorityRingQueue(2, DROP_NEWEST)
    queue.put("ack0", URGENT)
    queue.put("ack1", URGENT)

    assert queue.put("chatter", BULK) is False
    assert _drain(queue) == ["ack0", "ack1"]


def test_urgent_still_evicts_bulk_under_drop_newest():
    """Priority outranks the freshness policy.

    drop_newest decides *which* bulk item goes, not whether an urgent frame gets
    in. A queue that let a stale bulk backlog lock out an ACK would buy nothing.
    """
    queue = BoundedPriorityRingQueue(3, DROP_NEWEST)
    for i in range(3):
        queue.put(f"bulk{i}", BULK)

    assert queue.put("ack", URGENT) is False
    # The newest bulk item is the one that went, per the policy.
    assert _drain(queue) == ["ack", "bulk0", "bulk1"]
    assert queue.dropped_by_priority == (0, 1)


def test_same_band_overflow_follows_the_policy():
    oldest = BoundedPriorityRingQueue(3, DROP_OLDEST)
    newest = BoundedPriorityRingQueue(3, DROP_NEWEST)
    for i in range(5):
        oldest.put(i, BULK)
        newest.put(i, BULK)
    assert _drain(oldest) == [2, 3, 4]
    assert _drain(newest) == [0, 1, 2]


def test_unknown_band_clamps_toward_least_urgent():
    """A uint8 off the wire must not raise, and must not jump the queue."""
    queue = BoundedPriorityRingQueue(4, DROP_OLDEST)
    queue.put("stranger", 200)
    queue.put("ack", URGENT)
    assert _drain(queue) == ["ack", "stranger"]


def test_capacity_is_shared_across_bands():
    queue = BoundedPriorityRingQueue(4, DROP_OLDEST)
    for i in range(4):
        queue.put(i, BULK)
    assert len(queue) == 4
    queue.put("ack", URGENT)
    assert len(queue) == 4, "one budget across bands, not one each"


def test_priority_put_never_blocks_even_when_permanently_full():
    queue = BoundedPriorityRingQueue(4, DROP_OLDEST)
    start = time.monotonic()
    for i in range(200_000):
        queue.put(i, i % 2)
    elapsed = time.monotonic() - start
    assert elapsed < 5.0, f"200k non-blocking puts took {elapsed:.2f}s"


def test_priority_get_blocks_until_an_item_arrives():
    queue = BoundedPriorityRingQueue(4)
    received = []

    def consumer():
        received.append(queue.get(timeout=2.0))

    thread = threading.Thread(target=consumer)
    thread.start()
    time.sleep(0.1)
    queue.put("late", BULK)
    thread.join(timeout=2.0)
    assert received == ["late"]


def test_priority_close_wakes_a_waiting_consumer():
    queue = BoundedPriorityRingQueue(2)
    result = []

    def consumer():
        result.append(queue.get(timeout=5.0))

    thread = threading.Thread(target=consumer)
    thread.start()
    time.sleep(0.1)
    start = time.monotonic()
    queue.close()
    thread.join(timeout=2.0)
    assert result == [None]
    assert time.monotonic() - start < 1.0, "close() must not wait out the timeout"


def test_priority_rejects_nonsense_configuration():
    with pytest.raises(ValueError):
        BoundedPriorityRingQueue(0)
    with pytest.raises(ValueError):
        BoundedPriorityRingQueue(4, "drop_something")
    with pytest.raises(ValueError):
        BoundedPriorityRingQueue(4, DROP_OLDEST, levels=0)


def test_priority_concurrent_producers_never_exceed_capacity():
    queue = BoundedPriorityRingQueue(8, DROP_OLDEST)

    def producer(base):
        for i in range(5000):
            queue.put((base, i), base % 2)

    threads = [threading.Thread(target=producer, args=(n,)) for n in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert len(queue) <= 8
    assert len(_drain(queue)) <= 8
