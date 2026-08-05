#!/usr/bin/env python3
"""Unit tests for the bounded ring buffer's overflow policies."""

import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.ring_queue import (  # noqa: E402
    DROP_NEWEST,
    DROP_OLDEST,
    BoundedRingQueue,
)


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
