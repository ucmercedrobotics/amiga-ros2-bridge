#!/usr/bin/env python3
"""Bounded, never-blocking-on-put queues with an explicit overflow policy.

Transport-agnostic: any comms node in this package can use them. Producers call
:meth:`put`, which always returns immediately — that is the whole point, since
a behaviour-tree tick publishing to a radio must not be able to stall on it.
Consumers call :meth:`get`, which blocks.

What happens when a bounded queue is full is a design decision, not an
accident, so it is named rather than implied.

Two queues, because the two directions want different things.
:class:`BoundedRingQueue` is FIFO and is what the inbound path uses: a received
frame has already cost its airtime, and reordering it would only confuse the
dedup cache above.  :class:`BoundedPriorityRingQueue` is the outbound one, where
a half-duplex radio means the queue is a real bottleneck and the order frames
leave in decides whether an ACK beats its own retransmit timer.

Neither knows what an item *is*. The priority queue sorts on a number the
producer hands it and interprets nothing.
"""

import threading
from collections import deque
from typing import Any, Optional

# A stale coordination message is worthless, so the default is to throw away the
# oldest item and keep the newest.
DROP_OLDEST = "drop_oldest"
# The opposite: refuse the new item and preserve what is already queued. Right
# when order matters more than freshness.
DROP_NEWEST = "drop_newest"

POLICIES = (DROP_OLDEST, DROP_NEWEST)


class BoundedRingQueue:
    """Fixed-capacity queue. Never blocks a producer; drops per ``policy``."""

    def __init__(self, capacity: int, policy: str = DROP_OLDEST):
        if capacity < 1:
            raise ValueError(f"capacity must be >= 1, got {capacity}")
        if policy not in POLICIES:
            raise ValueError(
                f"unknown overflow policy {policy!r}, want one of {POLICIES}"
            )
        self._capacity = capacity
        self._policy = policy
        self._items: deque = deque()
        self._cv = threading.Condition()
        self._closed = False
        self._dropped = 0

    @property
    def policy(self) -> str:
        return self._policy

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def dropped(self) -> int:
        """Total items discarded to overflow since construction."""
        with self._cv:
            return self._dropped

    def __len__(self) -> int:
        with self._cv:
            return len(self._items)

    def put(self, item: Any) -> bool:
        """Enqueue without ever blocking.

        Returns True if nothing was discarded, False if this call cost an item
        (either ``item`` itself under DROP_NEWEST, or the oldest one under
        DROP_OLDEST).
        """
        with self._cv:
            if self._closed:
                return False
            if len(self._items) >= self._capacity:
                self._dropped += 1
                if self._policy == DROP_NEWEST:
                    return False
                self._items.popleft()
                self._items.append(item)
                self._cv.notify()
                return False
            self._items.append(item)
            self._cv.notify()
            return True

    def get(self, timeout: Optional[float] = None) -> Any:
        """Pop the oldest item, waiting up to ``timeout``.

        Returns None on timeout or once the queue is closed and drained, which
        is how consumer threads learn to exit.
        """
        with self._cv:
            if not self._items and not self._closed:
                self._cv.wait(timeout)
            if self._items:
                return self._items.popleft()
            return None

    def snapshot(self) -> list:
        """The queued items, in the order :meth:`get` would return them.

        A copy, and non-destructive. For tests and diagnostics that want to see
        what survived an overflow without consuming it.
        """
        with self._cv:
            return list(self._items)

    def close(self) -> None:
        """Wake every waiting consumer so it can shut down."""
        with self._cv:
            self._closed = True
            self._cv.notify_all()


class BoundedPriorityRingQueue:
    """Fixed-capacity queue that drains urgent items first. Never blocks a producer.

    Same surface as :class:`BoundedRingQueue` — ``put`` cannot block, ``get``
    blocks, overflow is named rather than implied — with one addition: every
    item carries a band index, ``0`` being the most urgent, and :meth:`get`
    empties band 0 before it looks at band 1.

    **FIFO holds within a band.** Only the bands are ordered against each other,
    never two items in the same one. That is load-bearing rather than
    incidental: a retransmit must not overtake the original it is repeating, and
    the fragments of one message must arrive in the order they were split.

    **Capacity is shared.** One budget across all bands, not a budget each, so a
    quiet channel lets bulk traffic use the whole queue and a busy one lets
    urgent traffic take it back.

    **Priority outranks the overflow policy.** When the queue is full the victim
    is chosen from the *least urgent occupied* band and the policy only decides
    which item of that band goes:

    ===========================  ============================================
    incoming vs. what is queued  what happens
    ===========================  ============================================
    more urgent than some item   that band gives one up; the newcomer is kept
    same band                    ``DROP_OLDEST`` evicts, ``DROP_NEWEST``
                                 refuses the newcomer
    less urgent than everything  the newcomer is refused, whatever the policy
    ===========================  ============================================

    The last row is the guarantee worth stating on its own: **bulk can never
    displace urgent.** Without it a long enough burst of low-priority traffic
    would push out exactly the frames this queue exists to protect, and the
    ordering would buy nothing.

    Knows nothing about what an item is. It sorts on the number the producer
    hands it and interprets neither the number nor the item.
    """

    def __init__(self, capacity: int, policy: str = DROP_OLDEST, levels: int = 2):
        if capacity < 1:
            raise ValueError(f"capacity must be >= 1, got {capacity}")
        if policy not in POLICIES:
            raise ValueError(
                f"unknown overflow policy {policy!r}, want one of {POLICIES}"
            )
        if levels < 1:
            raise ValueError(f"levels must be >= 1, got {levels}")
        self._capacity = capacity
        self._policy = policy
        self._levels = levels
        #: One deque per band, most urgent first.
        self._bands = [deque() for _ in range(levels)]
        self._cv = threading.Condition()
        self._closed = False
        self._dropped = 0
        self._dropped_by_band = [0] * levels

    @property
    def policy(self) -> str:
        return self._policy

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def levels(self) -> int:
        return self._levels

    @property
    def dropped(self) -> int:
        """Total items discarded to overflow since construction."""
        with self._cv:
            return self._dropped

    @property
    def dropped_by_priority(self) -> "tuple":
        """Items discarded per band, indexed by band. Most urgent first.

        Split out because the interesting question under load is not how much
        was shed but *what* — a queue shedding bulk is working, and one shedding
        urgent is a queue that is too small.
        """
        with self._cv:
            return tuple(self._dropped_by_band)

    def __len__(self) -> int:
        with self._cv:
            return self._size()

    def _size(self) -> int:
        """Total items across every band. Caller holds the lock."""
        return sum(len(band) for band in self._bands)

    def _band_index(self, priority: int) -> int:
        """Clamp a producer-supplied number onto a band.

        Clamped rather than refused because the producer is a ROS subscription
        callback carrying a ``uint8`` from another process, and a number we did
        not expect must not be able to raise inside it. Anything above the known
        bands lands in the least urgent one, which is the safe direction: an
        unrecognised class is not one we have agreed may jump the queue.
        """
        return max(0, min(int(priority), self._levels - 1))

    def _lowest_occupied(self) -> Optional[int]:
        """Index of the least urgent band holding anything. Caller holds the lock."""
        for index in range(self._levels - 1, -1, -1):
            if self._bands[index]:
                return index
        return None

    def _discard(self, band_index: int) -> None:
        """Count one drop against ``band_index``. Caller holds the lock."""
        self._dropped += 1
        self._dropped_by_band[band_index] += 1

    def put(self, item: Any, priority: int) -> bool:
        """Enqueue at ``priority`` without ever blocking.

        Returns True if nothing was discarded, False if this call cost an item —
        either ``item`` itself or whatever it displaced. See the class docstring
        for which of those it is.
        """
        with self._cv:
            if self._closed:
                return False

            band = self._band_index(priority)
            if self._size() < self._capacity:
                self._bands[band].append(item)
                self._cv.notify()
                return True

            victim = self._lowest_occupied()
            if victim is None:
                # Full but every band empty means capacity < 1, which the
                # constructor already refused. Belt and braces.
                self._bands[band].append(item)
                self._cv.notify()
                return True

            if victim < band:
                # Everything queued is strictly more urgent than the newcomer.
                self._discard(band)
                return False

            if victim == band and self._policy == DROP_NEWEST:
                self._discard(band)
                return False

            # Take one from the least urgent occupied band. Which one depends on
            # the policy: drop_oldest sheds the stalest, drop_newest keeps the
            # run that is already queued intact.
            if self._policy == DROP_NEWEST:
                self._bands[victim].pop()
            else:
                self._bands[victim].popleft()
            self._discard(victim)

            self._bands[band].append(item)
            self._cv.notify()
            return False

    def get(self, timeout: Optional[float] = None) -> Any:
        """Pop the oldest item of the most urgent occupied band.

        Returns None on timeout or once the queue is closed and drained, which
        is how consumer threads learn to exit.
        """
        with self._cv:
            if not self._size() and not self._closed:
                self._cv.wait(timeout)
            for band in self._bands:
                if band:
                    return band.popleft()
            return None

    def snapshot(self) -> list:
        """The queued items, in the order :meth:`get` would return them.

        Flattened across bands, most urgent first, so it reads as the drain
        order rather than as the internal layout. A copy, and non-destructive.
        """
        with self._cv:
            return [item for band in self._bands for item in band]

    def close(self) -> None:
        """Wake every waiting consumer so it can shut down."""
        with self._cv:
            self._closed = True
            self._cv.notify_all()
