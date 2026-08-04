#!/usr/bin/env python3
"""A bounded, never-blocking-on-put queue with an explicit overflow policy.

Transport-agnostic: any comms node in this package can use it. Producers call
:meth:`put`, which always returns immediately — that is the whole point, since
a behaviour-tree tick publishing to a radio must not be able to stall on it.
Consumers call :meth:`get`, which blocks.

What happens when a bounded queue is full is a design decision, not an
accident, so it is named rather than implied.
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

    def close(self) -> None:
        """Wake every waiting consumer so it can shut down."""
        with self._cv:
            self._closed = True
            self._cv.notify_all()
