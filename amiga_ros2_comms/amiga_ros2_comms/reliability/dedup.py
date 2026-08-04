#!/usr/bin/env python3
"""Remembers which ``(src, seq)`` have been seen, so each is delivered once.

Duplicates are not an anomaly here, they are the normal cost of the design.
Every reliable send may retransmit; every broadcast is re-emitted by the
coordinator because that is the only reliability a broadcast can have. Both
mechanisms deliberately put the same message on the air more than once, and
both are only safe because something downstream collapses the copies. This is
that something.

Bounded in two directions, because it runs forever on a robot:

* **TTL** -- an entry outlives the longest retransmit campaign or app-level
  re-broadcast that could still produce a copy, and no longer.
* **Capacity** -- a hard entry count, so a peer emitting nonsense at speed
  costs memory in proportion to the cache, not to its output.

Seeing a key again refreshes its TTL. A sender still retransmitting is exactly
the case dedup exists for, and letting the entry age out from under an ongoing
campaign would deliver the tail of it a second time.

Pure Python. No ROS, no clock of its own -- ``now`` is passed in, which is what
lets the tests drive months of TTL behaviour in microseconds.
"""

from collections import OrderedDict
from typing import Hashable, Tuple

#: (src, seq): the globally unique message ID, per the codec's header.
Key = Tuple[int, int]


class DedupCache:
    """TTL'd, capacity-bounded set of message IDs."""

    def __init__(self, ttl_sec: float, max_entries: int):
        if ttl_sec <= 0:
            raise ValueError(f"ttl_sec must be positive, got {ttl_sec}")
        if max_entries < 1:
            raise ValueError(f"max_entries must be >= 1, got {max_entries}")
        self._ttl = float(ttl_sec)
        self._max = int(max_entries)
        # Ordered by expiry, which -- since every insert and refresh uses the
        # same fixed TTL and moves the key to the end -- is also insertion
        # order. That is what makes pruning a walk from the front that stops at
        # the first live entry, rather than a scan of the whole cache.
        self._seen: "OrderedDict[Key, float]" = OrderedDict()
        self._duplicates = 0
        self._evicted = 0

    @property
    def ttl_sec(self) -> float:
        return self._ttl

    @property
    def max_entries(self) -> int:
        return self._max

    @property
    def duplicates(self) -> int:
        """Total suppressed repeats since construction."""
        return self._duplicates

    @property
    def evicted(self) -> int:
        """Entries dropped to the capacity bound, not to their TTL.

        Worth watching separately: TTL expiry is the cache working, while
        eviction means it is too small for the traffic and a duplicate old
        enough to have been pushed out would be delivered twice.
        """
        return self._evicted

    def __len__(self) -> int:
        return len(self._seen)

    def __contains__(self, key: Hashable) -> bool:
        return key in self._seen

    def prune(self, now: float) -> int:
        """Drop everything past its TTL. Returns how many went."""
        dropped = 0
        for key, expiry in list(self._seen.items()):
            if expiry > now:
                # Ordered by expiry, so the first live entry ends the walk.
                break
            del self._seen[key]
            dropped += 1
        return dropped

    def first_sight(self, key: Key, now: float) -> bool:
        """Record ``key`` and report whether this is the first time we saw it.

        True means deliver it; False means a copy already went up and this one
        must not. Refreshes the TTL either way.
        """
        self.prune(now)

        known = key in self._seen
        if known:
            self._duplicates += 1
        self._seen[key] = now + self._ttl
        self._seen.move_to_end(key)

        while len(self._seen) > self._max:
            # Oldest expiry first: the entry closest to being useless anyway.
            self._seen.popitem(last=False)
            self._evicted += 1

        return not known

    def stats(self) -> dict:
        return {
            "dedup_size": len(self._seen),
            "dedup_duplicates": self._duplicates,
            "dedup_evicted": self._evicted,
        }
