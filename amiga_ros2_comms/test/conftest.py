#!/usr/bin/env python3
"""Virtual serial ports and the ROS context, so every test runs without a radio.

``VirtualSerialPair`` is the pure-Python equivalent of

    socat -d -d pty,raw,echo=0 pty,raw,echo=0

i.e. two device paths wired back to back. It is built in-process instead of
shelling out to socat so the suite has no external dependency and so the test
owns the relay's lifetime. ``VirtualSerialSink`` is the opposite: a port that
accepts bytes until its buffer fills and then blocks, which is how the
backpressure test wedges the writer thread.

Both use the same pty setup as the simulator's virtual medium, imported rather
than copied so a difference in termios flags cannot creep in between them.
"""

import os
import select
import sys
import threading

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.lora.virtual_medium import open_raw_pty  # noqa: E402


@pytest.fixture(scope="session", autouse=True)
def ros():
    """One rclpy context for the whole session.

    Session-scoped because several modules spin nodes, and initialising and
    shutting down rclpy repeatedly in one process invites flakiness.
    """
    import rclpy

    rclpy.init()
    yield
    rclpy.shutdown()


class VirtualSerialPair:
    """Two device paths that carry bytes to each other in both directions."""

    def __init__(self):
        self._master_a, self._slave_a, self.port_a = open_raw_pty()
        self._master_b, self._slave_b, self.port_b = open_raw_pty()
        # The slave fds stay open only to keep the pty nodes alive; the relay
        # never reads them, so it cannot steal bytes from the node under test.
        self._stop = threading.Event()
        self._relay = threading.Thread(target=self._pump, daemon=True)
        self._relay.start()

    def _pump(self):
        fds = [self._master_a, self._master_b]
        peer = {self._master_a: self._master_b, self._master_b: self._master_a}
        while not self._stop.is_set():
            try:
                readable, _, _ = select.select(fds, [], [], 0.05)
            except (OSError, ValueError):
                return
            for fd in readable:
                try:
                    chunk = os.read(fd, 4096)
                except OSError:
                    continue
                if chunk:
                    try:
                        os.write(peer[fd], chunk)
                    except OSError:
                        pass

    def close(self):
        self._stop.set()
        self._relay.join(timeout=2.0)
        for fd in (self._master_a, self._slave_a, self._master_b, self._slave_b):
            try:
                os.close(fd)
            except OSError:
                pass


class VirtualSerialSink:
    """A port nobody drains. Writes succeed until the pty buffer fills, then block."""

    def __init__(self):
        self._master, self._slave, self.port = open_raw_pty()

    def drain(self, limit=1 << 20):
        """Read whatever has piled up, so the writer can make progress again."""
        total = 0
        while total < limit:
            readable, _, _ = select.select([self._master], [], [], 0.05)
            if not readable:
                return total
            try:
                chunk = os.read(self._master, 4096)
            except OSError:
                return total
            if not chunk:
                return total
            total += len(chunk)
        return total

    def close(self):
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass


@pytest.fixture
def serial_pair():
    pair = VirtualSerialPair()
    yield pair
    pair.close()


@pytest.fixture
def serial_sink():
    sink = VirtualSerialSink()
    yield sink
    sink.close()
