#!/usr/bin/env python3
"""Tests for the virtual LoRa medium. No ROS: these drive the ports directly.

The medium only claims two things a socat pty pair does not: broadcast minus the
sender, and real time on air. Everything here asserts one of those two, or the
lifecycle around them.
"""

import os
import threading
import time

import pytest
import serial

from amiga_ros2_comms.lora.airtime import RadioConfig, airtime_sec
from amiga_ros2_comms.lora.framing import FrameParser, encode_frame
from amiga_ros2_comms.lora.virtual_medium import VirtualLoRaMedium

# Real 125 kHz timings, so airtime is hundreds of milliseconds: long enough to
# measure, short enough to keep the suite quick.
TEST_RADIO = RadioConfig(spreading_factor=7, bandwidth_hz=125_000)


class FakeRadio:
    """Stands in for a bridge: frames on the way out, parses on the way in."""

    def __init__(self, path):
        self.port = serial.Serial(path, 115200, timeout=0.05)
        self.parser = FrameParser()
        self.frames = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                chunk = self.port.read(max(self.port.in_waiting, 1))
            except Exception:
                return
            if not chunk:
                continue
            with self._lock:
                self.frames += self.parser.feed(chunk)

    def send(self, payload):
        self.port.write(encode_frame(payload))
        self.port.flush()

    def payloads(self):
        with self._lock:
            return [frame.payload for frame in self.frames]

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.port.close()


class Fleet:
    """A medium plus one FakeRadio per robot, torn down together."""

    def __init__(self, names, **kwargs):
        kwargs.setdefault("radio", TEST_RADIO)
        self.medium = VirtualLoRaMedium(names, **kwargs)
        self.radios = {
            name: FakeRadio(path) for name, path in self.medium.ports.items()
        }

    def __getitem__(self, name):
        return self.radios[name]

    def close(self):
        for radio in self.radios.values():
            radio.close()
        self.medium.close()


@pytest.fixture
def fleet_factory():
    made = []

    def build(names, **kwargs):
        fleet = Fleet(names, **kwargs)
        made.append(fleet)
        return fleet

    yield build
    for fleet in made:
        fleet.close()


def wait_until(predicate, timeout=5.0, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


# ----------------------------------------------------------------------
# Broadcast
# ----------------------------------------------------------------------


def test_a_transmission_reaches_every_other_radio_and_not_the_sender(fleet_factory):
    fleet = fleet_factory(["a", "b", "c"])
    payload = b"hello everyone"
    fleet["a"].send(payload)

    assert wait_until(lambda: fleet["b"].payloads() and fleet["c"].payloads())
    assert fleet["b"].payloads() == [payload]
    assert fleet["c"].payloads() == [payload]
    # A half-duplex radio does not hear itself. If it did, every robot would
    # process its own broadcasts and any dedup layer above would be masked.
    assert fleet["a"].payloads() == []
    assert fleet.medium.stats()["frames_delivered"] == 2


def test_payloads_cross_byte_identical(fleet_factory):
    fleet = fleet_factory(["a", "b"])
    blobs = [b"", b"\x00", b"\x00" * 30, bytes(range(200)), b"\xff" * 200]
    for blob in blobs:
        fleet["a"].send(blob)

    assert wait_until(lambda: len(fleet["b"].payloads()) == len(blobs), timeout=15.0)
    assert fleet["b"].payloads() == blobs
    assert fleet["b"].parser.stats.drops == 0


# ----------------------------------------------------------------------
# Time on air
# ----------------------------------------------------------------------


def test_delivery_waits_out_the_time_on_air(fleet_factory):
    """A frame must not arrive faster than the radio could have sent it.

    This is the whole reason for not using a plain pty pair: instant delivery
    lets timing bugs pass in sim that a 300 ms frame would expose on hardware.
    """
    fleet = fleet_factory(["a", "b"])
    payload = bytes(200)
    expected = airtime_sec(len(payload), TEST_RADIO)
    assert expected > 0.2, "the test radio should be slow enough to measure"

    start = time.monotonic()
    fleet["a"].send(payload)
    assert wait_until(lambda: fleet["b"].payloads(), timeout=5.0)
    elapsed = time.monotonic() - start

    assert elapsed >= expected * 0.9, (
        f"delivered in {elapsed * 1000:.0f} ms but {expected * 1000:.0f} ms of "
        "airtime was owed"
    )
    assert elapsed < expected + 0.5


def test_a_burst_from_one_radio_is_serialised(fleet_factory):
    """One transmitter finishes a frame before starting the next.

    The bridge hands frames to the port as fast as its writer thread drains, so
    the medium has to chain them rather than start them all at once.
    """
    fleet = fleet_factory(["a", "b"])
    payloads = [f"burst {i}".encode() for i in range(5)]
    for payload in payloads:
        fleet["a"].send(payload)

    assert wait_until(lambda: len(fleet["b"].payloads()) == len(payloads), timeout=10.0)
    assert fleet["b"].payloads() == payloads


def test_a_busy_modem_stops_draining_its_serial_port(fleet_factory):
    """Backpressure has to reach the host, or the bridge's whole design is untested.

    Real firmware mid-transmission is not reading its UART, so the host's write
    buffer fills and its writes block. A plain pty accepts everything instantly,
    which means the "never blocks upstream" guarantee the bridge exists to
    provide can only be exercised against a medium that pushes back.
    """
    fleet = fleet_factory(["a", "b"])
    payload = bytes(200)

    # A pty absorbs about 16 kB before it blocks a writer, so the burst has to
    # comfortably exceed that or it proves nothing: 200 frames is ~41 kB.
    burst = 200
    sent = []

    def flood():
        for _ in range(burst):
            try:
                fleet["a"].send(payload)
            except Exception:
                return
            sent.append(1)

    sender = threading.Thread(target=flood, daemon=True)
    sender.start()
    time.sleep(1.0)

    assert sender.is_alive(), (
        f"the writer pushed all {burst} frames without blocking; the modem is "
        "still draining its port while transmitting"
    )
    # One 4 kB read is ~20 frames, and each costs 318 ms of airtime, so a
    # second of wall clock cannot legitimately have consumed many more.
    assert fleet.medium.stats()["frames_sent"] <= 40, (
        "the modem consumed more airtime than a second of wall clock allows"
    )
    assert len(sent) < burst


# ----------------------------------------------------------------------
# Ports and lifecycle
# ----------------------------------------------------------------------


def test_ports_are_stable_symlinks_that_are_cleaned_up(tmp_path):
    directory = str(tmp_path / "lora")
    medium = VirtualLoRaMedium(["alpha", "beta"], symlink_dir=directory, radio=TEST_RADIO)
    try:
        # Launch files name the port before the pty exists, so the path has to
        # be predictable rather than whatever /dev/pts number came up.
        assert medium.ports == {
            "alpha": os.path.join(directory, "alpha"),
            "beta": os.path.join(directory, "beta"),
        }
        for path in medium.ports.values():
            assert os.path.islink(path)
            assert os.path.exists(os.path.realpath(path))
    finally:
        medium.close()

    # Left behind, a stale link would point at a recycled pts node and silently
    # wire the next run's robot to somebody else's port.
    for name in ("alpha", "beta"):
        assert not os.path.lexists(os.path.join(directory, name))


def test_a_stale_symlink_from_a_previous_run_is_replaced(tmp_path):
    directory = tmp_path / "lora"
    directory.mkdir()
    stale = directory / "alpha"
    stale.symlink_to("/dev/pts/999")

    medium = VirtualLoRaMedium(["alpha"], symlink_dir=str(directory), radio=TEST_RADIO)
    try:
        assert os.path.realpath(str(stale)).startswith("/dev/pts/")
        assert os.path.exists(os.path.realpath(str(stale)))
    finally:
        medium.close()


def test_duplicate_robot_names_are_rejected():
    with pytest.raises(ValueError):
        VirtualLoRaMedium(["a", "a"])
    with pytest.raises(ValueError):
        VirtualLoRaMedium([])


def test_the_medium_stays_idle_when_nobody_is_talking():
    """Same failure mode as the bridge's unplugged-radio test, different loop.

    A sim that burns a core doing nothing costs more than the sim is worth on a
    machine already running Gazebo.
    """
    medium = VirtualLoRaMedium(["a", "b"], radio=TEST_RADIO)
    try:
        time.sleep(0.3)
        before = time.process_time()
        time.sleep(2.0)
        cpu_seconds = time.process_time() - before
        assert cpu_seconds < 0.5, (
            f"the medium burned {cpu_seconds:.2f} CPU-seconds over 2s idle; "
            "a loop is spinning"
        )
    finally:
        medium.close()
