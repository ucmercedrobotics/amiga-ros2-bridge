#!/usr/bin/env python3
"""A virtual LoRa medium: real serial ports, simulated air.

For Gazebo, CI, and anywhere else there is no radio. Each robot gets a device
path that behaves like the serial side of its own LoRa modem. The bridge node
opens it exactly as it opens /dev/ttyUSB0 and cannot tell the difference - same
frames, same CRC, same baud rate it politely ignores. Nothing in bridge_node.py
changes to run in simulation, which is the whole point: what Gazebo exercises is
the code that ships.

Only two things separate this from a socat pty pair, and each one exists because
a test would otherwise be a lie:

  * **Broadcast, minus the sender.** One robot transmits, every *other* robot
    hears it. A half-duplex radio never hears itself, and dedup logic that was
    only ever tested against a two-ended pipe looks correct until it meets a
    third robot.
  * **Time on air.** A frame occupies the modem for as long as the real one
    would - 318 ms for 200 bytes at SF7, not the microseconds a pty takes. While
    the modem is busy it stops draining its serial port, exactly as real
    firmware does, so the host's write buffer fills and the bridge finally
    experiences the backpressure it was built to absorb. Against a plain pty
    that path is unreachable, because a pty accepts everything instantly.

Nothing else about the radio is modelled. Collisions, packet loss, and range all
belong to the layer that can *react* to them; this bridge is forbidden to (no
ACKs, no retries, no dedup), so simulating them here would only be scenery.

Payloads stay opaque. This module knows about frames and airtime; it has no idea
what any of the bytes mean.

Pure Python and no ROS, so it is unit-testable on its own. sim_node.py is the
thin ROS wrapper that exposes the knobs as parameters.
"""

import contextlib
import heapq
import itertools
import os
import select
import termios
import threading
import time
import tty
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence

from .airtime import DWELL_LIMIT_SEC, RadioConfig, airtime_sec
from .framing import MAX_BODY_BYTES, FrameParser, encode_frame

# Longest the loop sleeps before rechecking the stop flag.
_MAX_WAIT_SEC = 0.05

# Ceiling on one read from a robot's port. A frame is at most ~260 bytes.
_READ_CHUNK = 4096


def open_raw_pty():
    """Open a pty in raw mode. Returns ``(master_fd, slave_fd, device_path)``.

    Raw both ways with echo off: the link has to be an exact byte pipe, since a
    line discipline that helpfully translated CR to LF would corrupt frames.
    """
    master_fd, slave_fd = os.openpty()
    tty.setraw(master_fd)
    tty.setraw(slave_fd)
    attrs = termios.tcgetattr(slave_fd)
    attrs[1] &= ~termios.OPOST
    attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON)
    termios.tcsetattr(slave_fd, termios.TCSANOW, attrs)
    return master_fd, slave_fd, os.ttyname(slave_fd)


@dataclass
class MediumStats:
    """Counters for the simulated air. Diagnostics only."""

    frames_sent: int = 0
    frames_delivered: int = 0
    # Frames the bridge wrote that this side could not parse. There is no line
    # noise on a pty, so any of these means the host framing disagrees with
    # docs/lora_frame_contract.md - a bug, not a loss.
    tx_frame_errors: int = 0

    def as_dict(self) -> dict:
        return dict(self.__dict__)


class _Radio:
    """One robot's end: a pty, a frame parser, and when its modem frees up."""

    def __init__(self, name: str, max_payload_bytes: int):
        self.name = name
        self.master_fd, self.slave_fd, self.device = open_raw_pty()
        # Non-blocking, so a robot that is not draining its port can never wedge
        # delivery for every other robot.
        os.set_blocking(self.master_fd, False)
        # The slave fd is held open only so the pty survives a bridge closing
        # its end. It is never read, so it cannot steal bytes from the bridge.
        self.parser = FrameParser(max_payload_bytes=max_payload_bytes)
        self.free_at = 0.0
        self.reported_drops = 0
        self.path = self.device
        self.symlink: Optional[str] = None

    def close(self) -> None:
        if self.symlink:
            with contextlib.suppress(OSError):
                os.unlink(self.symlink)
        for fd in (self.master_fd, self.slave_fd):
            with contextlib.suppress(OSError):
                os.close(fd)


class VirtualLoRaMedium:
    """N virtual radios sharing one simulated channel."""

    def __init__(
        self,
        names: Sequence[str],
        symlink_dir: Optional[str] = None,
        radio: RadioConfig = RadioConfig(),
        max_payload_bytes: int = MAX_BODY_BYTES,
        dwell_limit_sec: float = DWELL_LIMIT_SEC,
        on_event: Optional[Callable[[str, str], None]] = None,
    ):
        if not names:
            raise ValueError("a medium needs at least one radio")
        if len(set(names)) != len(names):
            raise ValueError(f"radio names must be unique, got {list(names)}")

        self._radio_config = radio
        self._dwell_limit_sec = dwell_limit_sec
        self._on_event = on_event or (lambda level, msg: None)

        self._radios: Dict[str, _Radio] = {
            name: _Radio(name, max_payload_bytes) for name in names
        }
        if symlink_dir:
            self._install_symlinks(symlink_dir)

        self._stats = MediumStats()
        self._dwell_warned = False

        self._lock = threading.Lock()
        self._inflight: List[tuple] = []  # heap of (deliver_at, seq, sender, payload)
        self._seq = itertools.count()

        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="lora-sim-air", daemon=True
        )
        self._thread.start()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _install_symlinks(self, directory: str) -> None:
        """Give each radio a stable path, since /dev/pts numbers are arbitrary.

        A launch file has to name the port before the pty exists, so the pty
        cannot be allowed to name itself.
        """
        os.makedirs(directory, exist_ok=True)
        for name, radio in self._radios.items():
            path = os.path.join(directory, name)
            # A stale link from a previous run may point at a recycled pts node,
            # which would silently wire a robot to somebody else's port.
            with contextlib.suppress(OSError):
                os.unlink(path)
            os.symlink(radio.device, path)
            radio.symlink = path
            radio.path = path

    @property
    def ports(self) -> Dict[str, str]:
        """Robot name -> the device path its bridge should open."""
        return {name: radio.path for name, radio in self._radios.items()}

    @property
    def radio_config(self) -> RadioConfig:
        return self._radio_config

    def stats(self) -> dict:
        with self._lock:
            counters = self._stats.as_dict()
            counters["in_flight"] = len(self._inflight)
        return counters

    # ------------------------------------------------------------------
    # The loop: one thread does both halves, because both are driven by the
    # same clock and neither does enough work to want its own.
    # ------------------------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            self._deliver_due(now)

            # A modem that is mid-transmission is not reading its serial port.
            # Leaving those fds out of the select is the entire backpressure
            # model: the pty buffer fills and the bridge's write blocks, which
            # is what happens on real hardware.
            idle = [r for r in self._radios.values() if r.free_at <= now]
            fds = [r.master_fd for r in idle]

            try:
                readable, _, _ = select.select(fds, [], [], self._next_wait(now))
            except (OSError, ValueError):
                return

            by_fd = {r.master_fd: r for r in idle}
            for fd in readable:
                self._read_from(by_fd[fd])

    def _next_wait(self, now: float) -> float:
        """Sleep only until the next thing that needs doing."""
        deadlines = [r.free_at for r in self._radios.values() if r.free_at > now]
        with self._lock:
            if self._inflight:
                deadlines.append(self._inflight[0][0])
        if not deadlines:
            return _MAX_WAIT_SEC
        return max(0.0, min(min(deadlines) - now, _MAX_WAIT_SEC))

    def _read_from(self, radio: _Radio) -> None:
        try:
            chunk = os.read(radio.master_fd, _READ_CHUNK)
        except (BlockingIOError, OSError):
            return
        if not chunk:
            return
        for frame in radio.parser.feed(chunk):
            self._transmit(radio, frame.payload)
        self._report_tx_frame_errors(radio)

    def _report_tx_frame_errors(self, radio: _Radio) -> None:
        drops = radio.parser.stats.drops
        if drops == radio.reported_drops:
            return
        new_errors = drops - radio.reported_drops
        radio.reported_drops = drops
        with self._lock:
            self._stats.tx_frame_errors += new_errors
        self._on_event(
            "warn",
            f"{radio.name} sent {new_errors} unparseable frame(s); "
            "host framing does not match docs/lora_frame_contract.md",
        )

    def _transmit(self, radio: _Radio, payload: bytes) -> None:
        """Occupy this radio's modem for the frame's real time on air."""
        air = airtime_sec(len(payload), self._radio_config)
        if not self._dwell_warned and 0 < self._dwell_limit_sec < air:
            self._dwell_warned = True
            self._on_event(
                "warn",
                f"a {len(payload)}-byte payload takes {air * 1000:.0f} ms on air at "
                f"{self._radio_config.describe()}, over the "
                f"{self._dwell_limit_sec * 1000:.0f} ms dwell limit; this frame would "
                "be illegal on real hardware in the US 915 MHz band",
            )

        # Chained off free_at, not off now: a burst is serialised behind itself,
        # because one modem sends one frame at a time.
        start = max(time.monotonic(), radio.free_at)
        radio.free_at = start + air
        with self._lock:
            self._stats.frames_sent += 1
            heapq.heappush(
                self._inflight, (radio.free_at, next(self._seq), radio.name, payload)
            )

    def _deliver_due(self, now: float) -> None:
        while True:
            with self._lock:
                if not self._inflight or self._inflight[0][0] > now:
                    return
                _, _, sender, payload = heapq.heappop(self._inflight)
            self._broadcast(sender, payload)

    def _broadcast(self, sender: str, payload: bytes) -> None:
        wire = encode_frame(payload)
        for name, radio in self._radios.items():
            if name == sender:
                continue  # a half-duplex radio never hears itself
            try:
                written = os.write(radio.master_fd, wire)
            except (BlockingIOError, OSError):
                continue
            if written == len(wire):
                with self._lock:
                    self._stats.frames_delivered += 1

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Stop the thread and remove the ports. Safe to call twice."""
        if self._stop.is_set():
            return
        self._stop.set()
        self._thread.join(timeout=2.0)
        for radio in self._radios.values():
            radio.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()
