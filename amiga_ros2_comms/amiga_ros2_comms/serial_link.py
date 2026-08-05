#!/usr/bin/env python3
"""A serial port that reopens itself.

Transport-agnostic helper shared by comms nodes in this package. A USB serial
adapter can vanish and come back — replugged cable, Arduino reset, hub glitch —
and a modem driver should ride that out rather than die, so the port is opened
lazily and reopened on a throttle after any error.

Both the reader and writer threads share one instance; the lock only covers
open/close bookkeeping, not the blocking read and write calls themselves.
"""

import threading
import time
from typing import Callable, Optional

import serial


class SerialLink:
    """Lazily-opened, self-reopening wrapper around ``serial.Serial``."""

    def __init__(
        self,
        port: str,
        baud: int,
        read_timeout: float = 0.1,
        write_timeout: float = 1.0,
        reconnect_period: float = 2.0,
        on_event: Optional[Callable[[str, str], None]] = None,
    ):
        self._port = port
        self._baud = baud
        self._read_timeout = read_timeout
        self._write_timeout = write_timeout
        self._reconnect_period = reconnect_period
        # ("info"|"warn", message) — lets the node log without this module
        # importing rclpy.
        self._on_event = on_event or (lambda level, msg: None)

        self._lock = threading.Lock()
        self._serial: Optional[serial.Serial] = None
        self._next_attempt = 0.0
        self._closed = False

    @property
    def port(self) -> str:
        return self._port

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._serial is not None

    def get(self) -> Optional[serial.Serial]:
        """Return an open port, opening one if due. None if not available yet.

        Reopen attempts are throttled, so a caller in a tight loop will not spin
        on a missing device; it just gets None until the next attempt is due.
        """
        with self._lock:
            if self._closed:
                return None
            if self._serial is not None:
                return self._serial
            now = time.monotonic()
            if now < self._next_attempt:
                return None
            self._next_attempt = now + self._reconnect_period
            try:
                self._serial = serial.Serial(
                    port=self._port,
                    baudrate=self._baud,
                    timeout=self._read_timeout,
                    write_timeout=self._write_timeout,
                )
            except (serial.SerialException, OSError) as exc:
                self._on_event("warn", f"cannot open {self._port}: {exc}")
                self._serial = None
                return None
            self._on_event("info", f"opened {self._port} at {self._baud} baud")
            return self._serial

    def drop(self, reason: str) -> None:
        """Close the port after an error so the next :meth:`get` reopens it."""
        with self._lock:
            if self._serial is None:
                return
            self._on_event("warn", f"dropping {self._port}: {reason}")
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001 - closing a dead port may fail
                pass
            self._serial = None
            self._next_attempt = time.monotonic() + self._reconnect_period

    def close(self) -> None:
        """Shut down for good; no further reopening."""
        with self._lock:
            self._closed = True
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:  # noqa: BLE001
                    pass
                self._serial = None
