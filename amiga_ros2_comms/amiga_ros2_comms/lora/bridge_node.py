#!/usr/bin/env python3
"""ROS2 node owning the serial port to the LoRa Arduino.

A dumb modem driver. It exposes the radio as two topics of opaque framed bytes:
publish a payload to ``/lora/tx`` and it goes out; a validated payload off the
air arrives on ``/lora/rx``. An identical instance runs on every robot.

This node knows about frames, CRC and queues. It knows nothing about what the
bytes mean — no message types, no task IDs, no ACKs, no dedup, no retries.
Those belong to the codec, reliability and coordinator layers above it. It would
work unchanged if we shipped weather data instead of coordination messages.

Concurrency is the part that matters here. The radio is half-duplex and slow,
and nothing upstream may ever block on a serial write, so there are three
threads around two bounded queues:

    /lora/tx callback --> [tx ring] --> writer thread --> serial
    serial --> reader thread --> [rx ring] --> publisher thread --> /lora/rx

The subscription callback only enqueues, so a behaviour-tree tick that publishes
returns instantly whether or not the radio is mid-transmit.

The outbound ring is priority-ordered and the inbound one is not, because only
the outbound one is a real bottleneck: while the modem is busy it stops draining
the serial port, the write blocks, and frames pile up here. Whichever frame is
at the head when the port frees up is the one that gets the channel, so the
order of this queue decides whether an ACK beats the sender's retransmit timer.
The priority is a number the publisher set on the LoRaFrame -- this node sorts
on it and still never reads a payload byte, so it is exactly as opaque as it
was. Ordering only: nothing here paces or rate-limits the link.
"""

import threading
from typing import Optional

import rclpy
import serial
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from amiga_interfaces.msg import LoRaFrame

from ..ring_queue import (
    DROP_OLDEST,
    POLICIES,
    BoundedPriorityRingQueue,
    BoundedRingQueue,
)
from ..serial_link import SerialLink
from .framing import (
    LINK_STATS_BYTES,
    MAX_BODY_BYTES,
    FrameError,
    FrameParser,
    encode_frame,
)

# Chunk size for serial reads. The read is timeout-bounded, so this is only a
# ceiling on how much one call may return, not a required amount.
_READ_CHUNK = 512

# How long consumer threads block on their queue before rechecking the run flag.
_QUEUE_POLL_SEC = 0.2

# Names of the outbound priority bands, most urgent first. Display only: they
# key the stats line and nothing branches on them. Which *messages* land in
# which band is decided in reliability/priority.py, which is the authority; a
# rename there costs a stale log key here and nothing more.
TX_BAND_NAMES = ("urgent", "bulk")

# Link-stats modes, i.e. what the attached firmware prepends to inbound frames.
# Presence cannot be sniffed (see docs/lora_frame_contract.md), so it is stated.
LINK_STATS_NONE = "none"
LINK_STATS_HEADER = "header"
LINK_STATS_MODES = (LINK_STATS_NONE, LINK_STATS_HEADER)


def _describe(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


class LoRaBridge(Node):
    """Serial<->topic bridge for the LoRa radio."""

    def __init__(self, **node_kwargs):
        # node_kwargs forwards the usual rclpy node options (namespace,
        # parameter_overrides). Production launches pass none of them and get
        # the node in the default namespace; the loopback test uses them to run
        # two instances side by side.
        super().__init__("lora_bridge", **node_kwargs)

        self._declare_parameters()
        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("baud").value)
        self._max_payload = self._validated_max_payload()
        tx_policy = self._validated_policy("tx_overflow_policy")
        rx_policy = self._validated_policy("rx_overflow_policy")
        tx_depth = int(self.get_parameter("tx_queue_depth").value)
        rx_depth = int(self.get_parameter("rx_queue_depth").value)
        self._link_stats = self._validated_link_stats()

        self._tx_queue = BoundedPriorityRingQueue(
            tx_depth, tx_policy, levels=len(TX_BAND_NAMES)
        )
        self._rx_queue = BoundedRingQueue(rx_depth, rx_policy)
        self._parser = FrameParser(
            max_payload_bytes=self._max_payload,
            with_link_stats=self._link_stats == LINK_STATS_HEADER,
        )
        self._link = SerialLink(
            port=port,
            baud=baud,
            read_timeout=float(self.get_parameter("read_timeout_sec").value),
            write_timeout=float(self.get_parameter("write_timeout_sec").value),
            reconnect_period=float(self.get_parameter("reconnect_period_sec").value),
            on_event=self._log_link_event,
        )

        # Counters the node owns; frame-level ones live on the parser.
        self._tx_frames = 0
        self._tx_oversize = 0
        self._tx_write_errors = 0
        self._rx_published = 0

        # Relative topic names, so a namespaced instance (used by the loopback
        # test, and by anything running two radios) remaps cleanly. In the
        # default namespace these resolve to /lora/tx and /lora/rx.
        self._rx_pub = self.create_publisher(LoRaFrame, "lora/rx", rx_depth)
        self._tx_sub = self.create_subscription(
            LoRaFrame, "lora/tx", self._on_tx, tx_depth
        )

        # Inverted polarity on purpose: the event is *clear* while running, so
        # `_stop.wait(t)` is a real interruptible sleep. An event that is set
        # while running would make every wait() return instantly, turning any
        # idle loop into a spin.
        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._reader_loop, name="lora-reader", daemon=True),
            threading.Thread(target=self._writer_loop, name="lora-writer", daemon=True),
            threading.Thread(
                target=self._publish_loop, name="lora-publish", daemon=True
            ),
        ]
        for thread in self._threads:
            thread.start()

        stats_period = float(self.get_parameter("stats_period_sec").value)
        self._stats_timer = (
            self.create_timer(stats_period, self._log_stats)
            if stats_period > 0
            else None
        )

        self.get_logger().info(
            f"lora_bridge on {port} @ {baud} baud, "
            f"max_payload_bytes={self._max_payload}, "
            f"tx={tx_depth}/{tx_policy} ({'>'.join(TX_BAND_NAMES)}), "
            f"rx={rx_depth}/{rx_policy}, "
            f"link_stats={self._link_stats}"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "serial_port",
            "/dev/ttyUSB0",
            _describe("Device path of the LoRa Arduino."),
        )
        self.declare_parameter(
            "baud",
            115200,
            _describe(
                "Serial baud rate. Must match the firmware's Serial.begin(). "
                "Assumed until the firmware repo can be checked."
            ),
        )
        self.declare_parameter(
            "max_payload_bytes",
            200,
            _describe(
                "Largest payload accepted in either direction. Ceiling is 255 "
                "(the frame's length byte); the real limit is the FCC 400 ms "
                "dwell budget at the firmware's spreading factor. See "
                "docs/lora_frame_contract.md."
            ),
        )
        self.declare_parameter(
            "tx_queue_depth",
            32,
            _describe(
                "Outbound ring buffer capacity, in frames. One budget shared "
                "across the priority bands, not one each, so bulk traffic may "
                "use the whole queue when the channel is quiet and gives it "
                "back when urgent traffic needs it."
            ),
        )
        self.declare_parameter(
            "rx_queue_depth", 64, _describe("Inbound queue capacity, in frames.")
        )
        self.declare_parameter(
            "tx_overflow_policy",
            DROP_OLDEST,
            _describe(
                f"What a full outbound queue does: one of {POLICIES}. Default "
                "drop_oldest, because a stale coordination message is worthless. "
                "Applies within a priority band -- the victim is always chosen "
                "from the least urgent band holding anything, so bulk traffic "
                "can never displace urgent traffic under either policy."
            ),
        )
        self.declare_parameter(
            "rx_overflow_policy",
            DROP_OLDEST,
            _describe(f"What a full inbound queue does: one of {POLICIES}."),
        )
        self.declare_parameter(
            "rx_link_stats",
            LINK_STATS_NONE,
            _describe(
                f"What the firmware prepends to inbound frames: {LINK_STATS_MODES}. "
                "Presence cannot be detected from opaque payloads, so it is "
                "configured. Current firmware sends none."
            ),
        )
        self.declare_parameter(
            "read_timeout_sec", 0.1, _describe("Serial read timeout.")
        )
        self.declare_parameter(
            "write_timeout_sec",
            1.0,
            _describe(
                "Serial write timeout. On expiry the frame is dropped rather "
                "than retried; the writer thread must not wedge."
            ),
        )
        self.declare_parameter(
            "reconnect_period_sec",
            2.0,
            _describe("Delay between attempts to reopen a missing serial port."),
        )
        self.declare_parameter(
            "stats_period_sec",
            30.0,
            _describe("Period of the counters log line. 0 disables it."),
        )

    def _validated_max_payload(self) -> int:
        value = int(self.get_parameter("max_payload_bytes").value)
        limit = MAX_BODY_BYTES - (
            LINK_STATS_BYTES
            if self.get_parameter("rx_link_stats").value == LINK_STATS_HEADER
            else 0
        )
        if not 1 <= value <= limit:
            self.get_logger().warn(
                f"max_payload_bytes={value} out of range 1..{limit}; clamping"
            )
            value = max(1, min(value, limit))
        return value

    def _validated_policy(self, name: str) -> str:
        value = self.get_parameter(name).value
        if value not in POLICIES:
            self.get_logger().warn(
                f"{name}={value!r} is not one of {POLICIES}; using {DROP_OLDEST}"
            )
            return DROP_OLDEST
        return value

    def _validated_link_stats(self) -> str:
        value = self.get_parameter("rx_link_stats").value
        if value not in LINK_STATS_MODES:
            self.get_logger().warn(
                f"rx_link_stats={value!r} is not one of {LINK_STATS_MODES}; "
                f"using {LINK_STATS_NONE}"
            )
            return LINK_STATS_NONE
        return value

    # ------------------------------------------------------------------
    # TX path
    # ------------------------------------------------------------------

    def _on_tx(self, msg: LoRaFrame) -> None:
        """Queue a payload and return. Never touches the serial port."""
        payload = bytes(msg.data)
        if len(payload) > self._max_payload:
            self._tx_oversize += 1
            self.get_logger().warn(
                f"dropping {len(payload)}-byte tx payload, "
                f"max_payload_bytes={self._max_payload} (no fragmentation here)"
            )
            return
        # msg.priority is whatever the publisher set, unvalidated on purpose:
        # the queue clamps an unrecognised band toward the least urgent end, and
        # a subscription callback is the last place that should be able to raise.
        if not self._tx_queue.put(payload, msg.priority):
            # Expected under load, so throttled rather than silent or spammy.
            # The per-band split is the number that matters: shedding bulk is
            # the queue working as intended, shedding urgent means it is too
            # shallow for the traffic.
            by_band = ", ".join(
                f"{name}={count}"
                for name, count in zip(
                    TX_BAND_NAMES, self._tx_queue.dropped_by_priority
                )
            )
            self.get_logger().warn(
                f"tx queue full (depth {self._tx_queue.capacity}, "
                f"{self._tx_queue.policy}); {self._tx_queue.dropped} dropped total "
                f"({by_band})",
                throttle_duration_sec=5.0,
            )

    def _writer_loop(self) -> None:
        """Drain the outbound ring at whatever pace the link accepts bytes."""
        while not self._stop.is_set():
            payload = self._tx_queue.get(timeout=_QUEUE_POLL_SEC)
            if payload is None:
                continue
            port = self._link.get()
            if port is None:
                # No radio attached. Dropping is correct: buffering for an
                # absent port only delivers stale frames later.
                self._tx_write_errors += 1
                continue
            try:
                frame = encode_frame(payload)
            except FrameError as exc:
                self._tx_oversize += 1
                self.get_logger().warn(f"cannot frame tx payload: {exc}")
                continue
            try:
                port.write(frame)
                port.flush()
                self._tx_frames += 1
            except serial.SerialTimeoutException:
                # Congestion, not a fault: the link is alive but not draining.
                # Drop the frame and keep the port, because tearing down a
                # working radio because it was busy would be worse.
                self._tx_write_errors += 1
                self.get_logger().warn(
                    "serial write timed out; dropping frame",
                    throttle_duration_sec=5.0,
                )
            except Exception as exc:  # noqa: BLE001 - any serial fault, same handling
                self._tx_write_errors += 1
                self._link.drop(f"write failed: {exc}")

    # ------------------------------------------------------------------
    # RX path
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        """Parse frames off the incoming stream into the inbound queue."""
        while not self._stop.is_set():
            port = self._link.get()
            if port is None:
                # get() throttles reopen attempts, so this is not a spin.
                self._stop.wait(_QUEUE_POLL_SEC)
                continue
            try:
                waiting = port.in_waiting or 1
                chunk = port.read(min(waiting, _READ_CHUNK))
            except Exception as exc:  # noqa: BLE001
                self._link.drop(f"read failed: {exc}")
                continue
            if not chunk:
                continue
            for frame in self._parser.feed(chunk):
                if not self._rx_queue.put(frame):
                    self.get_logger().warn(
                        f"rx queue full (depth {self._rx_queue.capacity}, "
                        f"{self._rx_queue.policy}); "
                        f"{self._rx_queue.dropped} dropped total",
                        throttle_duration_sec=5.0,
                    )

    def _publish_loop(self) -> None:
        """Publish validated payloads out of the inbound queue."""
        while not self._stop.is_set():
            frame = self._rx_queue.get(timeout=_QUEUE_POLL_SEC)
            if frame is None:
                continue
            msg = LoRaFrame()
            msg.data = list(frame.payload)
            msg.has_link_stats = frame.has_link_stats
            if frame.has_link_stats:
                msg.rssi = int(frame.rssi)
                msg.snr = float(frame.snr)
            self._rx_pub.publish(msg)
            self._rx_published += 1

    # ------------------------------------------------------------------
    # Diagnostics and lifecycle
    # ------------------------------------------------------------------

    def _log_link_event(self, level: str, message: str) -> None:
        # Two separate call sites on purpose. rclpy keys its logger state on the
        # caller's file and line, and raises if one line ever logs at two
        # different severities. Dispatching through a single getattr() call put
        # both levels on the same line, so the first "opened" after a "cannot
        # open" threw inside the reader thread and killed it: the radio came
        # back and the node stayed deaf, silently, forever.
        if level == "warn":
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def stats(self) -> dict:
        """Everything this node counts. Used by the logger and by tests."""
        counters = self._parser.stats.as_dict()
        counters.update(
            tx_frames=self._tx_frames,
            tx_dropped_queue=self._tx_queue.dropped,
            tx_oversize=self._tx_oversize,
            tx_write_errors=self._tx_write_errors,
            rx_published=self._rx_published,
            rx_dropped_queue=self._rx_queue.dropped,
            port_open=self._link.is_open,
        )
        # Broken out per band as well as in total, so a run can say which
        # traffic the queue shed rather than only how much.
        for name, count in zip(TX_BAND_NAMES, self._tx_queue.dropped_by_priority):
            counters[f"tx_dropped_{name}"] = count
        return counters

    def _log_stats(self) -> None:
        self.get_logger().debug(
            " ".join(f"{k}={v}" for k, v in sorted(self.stats().items()))
        )

    def shutdown(self) -> None:
        """Stop the threads and release the port. Safe to call twice."""
        if self._stop.is_set():
            return
        self._stop.set()
        self._tx_queue.close()
        self._rx_queue.close()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._link.close()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LoRaBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or SIGTERM from launch tearing the stack down. Both are
        # ordinary ways for this node to stop, not errors worth a traceback.
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
