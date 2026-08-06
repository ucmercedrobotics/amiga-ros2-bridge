#!/usr/bin/env python3
"""ROS2 node wiring the reliability session to the LoRa bridge's topics.

Thin by design. Everything that can be decided without ROS is decided in
session.py; this file owns parameters, the two topic endpoints, the retransmit
tick and the counters log line. That split is what lets the acceptance tests
drive a whole retransmit campaign over a lossy link with no executor, no timers
and no sleeping.

    /lora/rx --> on_frame --> [dedup] --> on_deliver --> coordinator
    coordinator --> send_reliable/send_broadcast --> [seq, ACK, retry] --> /lora/tx

The coordinator is ``amiga_ros2_coordinator``, and it consumes this node
in-process -- its ``coordinator`` executable constructs one of these and adds
both to a single executor. This node also runs standalone (it will ACK inbound
GRANTs addressed to it and dedup everything else, which is enough to be useful
on a bench with two radios), but not *alongside* the coordinator: two
reliability layers on one radio would each ACK the other's inbound traffic and
duplicate every send. The wiring above looks like::

    reliability = ReliabilityNode()
    reliability.set_on_deliver(my_handler)
    executor.add_node(reliability)

    future = reliability.send_reliable(dst=4, msg=Grant(src=0, seq=0,
                                                        task_id=7, winner_id=4))
    future.add_done_callback(lambda f: print(f.result()))

In-process rather than over a ROS service on purpose: the interface carries
typed codec messages and a delivery outcome, and re-encoding those into .msg
files to move them between two nodes on the same machine would add a
serialization hop, a second failure mode, and an interface to keep in step with
the codec -- to connect two layers that always ship together.
"""

from typing import Callable, Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from amiga_interfaces.msg import LoRaFrame

from ..codec import (
    DEFAULT_MAX_PAYLOAD_BYTES,
    MAX_MESSAGE_BYTES,
    MESSAGE_SIZES,
    Ack,
    Message,
)
from ..lora.airtime import RadioConfig, airtime_sec
from .notes import CompletedNote
from .session import ReliabilityParams, ReliabilitySession


def _describe(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


def round_trip_floor_sec(
    radio: RadioConfig, request_bytes: int = MAX_MESSAGE_BYTES
) -> float:
    """Absolute floor for ``retransmit_timeout_sec`` on this radio.

    The time on air of the message plus the time on air of the ACK it is
    waiting for. Nothing can arrive sooner than this, so a timeout below it
    guarantees a retransmit while the first ACK is still being transmitted --
    on a half-duplex channel that is how one lost packet turns into a storm.

    A floor, not a recommendation. It counts no queueing in the bridge, no
    scheduling delay and no contention with other robots, all of which are
    real. Hence the generous multiplier at the call site.
    """
    return airtime_sec(request_bytes, radio) + airtime_sec(MESSAGE_SIZES[Ack], radio)


#: How much headroom over the bare round-trip floor a configured timeout should
#: have before we stop complaining about it. Queueing, half-duplex turnaround
#: and other robots' traffic all land in this factor.
TIMEOUT_SAFETY_FACTOR = 4.0


def timeout_shortfall(
    retransmit_timeout_sec: float,
    radio: RadioConfig,
    safety_factor: float = TIMEOUT_SAFETY_FACTOR,
) -> Optional[str]:
    """Complain about a retransmit timeout the link cannot support, or return None.

    Separate from the node, and returning a string rather than logging one, so
    the rule can be tested without an rclpy logger -- rclpy logs through rcutils
    rather than Python's logging module, and a warning nothing can assert on is
    a warning that quietly stops working.
    """
    floor = round_trip_floor_sec(radio)
    if retransmit_timeout_sec >= floor * safety_factor:
        return None
    return (
        f"retransmit_timeout_sec={retransmit_timeout_sec} is close to or below "
        f"the round trip at {radio.describe()} ({floor * 1000:.0f} ms of pure "
        f"airtime, before any queueing). Retransmits will be sent while ACKs "
        f"are still in flight and will congest a half-duplex channel; want at "
        f"least {floor * safety_factor:.2f}s."
    )


class ReliabilityNode(Node):
    """Reliable/best-effort delivery over the bridge's opaque frame topics."""

    def __init__(self, **node_kwargs):
        # node_kwargs forwards the usual rclpy options. Production passes none;
        # the two-robot loopback test uses namespace and parameter_overrides to
        # run several instances in one process.
        super().__init__("lora_reliability", **node_kwargs)

        self._declare_parameters()
        node_id = self._validated_node_id()
        params = self._validated_params()
        rx_depth = int(self.get_parameter("rx_queue_depth").value)
        tx_depth = int(self.get_parameter("tx_queue_depth").value)

        # Relative names, so a namespaced instance remaps cleanly onto its own
        # bridge. In the default namespace these are /lora/tx and /lora/rx.
        self._tx_pub = self.create_publisher(LoRaFrame, "lora/tx", tx_depth)

        self._session = ReliabilitySession(
            node_id=node_id,
            send_frame=self._publish_frame,
            params=params,
            clock=self._now,
            on_event=self._log_event,
        )

        self._rx_sub = self.create_subscription(
            LoRaFrame, "lora/rx", self._on_rx, rx_depth
        )

        tick_period = float(self.get_parameter("tick_period_sec").value)
        self._tick_timer = self.create_timer(tick_period, self._session.tick)

        stats_period = float(self.get_parameter("stats_period_sec").value)
        self._stats_timer = (
            self.create_timer(stats_period, self._log_stats)
            if stats_period > 0
            else None
        )

        self._warn_if_timeout_too_short(params)
        self.get_logger().info(
            f"lora_reliability as robot {node_id}: "
            f"retransmit_timeout={params.retransmit_timeout_sec}s "
            f"x{params.retransmit_backoff} up to {params.max_retries} retries, "
            f"dedup {params.dedup_max_entries} entries / "
            f"{params.dedup_ttl_sec}s, tick={tick_period}s"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "node_id",
            1,
            _describe(
                "This robot's ID, 1..255, and the 'src' of everything it sends. "
                "Must be unique across the fleet: two robots sharing an ID make "
                "(src, seq) ambiguous, which silently dedups one robot's traffic "
                "away as duplicates of the other's."
            ),
        )
        self.declare_parameter(
            "retransmit_timeout_sec",
            3.0,
            _describe(
                "Wait for an ACK before retransmitting. Must exceed the round "
                "trip time on air; the node warns at startup if it does not. "
                "The default has headroom from SF7 to SF10; time on air doubles "
                "per spreading factor, so SF11 and SF12 need it raised."
            ),
        )
        self.declare_parameter(
            "max_retries",
            3,
            _describe(
                "Retransmits after the original, so 3 puts up to 4 copies on "
                "the air before the send is reported failed."
            ),
        )
        self.declare_parameter(
            "retransmit_backoff",
            1.5,
            _describe(
                "Multiplier applied to the timeout per retransmit. 1.0 gives a "
                "flat timer; above 1.0 backs off so a congested channel is not "
                "hammered at a fixed rate."
            ),
        )
        self.declare_parameter(
            "max_retransmit_timeout_sec",
            10.0,
            _describe("Ceiling on the backed-off interval."),
        )
        self.declare_parameter(
            "dedup_ttl_sec",
            120.0,
            _describe(
                "How long a delivered (src, seq) is remembered. Must outlast "
                "the longest retransmit campaign and the coordinator's slowest "
                "re-broadcast, or a late copy is delivered twice."
            ),
        )
        self.declare_parameter(
            "dedup_max_entries",
            512,
            _describe(
                "Hard cap on remembered message IDs, so a fast or hostile peer "
                "costs bounded memory."
            ),
        )
        self.declare_parameter(
            "max_pending",
            32,
            _describe(
                "Reliable sends in flight at once. Further sends fail fast "
                "rather than queue, because a caller can retry a rejection but "
                "cannot retry a hang."
            ),
        )
        self.declare_parameter(
            "max_payload_bytes",
            DEFAULT_MAX_PAYLOAD_BYTES,
            _describe(
                "Per-message budget handed to the codec. Every built type fits "
                f"in {MAX_MESSAGE_BYTES} bytes; there is no fragmentation here."
            ),
        )
        self.declare_parameter(
            "tick_period_sec",
            0.1,
            _describe(
                "How often retransmit deadlines are checked. Sets the timing "
                "granularity of every timeout, so keep it well under "
                "retransmit_timeout_sec."
            ),
        )
        self.declare_parameter(
            "rx_queue_depth", 64, _describe("Depth of the /lora/rx subscription.")
        )
        self.declare_parameter(
            "tx_queue_depth", 32, _describe("Depth of the /lora/tx publisher.")
        )
        self.declare_parameter(
            "spreading_factor",
            7,
            _describe(
                "Spreading factor of the radio below, 6..12. Used only to check "
                "retransmit_timeout_sec against the round-trip time on air."
            ),
        )
        self.declare_parameter(
            "stats_period_sec",
            30.0,
            _describe("Period of the counters log line. 0 disables it."),
        )

    def _validated_node_id(self) -> int:
        value = int(self.get_parameter("node_id").value)
        if not 1 <= value <= 255:
            # Refused rather than clamped: a wrong ID is worse than no node,
            # because it corrupts other robots' dedup state as well as its own.
            raise ValueError(
                f"node_id must be 1..255, got {value}. It is this robot's "
                f"fleet-unique address and has no safe default."
            )
        return value

    def _validated_params(self) -> ReliabilityParams:
        return ReliabilityParams(
            retransmit_timeout_sec=float(
                self.get_parameter("retransmit_timeout_sec").value
            ),
            max_retries=int(self.get_parameter("max_retries").value),
            retransmit_backoff=float(self.get_parameter("retransmit_backoff").value),
            max_retransmit_timeout_sec=float(
                self.get_parameter("max_retransmit_timeout_sec").value
            ),
            dedup_ttl_sec=float(self.get_parameter("dedup_ttl_sec").value),
            dedup_max_entries=int(self.get_parameter("dedup_max_entries").value),
            max_pending=int(self.get_parameter("max_pending").value),
            max_payload_bytes=int(self.get_parameter("max_payload_bytes").value),
        )

    def _warn_if_timeout_too_short(self, params: ReliabilityParams) -> None:
        """Check the configured timeout against the link, not against the CPU."""
        radio = RadioConfig(
            spreading_factor=int(self.get_parameter("spreading_factor").value)
        )
        complaint = timeout_shortfall(params.retransmit_timeout_sec, radio)
        if complaint is not None:
            self.get_logger().warn(complaint)

    # ------------------------------------------------------------------
    # Interface to the coordinator
    # ------------------------------------------------------------------

    def send_reliable(self, dst: int, msg: Message):
        """Unicast ``msg`` to ``dst``, confirmed. See ReliabilitySession."""
        return self._session.send_reliable(dst, msg)

    def send_broadcast(self, msg: Message) -> bool:
        """Broadcast ``msg`` once, best-effort. See ReliabilitySession."""
        return self._session.send_broadcast(msg)

    def send_note(self, task_id: int, text: str) -> int:
        """Broadcast free text about ``task_id``. See ReliabilitySession."""
        return self._session.send_note(task_id, text)

    def completed_note(self, src: int, task_id: int):
        """The note ``src`` most recently finished about ``task_id``, or None."""
        return self._session.completed_note(src, task_id)

    def set_on_deliver(self, callback: Optional[Callable[[Message], None]]) -> None:
        """Install the deduplicated inbound callback.

        Called once per distinct message, from the /lora/rx callback's thread.
        Do not block in it -- hand off to your own executor.
        """
        self._session.set_on_deliver(callback)

    def set_on_note(self, callback: Optional[Callable[[CompletedNote], None]]) -> None:
        """Install the completed-note callback.

        Called once per note that arrived whole, never for a fragment and never
        for a note that lost one. Same threading rule as ``set_on_deliver``.
        """
        self._session.set_on_note(callback)

    @property
    def node_id(self) -> int:
        return self._session.node_id

    @property
    def session(self) -> ReliabilitySession:
        """The engine, for tests and for anything wanting the counters."""
        return self._session

    # ------------------------------------------------------------------
    # Topic plumbing
    # ------------------------------------------------------------------

    def _publish_frame(self, payload: bytes, priority: int) -> None:
        # The priority rides on the message rather than being inferred by the
        # bridge, so the bridge can order its outbound queue without ever
        # reading a payload byte. Classification is priority.py's; this is only
        # the wire it travels on.
        msg = LoRaFrame()
        msg.data = list(payload)
        msg.priority = int(priority)
        self._tx_pub.publish(msg)

    def _on_rx(self, msg: LoRaFrame) -> None:
        self._session.on_frame(bytes(msg.data))

    def _now(self) -> float:
        """Monotonic seconds from the ROS clock.

        The ROS clock rather than time.monotonic so that ``use_sim_time`` makes
        retransmit timing follow simulation time along with everything else.
        """
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------
    # Diagnostics and lifecycle
    # ------------------------------------------------------------------

    def _log_event(self, level: str, message: str) -> None:
        # Two call sites, not one dispatched getattr: rclpy keys logger state on
        # the caller's line and raises if one line logs at two severities. See
        # the same note in bridge_node._log_link_event.
        if level == "warn":
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def stats(self) -> dict:
        return self._session.stats()

    def _log_stats(self) -> None:
        self.get_logger().info(
            " ".join(f"{k}={v}" for k, v in sorted(self.stats().items()))
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ReliabilityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or SIGTERM from launch. Ordinary ways to stop, not errors.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
