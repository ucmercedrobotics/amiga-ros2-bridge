#!/usr/bin/env python3
"""ROS2 node wiring the coordinator to the reliability layer and the robot.

Thin by design, like the node one layer down. Everything that can be decided
without ROS is decided in coordinator.py; this file owns parameters, the tick
timer, the preemption topic and the counters log line. That split is what lets
the acceptance tests drive whole auctions and backoff windows with
no executor, no timers and no sleeping.

    /lora/rx --> reliability --> on_message --> [contract net] --> send_* --> /lora/tx
                                        |
                                        +--> ~/preempt_requested --> behaviour tree

    behaviour tree -> planner -> arbiter -> (gave up)
                                              |
              /coordination/infeasible -------+
                       |
                       v
              [ask the triage agent] --> one typed action --> [contract net]

The infeasibility signal is a topic rather than a call, because the thing that
knows local recovery has run out is the agent stack watching the planner and the
arbiter, in another process. What arrives here is "this robot cannot fix this",
never a suggestion of what to do about it -- that is the triage agent's answer,
fetched over a service and constrained to the three actions in schema.py.

The reliability layer is consumed **in-process**: this node adds a
``ReliabilityNode`` to the same executor and calls it directly, because the
interface carries typed codec messages and a delivery outcome and re-encoding
those into .msg files to cross a process boundary that does not exist would buy
a serialization hop and a second failure mode. ``main()`` does that wiring.

What this file deliberately does **not** contain is a navigation client or a
mission client. Those are separate existing nodes, and building adapters to
them here would mean this layer's tests started depending on their behaviour.
The ports are injected; ``main()`` supplies placeholders that decline
everything and say so, which is enough to run on a bench with two radios and
watch the peer registry populate. Wiring real nav and mission stacks means
passing two objects to the constructor and changing nothing else.

Preemption is the one port implemented here, because it *is* a ROS surface: a
latched ``std_msgs/Bool`` the behaviour tree mirrors onto its blackboard and a
reactive condition node reads at safe tick points. Publishing a flag is the
whole of it -- there is no code path in this package that stops an action.
"""

import json
from threading import Lock, Thread
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from amiga_ros2_comms.codec import Capability, cap_mask
from amiga_ros2_comms.reliability.node import ReliabilityNode

from .coordinator import CoordinatorParams, CoordinatorSession
from .model import Location, Task
from .triage_client import optional_client


def _describe(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


#: Latched, so a behaviour tree that starts after the coordinator immediately
#: learns the current state of the flag instead of assuming it is clear. A
#: preemption request that only existing subscribers hear is a preemption
#: request a restarted tree ignores.
LATCHED = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TopicPreemption:
    """The blackboard flag, published as a latched topic.

    Two topics rather than one: the Bool is what the behaviour-tree condition
    node reads, and the String is why, for the operator. Keeping the reason out
    of the flag means a condition node never has to parse anything.

    Idempotent on purpose. Coordination events arrive in bursts -- three GRANTs
    landing together raise the flag three times -- and republishing a flag that
    is already raised would make the tree re-evaluate for no reason.
    """

    def __init__(self, node: Node):
        self._flag_pub = node.create_publisher(Bool, "~/preempt_requested", LATCHED)
        self._reason_pub = node.create_publisher(String, "~/preempt_reason", LATCHED)
        self._logger = node.get_logger()
        self.requested = False
        self.reason = ""
        self._publish()

    def request_yield(self, reason: str) -> None:
        self.reason = reason
        self._reason_pub.publish(String(data=reason))
        if self.requested:
            return
        self.requested = True
        self._publish()
        self._logger.info(f"preemption requested: {reason}")

    def clear(self) -> None:
        if not self.requested:
            return
        self.requested = False
        self.reason = ""
        self._publish()
        self._reason_pub.publish(String(data=""))

    def _publish(self) -> None:
        self._flag_pub.publish(Bool(data=self.requested))


class UnavailableNav:
    """A navigation port that is not connected to anything.

    Declines every question. Named and explicit rather than a silent default,
    because a robot that bids on everything because nav never answered is
    worse than one that bids on nothing: the first takes work it cannot do.
    """

    def __init__(self, logger=None):
        self._logger = logger
        self._warned = False

    def _warn(self) -> None:
        if self._logger is not None and not self._warned:
            self._warned = True
            self._logger.warn(
                "no navigation port is wired to the coordinator: every bid "
                "will be infeasible. Pass a NavInterface to "
                "CoordinatorNode(nav=...)"
            )

    def eta(self, location: Location) -> float:
        self._warn()
        raise RuntimeError("no navigation port wired")

    def can_reach(self, location: Location) -> bool:
        self._warn()
        return False

    def current_location(self) -> Optional[Location]:
        return None


class UnavailableMission:
    """A mission port that is not connected to anything. Same reasoning."""

    def __init__(self, logger=None):
        self._logger = logger
        self._warned = False

    def _warn(self) -> None:
        if self._logger is not None and not self._warned:
            self._warned = True
            self._logger.warn(
                "no mission port is wired to the coordinator: it will not bid, "
                "absorb or transfer anything. Pass a MissionInterface to "
                "CoordinatorNode(mission=...)"
            )

    def can_absorb(self, task: Task) -> bool:
        self._warn()
        return False

    def absorb(self, task: Task) -> None:
        self._warn()

    def release(self, task: Task) -> None:
        self._warn()

    def mark_transferred(self, task: Task) -> None:
        self._warn()

    def current_task_id(self) -> int:
        return 0

    def battery_percent(self) -> int:
        return 0


class CoordinatorNode(Node):
    """Contract-net coordination, wired to a reliability layer and a robot."""

    def __init__(
        self,
        reliability,
        nav=None,
        mission=None,
        interpreter=None,
        replanner=None,
        **node_kwargs,
    ):
        # node_kwargs forwards the usual rclpy options. Production passes none;
        # multi-robot tests use namespace and parameter_overrides to run
        # several instances in one process.
        super().__init__("coordinator", **node_kwargs)

        # Imported here rather than at module scope: the stubs are the current
        # implementations of the two reasoning points, and importing them at
        # the top would read like this node depends on them permanently. It
        # depends on the protocols; these are what is behind them today.
        from .reasoning import AcceptEverything, AlwaysReDelegate

        self._declare_parameters()
        node_id = int(reliability.node_id)
        params = self._validated_params()
        capabilities = self._validated_capabilities()

        self._preemption = TopicPreemption(self)
        self._reliability = reliability
        # Reentrant, and a MultiThreadedExecutor in main(): the triage call
        # blocks on a future this node's own executor has to complete.
        self._blocking_group = ReentrantCallbackGroup()

        self.session = CoordinatorSession(
            node_id=node_id,
            reliability=reliability,
            nav=nav if nav is not None else UnavailableNav(self.get_logger()),
            mission=(
                mission
                if mission is not None
                else UnavailableMission(self.get_logger())
            ),
            interpreter=interpreter if interpreter is not None else AlwaysReDelegate(),
            replanner=replanner if replanner is not None else AcceptEverything(),
            capabilities=capabilities,
            preemption=self._preemption,
            params=params,
            clock=self._now,
            on_event=self._log_event,
        )
        reliability.set_on_deliver(self.session.on_message)

        tick_period = float(self.get_parameter("tick_period_sec").value)
        self._tick_timer = self.create_timer(tick_period, self.session.tick)

        self._clear_sub = self.create_subscription(
            Bool, "~/preempt_ack", self._on_preempt_ack, 10
        )

        self._triage = (
            optional_client(
                self,
                service_name=str(self.get_parameter("triage_service").value),
                timeout_sec=float(self.get_parameter("triage_timeout_sec").value),
                callback_group=self._blocking_group,
            )
            if bool(self.get_parameter("use_triage_agent").value)
            else None
        )
        self._default_capability = int(
            self.get_parameter("default_task_capability").value
        )
        self._in_flight: "set[int]" = set()
        self._in_flight_lock = Lock()
        self._infeasible_sub = self.create_subscription(
            String,
            str(self.get_parameter("infeasible_topic").value),
            self._on_infeasible,
            10,
            callback_group=self._blocking_group,
        )

        stats_period = float(self.get_parameter("stats_period_sec").value)
        self._stats_timer = (
            self.create_timer(stats_period, self._log_stats)
            if stats_period > 0
            else None
        )

        self.get_logger().info(
            f"coordinator as robot {node_id}: "
            f"capabilities={[Capability(c).name for c in capabilities]}, "
            f"announce window {params.announce_window_sec}s, "
            f"bid backoff <={params.bid_max_backoff_sec}s, "
            f"peer timeout {params.peer_timeout_sec}s, "
            f"heartbeat every {params.heartbeat_period_sec}s, tick={tick_period}s"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "capabilities",
            ["DRIVE"],
            _describe(
                "What this robot can do, as Capability names (DRIVE, INSPECT, "
                "SPRAY, HARVEST, MANIPULATE, TRANSPORT, SURVEY, CHARGE_HOST, "
                "RELAY). Advertised in every HEARTBEAT and checked against a "
                "TASK_ANNOUNCE's requirement before this robot bids."
            ),
        )
        self.declare_parameter(
            "announce_window_sec",
            5.0,
            _describe(
                "How long an announcement collects bids before it is "
                "arbitrated. Must exceed the fleet's bid_max_backoff_sec, or "
                "the best-fitting bidder transmits after the auction closed."
            ),
        )
        self.declare_parameter(
            "announce_repeat_sec",
            2.0,
            _describe(
                "Seconds between re-broadcasts of an announcement while its "
                "window is open. Announcements are best-effort broadcasts, so "
                "repetition is how one reaches a robot that was transmitting "
                "when the first went out. 0 disables it."
            ),
        )
        self.declare_parameter(
            "bid_max_backoff_sec",
            2.0,
            _describe(
                "Longest a bidder sits on a bid before transmitting. Backoff "
                "is fitness-proportional -- the best-suited robot waits least "
                "-- and a bidder that overhears a better bid while waiting "
                "suppresses its own entirely."
            ),
        )
        self.declare_parameter(
            "bid_jitter_fraction",
            0.05,
            _describe(
                "Random fraction of the backoff window added to each wait. "
                "Exists only to separate bidders of identical fitness, which "
                "would otherwise wait identical times and collide."
            ),
        )
        self.declare_parameter(
            "bid_memory_sec",
            60.0,
            _describe(
                "How long a bid we sent is remembered, so a GRANT can be "
                "matched to the task it awards. Must outlast the announcer's "
                "window plus its whole GRANT retransmit campaign."
            ),
        )
        self.declare_parameter(
            "max_delegation_attempts",
            2,
            _describe(
                "Announce-and-grant cycles spent on one task before it goes to "
                "local handling. A GRANT that fails to deliver consumes one."
            ),
        )
        self.declare_parameter(
            "redelegation_cooldown_sec",
            30.0,
            _describe(
                "Shortest interval between re-announcements of the same task, "
                "so a mission node reporting one failure in a loop cannot turn "
                "into an announce storm."
            ),
        )
        self.declare_parameter(
            "peer_timeout_sec",
            30.0,
            _describe(
                "Seconds without a HEARTBEAT before a peer is presumed gone. "
                "Wants to be a small multiple of the fleet's heartbeat period, "
                "so a peer survives losing one to the radio."
            ),
        )
        self.declare_parameter(
            "heartbeat_period_sec",
            10.0,
            _describe(
                "How often this robot broadcasts its own HEARTBEAT. This is "
                "what populates every peer's registry. 0 disables emission."
            ),
        )
        self.declare_parameter(
            "settled_retention_sec",
            300.0,
            _describe(
                "How long a transferred or relinquished task stays visible for "
                "introspection before being forgotten."
            ),
        )
        self.declare_parameter(
            "max_open_auctions",
            8,
            _describe(
                "Auctions run at once. Beyond it, re-delegation is refused and "
                "the task goes to local handling: a robot shedding everything "
                "at once has a problem that is not delegable."
            ),
        )
        self.declare_parameter(
            "tick_period_sec",
            0.1,
            _describe(
                "How often auction windows, bid backoffs and peer liveness "
                "are checked. Sets the granularity of every deadline here, so "
                "keep it well under bid_max_backoff_sec."
            ),
        )
        self.declare_parameter(
            "stats_period_sec",
            30.0,
            _describe("Period of the counters log line. 0 disables it."),
        )
        self.declare_parameter(
            "infeasible_topic",
            "/coordination/infeasible",
            _describe(
                "Topic carrying 'this robot cannot recover from this on its "
                "own'. Published by the triage agent when the planner or the "
                "arbiter gives up. This is the owner role's entry point."
            ),
        )
        self.declare_parameter(
            "use_triage_agent",
            True,
            _describe(
                "Ask the triage agent what to do about an anomaly. False falls "
                "back to the local stub interpreter, which is for bench runs "
                "with no model endpoint -- it is not a policy."
            ),
        )
        self.declare_parameter(
            "triage_service",
            "/coordination/interpret_anomaly",
            _describe("Service the triage agent serves interpretations on."),
        )
        self.declare_parameter(
            "triage_timeout_sec",
            45.0,
            _describe(
                "How long to wait for an interpretation. On a timeout the "
                "anomaly goes unanswered and the task stays exactly as it was."
            ),
        )
        self.declare_parameter(
            "default_task_capability",
            int(Capability.DRIVE),
            _describe(
                "Capability assumed for a task the escalation names but this "
                "node has never been told about. Only reached when no mission "
                "adapter has called own() for that task id."
            ),
        )

    def _validated_params(self) -> CoordinatorParams:
        # CoordinatorParams raises on an incoherent combination -- notably a
        # bid backoff that can outlast the announce window. Letting that
        # propagate out of the constructor is deliberate: the node refuses to
        # start rather than run an auction mechanism that cannot work.
        return CoordinatorParams(
            announce_window_sec=float(self.get_parameter("announce_window_sec").value),
            announce_repeat_sec=float(self.get_parameter("announce_repeat_sec").value),
            bid_max_backoff_sec=float(self.get_parameter("bid_max_backoff_sec").value),
            bid_jitter_fraction=float(self.get_parameter("bid_jitter_fraction").value),
            bid_memory_sec=float(self.get_parameter("bid_memory_sec").value),
            max_delegation_attempts=int(
                self.get_parameter("max_delegation_attempts").value
            ),
            redelegation_cooldown_sec=float(
                self.get_parameter("redelegation_cooldown_sec").value
            ),
            peer_timeout_sec=float(self.get_parameter("peer_timeout_sec").value),
            heartbeat_period_sec=float(
                self.get_parameter("heartbeat_period_sec").value
            ),
            settled_retention_sec=float(
                self.get_parameter("settled_retention_sec").value
            ),
            max_open_auctions=int(self.get_parameter("max_open_auctions").value),
        )

    def _validated_capabilities(self) -> "tuple[int, ...]":
        names = list(self.get_parameter("capabilities").value or [])
        capabilities = []
        for name in names:
            try:
                capabilities.append(Capability[str(name).strip().upper()])
            except KeyError:
                # Refused rather than skipped. A typo that silently drops SPRAY
                # produces a robot that never bids on spraying and no error
                # anywhere -- the hardest kind of misconfiguration to find.
                raise ValueError(
                    f"unknown capability '{name}'; known: "
                    f"{', '.join(c.name for c in Capability)}"
                ) from None
        if not capabilities:
            self.get_logger().warn(
                "no capabilities declared: this robot will never bid on anything"
            )
        # Built once here so an out-of-range index fails at startup.
        cap_mask(*capabilities)
        return tuple(capabilities)

    # ------------------------------------------------------------------
    # Plumbing
    # ------------------------------------------------------------------

    def _on_preempt_ack(self, msg: Bool) -> None:
        """The behaviour tree has yielded and dealt with the event.

        An acknowledgement, not a handshake: nothing here waits for it, and a
        tree that never sends one leaves the flag raised, which is the safe
        direction -- a flag stuck up costs a re-evaluation per tick, a flag
        stuck down loses a coordination event.
        """
        if msg.data:
            self.session.clear_preemption()

    # ------------------------------------------------------------------
    # The owner role's entry point
    # ------------------------------------------------------------------

    def _on_infeasible(self, msg: String) -> None:
        """Local recovery has run out. Get an interpretation, then act on it.

        Handed straight to a thread. Everything after this point blocks: the
        triage agent is a language model and takes seconds, and this callback
        is on the same executor as the tick timer that drives auction windows,
        bid backoffs and heartbeats. Blocking here would stall coordination for
        the length of a model call, which the fleet reads as this robot dying.
        """
        payload = self._parse_escalation(msg.data)
        if payload is None:
            return

        task_id = int(payload.get("task_id") or 0)
        with self._in_flight_lock:
            # One interpretation per task at a time. The planner and the
            # arbiter can both give up on the same fault within a second of
            # each other, and two model calls would race to announce it twice.
            if task_id in self._in_flight:
                self.get_logger().info(
                    f"already interpreting task {task_id}; ignoring the repeat"
                )
                return
            self._in_flight.add(task_id)

        Thread(
            target=self._interpret_and_apply,
            args=(task_id, payload),
            daemon=True,
        ).start()

    def _interpret_and_apply(self, task_id: int, payload: dict) -> None:
        try:
            task = self._task_for(task_id)
            detail = str(payload.get("detail", "")) or "local recovery exhausted"
            context = self.session.anomaly_context(task, detail=detail)

            action = None
            if self._triage is not None:
                try:
                    action = self._triage.interpret_anomaly(context)
                    self.get_logger().info(
                        f"triage says {type(action).__name__} for task {task_id}"
                    )
                except Exception as exc:  # noqa: BLE001 - a model, a timeout, a parse
                    # Deliberately not falling back to the stub interpreter. A
                    # robot that sheds a task because the agent was unreachable
                    # has made a fleet-wide decision on no evidence; leaving the
                    # task owned by a robot that knows it is stuck is worse for
                    # throughput and better for everything else.
                    self.get_logger().error(
                        f"no interpretation for task {task_id}, leaving it with "
                        f"us: {exc}"
                    )
                    return

            # action is None only when the triage agent is disabled, in which
            # case report_infeasible falls through to the injected interpreter.
            self.session.report_infeasible(task, detail=detail, action=action)
        finally:
            with self._in_flight_lock:
                self._in_flight.discard(task_id)

    def _task_for(self, task_id: int) -> Optional[Task]:
        """The task the escalation is about, as this node understands it.

        Prefers what a mission adapter already told us via ``own()``: that
        record has the real capability and location. Falling back to a
        synthesised task is the seam where the mission stack is not wired yet
        -- it is enough to interpret and announce, and the location will be
        wrong until a real MissionInterface supplies it.
        """
        if task_id <= 0:
            return None
        known = self.session.task(task_id)
        if known is not None:
            return known.task
        here = Location(0, 0)
        return Task(
            task_id=task_id,
            required_capability=self._default_capability,
            location=here,
            priority=0,
        )

    def _parse_escalation(self, data: str) -> Optional[dict]:
        try:
            payload = json.loads(data)
        except json.JSONDecodeError:
            self.get_logger().error("could not parse the infeasibility payload")
            return None
        if not isinstance(payload, dict):
            self.get_logger().error(
                f"infeasibility payload must be a JSON object, got "
                f"{type(payload).__name__}"
            )
            return None
        return payload

    def _now(self) -> float:
        """Monotonic seconds from the ROS clock.

        The ROS clock rather than time.monotonic so that ``use_sim_time`` makes
        auction windows and bid backoffs follow simulation time along with
        everything else -- the same reason the reliability layer does it.
        """
        return self.get_clock().now().nanoseconds / 1e9

    def _log_event(self, level: str, message: str) -> None:
        # Two call sites, not one dispatched getattr: rclpy keys logger state
        # on the caller's line and raises if one line logs at two severities.
        if level == "warn":
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def stats(self) -> dict:
        return self.session.stats()

    def _log_stats(self) -> None:
        self.get_logger().info(
            " ".join(f"{k}={v}" for k, v in sorted(self.stats().items()))
        )


def main(args: Optional[list] = None) -> None:
    """Run the coordinator with its reliability layer in the same process.

    Two nodes, one executor. The executor is multi-threaded because the
    reliability layer resolves a GRANT's future from its own retransmit tick
    and the coordinator's outcome handler runs there -- on a single-threaded
    executor that handler would be serialised behind whatever else was
    scheduled, which is exactly the latency the ownership rule is sensitive to.
    """
    rclpy.init(args=args)
    reliability = ReliabilityNode()
    coordinator = CoordinatorNode(reliability=reliability)
    executor = MultiThreadedExecutor()
    executor.add_node(reliability)
    executor.add_node(coordinator)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or SIGTERM from launch. Ordinary ways to stop, not errors.
        pass
    finally:
        coordinator.destroy_node()
        reliability.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
