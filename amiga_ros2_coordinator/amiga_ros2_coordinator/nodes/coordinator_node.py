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

from amiga_ros2_comms.codec import CAPABILITY_BY_ELEMENT as ELEMENT_CAPABILITY
from amiga_ros2_comms.codec import XML_ELEMENT as CAPABILITY_ELEMENT
from amiga_ros2_comms.codec import (
    Capability,
    ReasonCode,
    Target,
    TargetKind,
    cap_mask,
)
from amiga_ros2_comms.reliability.node import ReliabilityNode

from ..vocabulary.capabilities import (
    SchemaError,
    capabilities_from_xsd,
    default_schema_path,
    unknown_actions,
)
from ..engine.coordinator import CoordinatorParams, CoordinatorSession
from ..vocabulary.model import Task, capability_names
from ..adapters.triage_client import optional_client


def _describe(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


def _dynamic(text: str) -> ParameterDescriptor:
    """Descriptor for a list parameter that legitimately defaults to empty."""
    return ParameterDescriptor(description=text, dynamic_typing=True)


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

    def eta(self, target: Target) -> float:
        self._warn()
        raise RuntimeError("no navigation port wired")

    def can_reach(self, target: Target) -> bool:
        self._warn()
        return False

    def current_location(self) -> Optional[Target]:
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
        note_interpreter=None,
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
        from ..ports.reasoning import AlwaysReDelegate, IgnoreNotes

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
            replanner=(
                replanner if replanner is not None else self._default_replanner()
            ),
            capabilities=capabilities,
            preemption=self._preemption,
            params=params,
            clock=self._now,
            on_event=self._log_event,
            note_interpreter=(
                note_interpreter if note_interpreter is not None else IgnoreNotes()
            ),
        )
        reliability.set_on_deliver(self.session.on_message)
        reliability.set_on_note(self.session.on_note)

        tick_period = float(self.get_parameter("tick_period_sec").value)
        self._tick_timer = self.create_timer(tick_period, self.session.tick)
        # Separate from the tick, on the blocking group: draining this runs a
        # model call per note, and it must not sit in the same callback as the
        # state machine's own timer. The session hands out contexts and takes
        # answers back; nothing between those two points holds its lock.
        self._note_timer = self.create_timer(
            tick_period,
            self._drain_note_requests,
            callback_group=self._blocking_group,
        )

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
        self._default_capabilities = cap_mask(
            *(
                ELEMENT_CAPABILITY[str(name).strip()]
                for name in self.get_parameter("default_task_capabilities").value or []
                if str(name).strip() in ELEMENT_CAPABILITY
            )
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
            f"capabilities={capability_names(cap_mask(*capabilities))}, "
            f"announce window {params.announce_window_sec}s, "
            f"bid backoff <={params.bid_max_backoff_sec}s, "
            f"peer timeout {params.peer_timeout_sec}s, "
            f"heartbeat every {params.heartbeat_period_sec}s, tick={tick_period}s"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _default_replanner(self):
        """The verification client, or the pass-through stub if it is off.

        Explicitly injected replanners win: the acceptance tests pin the state
        machine against scripted stubs, and they must not start needing an
        arbiter to run.
        """
        from ..ports.reasoning import AcceptEverything

        if not bool(self.get_parameter("verify_replans").value):
            self.get_logger().warn(
                "verify_replans is false — mission changes will be accepted "
                "without being checked against the mission's LTL specification"
            )
            return AcceptEverything()

        from ..adapters.replanner_client import VerifyingReplanner

        return VerifyingReplanner(
            self,
            on_rejected=self._on_replan_rejected,
            require_verifier=bool(self.get_parameter("require_verifier").value),
        )

    def _on_replan_rejected(self, task, reason: str) -> None:
        """A mission change we already committed does not verify.

        Runs on the verification thread, off the coordinator's lock, which is
        why it can go through the ordinary public entry point:
        ``report_infeasible`` takes that lock itself.

        Work this robot holds and cannot legitimately do is exactly what the
        anomaly path exists for, and it is the only route that can offer the
        task back to the fleet. Identical in effect to the inline rejection
        branch in ``_take_on``; it just arrives once the verifier has answered.
        """
        self.session.report_infeasible(
            task=task,
            detail=f"replan rejected by verification: {reason}",
            reason_code=ReasonCode.TASK_FAILED,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "verify_replans",
            True,
            _describe(
                "Send every mission change to the arbiter's "
                "/mission/verify_replan to be re-verified against the mission's "
                "LTL specification before it is committed. Turning this off "
                "makes replan-and-verify a no-op that accepts everything, which "
                "is a coordinator whose transfers nothing checks."
            ),
        )
        self.declare_parameter(
            "require_verifier",
            False,
            _describe(
                "What to do when the arbiter is not reachable. False accepts "
                "the change and records it as unverified, so a coordinator can "
                "run without the agent stack. True refuses it, which is what a "
                "run whose results depend on verification wants -- a mission "
                "that completed unchecked must not be counted as one that "
                "passed."
            ),
        )
        self.declare_parameter(
            "mission_schema",
            default_schema_path() or "",
            _describe(
                "The behaviour-tree mission schema this robot validates against "
                "-- the same amiga_btcpp.xsd bt_runner resolves. Its ActionGroup "
                "is what this robot advertises it can do, because an action the "
                "schema forbids is an action no mission can ever contain. Empty "
                "falls back to the 'capabilities' parameter."
            ),
        )
        self.declare_parameter(
            # Empty by default: the schema is the answer, and a non-empty
            # default here would silently win over it on every robot. The
            # descriptor allows dynamic typing because rclpy infers an empty
            # list as BYTE_ARRAY, and a statically typed empty default would
            # then reject the very strings this parameter exists to take.
            "capabilities",
            [],
            _dynamic(
                "Override for what this robot advertises, as behaviour-tree "
                "action names (MoveToTreeID, MoveToAisleHead, "
                "MoveToGPSLocation, ApproachGPSWaypoint, "
                "MoveToRelativeLocation, OrientRobotHeading, FollowPerson, "
                "SampleLeaf, MoveArmToPosition). For a robot whose hardware is "
                "a subset of what its schema allows -- an arm removed for "
                "maintenance. Empty means read mission_schema."
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
            "default_task_capabilities",
            [CAPABILITY_ELEMENT[Capability.MOVE_TO_TREE_ID]],
            _describe(
                "Actions assumed necessary for a task the escalation names "
                "without describing. Only reached when the triage agent could "
                "not resolve the failing node to a subtree in /mission/xml -- "
                "an ordinary escalation carries the real action set."
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
        """What this robot advertises: the schema's ActionGroup, or an override.

        The schema first, because it is the artifact that decides what missions
        this robot can be given. The parameter is for the case the schema
        cannot express -- hardware temporarily absent from a robot whose schema
        still permits the action.
        """
        override = self._capabilities_parameter()
        if override:
            self.get_logger().info(
                "capabilities set explicitly, overriding the mission schema: "
                f"{[CAPABILITY_ELEMENT[c] for c in override]}"
            )
            return override

        schema = str(self.get_parameter("mission_schema").value or "")
        if not schema:
            # No schema and no override. Refused rather than defaulted: a robot
            # that quietly advertises nothing never bids and reports no error,
            # which is the hardest misconfiguration there is to find.
            raise ValueError(
                "neither mission_schema nor capabilities is set, so this robot "
                "has no way to know what it can do. Point mission_schema at the "
                "installed amiga_btcpp.xsd, or list capabilities explicitly."
            )

        try:
            capabilities = capabilities_from_xsd(schema)
        except SchemaError as exc:
            raise ValueError(f"cannot determine capabilities: {exc}") from None

        unknown = unknown_actions(schema)
        if unknown:
            # Not fatal -- a newer schema naming actions this build has no bit
            # for is legitimate -- but not silent either: work using them can
            # never be delegated in either direction.
            self.get_logger().warn(
                f"{schema} permits {list(unknown)}, which this build has no "
                f"capability bit for; work using them cannot be delegated"
            )
        self.get_logger().info(
            f"capabilities read from {schema}: "
            f"{[CAPABILITY_ELEMENT[c] for c in capabilities]}"
        )
        # Built once here so an out-of-range index fails at startup.
        cap_mask(*capabilities)
        return tuple(capabilities)

    def _capabilities_parameter(self) -> "tuple[int, ...]":
        """The explicit override, by XML element name. Empty when unset."""
        names = list(self.get_parameter("capabilities").value or [])
        capabilities = []
        for name in names:
            capability = ELEMENT_CAPABILITY.get(str(name).strip())
            if capability is None:
                # Refused rather than skipped. A typo that silently drops
                # SampleLeaf produces a robot that never bids on sampling and
                # no error anywhere.
                raise ValueError(
                    f"unknown capability '{name}'; known: "
                    f"{', '.join(sorted(ELEMENT_CAPABILITY))}"
                )
            capabilities.append(capability)
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

    def _drain_note_requests(self) -> None:
        """Interpret the notes waiting on it, off the session's lock.

        The bidder-side counterpart of ``_on_infeasible``: the session queues a
        context, this runs the model, and the answer goes back through
        ``revise_bid`` where the closed union is enforced. Nothing between the
        drain and the answer holds the state machine's lock, which is the whole
        reason the queue exists -- ``on_message`` and ``tick`` need that lock,
        and a bidder that stopped answering heartbeats for the length of a model
        call would be written off as dead by the fleet it is bidding into.
        """
        requests = self.session.take_note_requests()
        if not requests:
            return
        interpreter = self.session.note_interpreter()
        for context in requests:
            try:
                revision = interpreter.interpret_note(context)
            except Exception as exc:  # noqa: BLE001 - a model, a parser, anything
                # The bid still goes out, costed on mechanics alone. An
                # uninterpretable note leaves the auction working the way it
                # worked before notes existed, which is the safe direction.
                self.get_logger().warn(
                    f"interpret_note failed for task {context.task_id}, "
                    f"bidding unrevised: {exc}"
                )
                continue
            self.session.revise_bid(context.task_id, revision)

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
            task = self._task_for(task_id, payload)
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

    def _task_for(self, task_id: int, payload: dict) -> Optional[Task]:
        """The task the escalation is about, as this node understands it.

        Three sources, in order of how much they know:

        1. A record this session already owns, from a previous ``own()``.
        2. The escalation payload. The triage agent resolved the failing
           behaviour-tree node to a subtree of the running ``/mission/xml``, so
           it knows the action set and the target -- and it is the only thing in
           the system that does, because it is the only thing that reads the
           mission. This node deliberately never parses XML.
        3. A placeholder, when the agent could not resolve the node either.
           Undelegable by construction: its target is ``NONE``, so the
           coordinator will fall back to local disposition rather than announce
           work at a place nobody named.
        """
        if task_id <= 0:
            return None
        known = self.session.task(task_id)
        if known is not None:
            return known.task

        described = self._task_from_payload(task_id, payload)
        if described is not None:
            return described

        self.get_logger().warn(
            f"escalation for task {task_id} describes no work: no action set "
            f"and no target. It can be interpreted but not announced."
        )
        return Task(
            task_id=task_id,
            required_capabilities=self._default_capabilities,
            location=Target.none(),
            priority=0,
        )

    def _task_from_payload(self, task_id: int, payload: dict) -> Optional[Task]:
        """Rebuild the task the triage agent described, or None if it did not.

        Refuses a partial description rather than filling in the gaps. A task
        assembled half from the wire and half from a default is a task whose
        announcement means something nobody decided.
        """
        try:
            kind = payload["target_kind"]
            capabilities = int(payload["capabilities"])
        except (KeyError, TypeError, ValueError):
            return None
        try:
            return Task(
                task_id=task_id,
                required_capabilities=capabilities,
                location=Target(
                    kind=TargetKind(int(kind)),
                    a=int(payload.get("target_a", 0)),
                    b=int(payload.get("target_b", 0)),
                ),
                priority=int(payload.get("priority", 0)),
            )
        except (ValueError, TypeError) as exc:
            self.get_logger().error(
                f"escalation for task {task_id} describes work this fleet "
                f"cannot represent, ignoring the description: {exc}"
            )
            return None

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
