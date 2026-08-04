#!/usr/bin/env python3
"""What only a running node can show. Everything else is in test_coordinator.py.

Three things need a real executor and cannot be asserted against the engine:
that parameters actually reach the session, that the tick timer drives the
deadlines the engine only exposes as a ``tick()`` method, and that the
preemption flag reaches a subscriber as a latched topic.

The fourth is the one worth the setup cost: **a task crossing two robots**.
Two coordinators, two reliability layers, a relay standing in for the radio,
and nothing else faked but nav and the mission stack -- an announcement that
becomes a bid that becomes a GRANT that becomes a confirmed transfer, with the
real state machine on both ends and real ROS in between.
"""

import os
import sys
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_interfaces.msg import LoRaFrame  # noqa: E402
from amiga_ros2_comms.codec import Capability  # noqa: E402
from amiga_ros2_comms.reliability.node import ReliabilityNode  # noqa: E402

from amiga_ros2_coordinator.model import Location, Task, TaskState  # noqa: E402
from amiga_ros2_coordinator.node import CoordinatorNode  # noqa: E402
from amiga_ros2_coordinator.reasoning import (  # noqa: E402
    AcceptEverything,
    ScriptedInterpreter,
)
from amiga_ros2_coordinator.schema import ReDelegate  # noqa: E402
from fakes import FakeMission, FakeNav  # noqa: E402

TASK_ID = 21

# Short everywhere, because these run against the wall clock rather than a
# clock the test owns. The relationships the engine cares about are preserved:
# the backoff still fits inside the window with room to spare.
FAST_PARAMS = {
    "announce_window_sec": 1.0,
    "announce_repeat_sec": 0.4,
    "bid_max_backoff_sec": 0.2,
    "bid_memory_sec": 10.0,
    "heartbeat_period_sec": 0.5,
    "tick_period_sec": 0.02,
    "stats_period_sec": 0.0,
}


def a_task():
    return Task(
        task_id=TASK_ID,
        required_capability=Capability.SPRAY,
        location=Location(row=11, col=12),
        priority=100,
    )


def _params(**overrides):
    return [Parameter(k, value=v) for k, v in overrides.items()]


class FrameRelay(Node):
    """Carries frames between two namespaced bridges. The radio, minus the radio.

    Deliberately not a broadcast medium: two robots is what the test needs, and
    a general simulated medium already exists in amiga_ros2_comms for the cases
    that need more.
    """

    def __init__(self, left: str, right: str):
        super().__init__("frame_relay")
        self._to_right = self.create_publisher(LoRaFrame, f"/{right}/lora/rx", 32)
        self._to_left = self.create_publisher(LoRaFrame, f"/{left}/lora/rx", 32)
        self.create_subscription(
            LoRaFrame, f"/{left}/lora/tx", lambda m: self._to_right.publish(m), 32
        )
        self.create_subscription(
            LoRaFrame, f"/{right}/lora/tx", lambda m: self._to_left.publish(m), 32
        )


class Robot:
    """One robot's pair of nodes plus the fakes standing in for its stack."""

    def __init__(self, namespace, node_id, capabilities, interpreter=None, **overrides):
        self.nav = FakeNav(eta_sec=45.0, location=Location(node_id, node_id))
        self.mission = FakeMission(battery=88)
        self.replanner = AcceptEverything()
        self.reliability = ReliabilityNode(
            namespace=namespace,
            parameter_overrides=_params(
                node_id=node_id, retransmit_timeout_sec=0.3, tick_period_sec=0.02
            ),
        )
        settings = dict(FAST_PARAMS)
        settings.update(overrides)
        self.coordinator = CoordinatorNode(
            reliability=self.reliability,
            nav=self.nav,
            mission=self.mission,
            interpreter=interpreter,
            replanner=self.replanner,
            namespace=namespace,
            parameter_overrides=_params(
                capabilities=[c.name for c in capabilities], **settings
            ),
        )

    @property
    def session(self):
        return self.coordinator.session

    def nodes(self):
        return (self.reliability, self.coordinator)

    def destroy(self):
        self.coordinator.destroy_node()
        self.reliability.destroy_node()


class Spinner:
    """A background executor, so a test can wait on a condition instead of a sleep.

    Single-threaded, where ``main()`` uses a multi-threaded executor. Not an
    oversight: ``spin_once`` on a MultiThreadedExecutor hands the callback to a
    worker pool and returns, so stopping the spin thread does not stop the
    callbacks, and teardown races the timers on nodes the test is destroying.
    One thread makes teardown deterministic and exercises every callback path
    here unchanged -- including the delivery-outcome handler running inside the
    reliability layer's own tick, which is the ordering the ownership rule is
    sensitive to.
    """

    def __init__(self, nodes):
        self.executor = SingleThreadedExecutor()
        for node in nodes:
            self.executor.add_node(node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._stop = threading.Event()
        self._thread.start()

    def _spin(self):
        while not self._stop.is_set():
            self.executor.spin_once(timeout_sec=0.05)

    def until(self, predicate, timeout=8.0, what="condition"):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        pytest.fail(f"timed out after {timeout}s waiting for {what}")

    def stop(self):
        """Wind down in the order that leaves nothing running on a dead node.

        Joining the spin thread is not enough on a MultiThreadedExecutor: it
        dispatches callbacks to its own worker pool, and one of those can still
        be inside a timer callback when spin_once returns. ``shutdown()`` is
        what waits for that pool, and it has to happen before the nodes are
        destroyed or a callback lands on a node that is already gone.
        """
        self._stop.set()
        self._thread.join(timeout=3.0)
        self.executor.shutdown()
        for node in list(self.executor.get_nodes()):
            self.executor.remove_node(node)


@pytest.fixture
def fleet():
    """Two robots and a relay, spun on a background executor."""
    task = a_task()
    owner = Robot(
        "owner",
        1,
        (Capability.DRIVE,),
        interpreter=ScriptedInterpreter([ReDelegate(task=task)]),
    )
    worker = Robot("worker", 2, (Capability.DRIVE, Capability.SPRAY))
    relay = FrameRelay("owner", "worker")
    spinner = Spinner([*owner.nodes(), *worker.nodes(), relay])
    yield owner, worker, task, spinner
    spinner.stop()
    owner.destroy()
    worker.destroy()
    relay.destroy_node()


# ==========================================================================
# Wiring
# ==========================================================================


def test_parameters_reach_the_session():
    reliability = ReliabilityNode(
        namespace="params", parameter_overrides=_params(node_id=9)
    )
    node = CoordinatorNode(
        reliability=reliability,
        namespace="params",
        parameter_overrides=_params(
            capabilities=["DRIVE", "SPRAY"],
            announce_window_sec=7.5,
            bid_max_backoff_sec=1.25,
            peer_timeout_sec=45.0,
            max_open_auctions=3,
        ),
    )
    try:
        params = node.session.params
        assert node.session.node_id == 9
        assert params.announce_window_sec == 7.5
        assert params.bid_max_backoff_sec == 1.25
        assert params.peer_timeout_sec == 45.0
        assert params.max_open_auctions == 3
        assert node.session.registry.timeout_sec == 45.0
        # Capabilities are bit *indices*: DRIVE is 0 and SPRAY is 2.
        assert node.session.cap_mask == 0b101
    finally:
        node.destroy_node()
        reliability.destroy_node()


def test_an_unknown_capability_is_refused_at_startup():
    """A typo that silently drops SPRAY is the hardest kind of bug to find."""
    reliability = ReliabilityNode(
        namespace="typo", parameter_overrides=_params(node_id=8)
    )
    try:
        with pytest.raises(ValueError, match="unknown capability"):
            CoordinatorNode(
                reliability=reliability,
                namespace="typo",
                parameter_overrides=_params(capabilities=["SPRY"]),
            )
    finally:
        reliability.destroy_node()


def test_an_incoherent_window_and_backoff_stop_the_node_starting():
    reliability = ReliabilityNode(
        namespace="bad", parameter_overrides=_params(node_id=7)
    )
    try:
        with pytest.raises(ValueError, match="announce_window_sec"):
            CoordinatorNode(
                reliability=reliability,
                namespace="bad",
                parameter_overrides=_params(
                    announce_window_sec=1.0, bid_max_backoff_sec=1.0
                ),
            )
    finally:
        reliability.destroy_node()


def test_the_tick_timer_drives_deadlines_without_anyone_calling_tick():
    """The engine exposes tick(); this is what proves something calls it."""
    reliability = ReliabilityNode(
        namespace="ticking", parameter_overrides=_params(node_id=6)
    )
    node = CoordinatorNode(
        reliability=reliability,
        nav=FakeNav(),
        mission=FakeMission(),
        namespace="ticking",
        parameter_overrides=_params(**FAST_PARAMS),
    )
    spinner = Spinner([reliability, node])
    try:
        spinner.until(
            lambda: node.session.stats()["heartbeats_sent"] >= 2,
            what="two heartbeats on the timer",
        )
    finally:
        spinner.stop()
        node.destroy_node()
        reliability.destroy_node()


def test_the_preemption_flag_is_published_latched():
    """Latched, so a behaviour tree that starts late still sees a raised flag."""
    reliability = ReliabilityNode(
        namespace="preempt", parameter_overrides=_params(node_id=5)
    )
    node = CoordinatorNode(
        reliability=reliability,
        nav=FakeNav(),
        mission=FakeMission(),
        namespace="preempt",
        parameter_overrides=_params(**FAST_PARAMS),
    )
    listener = rclpy.create_node("preempt_listener", namespace="preempt")
    received = []
    listener.create_subscription(
        Bool,
        "/preempt/coordinator/preempt_requested",
        lambda msg: received.append(msg.data),
        QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    spinner = Spinner([reliability, node, listener])
    try:
        spinner.until(lambda: received, what="the initial latched flag")
        assert received[-1] is False

        node.session._request_yield("a coordination event")
        spinner.until(lambda: received[-1] is True, what="the flag to be raised")
    finally:
        spinner.stop()
        listener.destroy_node()
        node.destroy_node()
        reliability.destroy_node()


# ==========================================================================
# Two robots, one task
# ==========================================================================


def test_a_task_crosses_two_robots_over_the_real_transport(fleet):
    owner, worker, task, spinner = fleet

    # Wait for a heartbeat to have crossed in *both* directions before starting
    # an auction. One direction only proves half the relay is discovered, and
    # an announcement sent into the undiscovered half is simply lost -- a
    # best-effort broadcast with one window and no retry after it. This is a
    # test-harness precondition, not a property of the coordinator: on a real
    # radio there is no discovery step to wait for.
    spinner.until(
        lambda: worker.session.node_id in [p.robot_id for p in owner.session.registry]
        and owner.session.node_id in [p.robot_id for p in worker.session.registry],
        what="a heartbeat to cross in both directions",
    )
    assert owner.session.registry.capable(Capability.SPRAY)

    owner.session.own(task)
    owner.session.report_infeasible(task, detail="no spray capability on this robot")

    spinner.until(
        lambda: owner.session.state_of(TASK_ID) is TaskState.TRANSFERRED,
        what="the task to be confirmed transferred",
    )

    # The owner shed it, and only after a confirmed delivery.
    assert [t.task_id for t in owner.mission.transferred] == [TASK_ID]
    assert owner.session.stats()["grants_delivered"] == 1

    # The worker took it on, and re-planned around it.
    assert [t.task_id for t in worker.mission.absorbed] == [TASK_ID]
    assert worker.replanner.calls == 1
    assert worker.session.state_of(TASK_ID) is TaskState.OURS

    # And nothing was hard-interrupted on either robot.
    assert owner.nav.hard_interrupts == []
    assert owner.mission.hard_interrupts == []
    assert worker.nav.hard_interrupts == []
    assert worker.mission.hard_interrupts == []
