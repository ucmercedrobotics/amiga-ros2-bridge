#!/usr/bin/env python3
"""The reliability node over real topics, and over the real transport spine.

test_reliability.py proves the logic against a mock link. This file proves the
wiring: that the node's parameters reach the session, that its retransmit timer
actually fires under a ROS executor, and that a GRANT survives a round trip
through two bridges and a serial cable -- bridge, codec and reliability
together, with nothing mocked but the radio itself.

Two robots, two reliability nodes, two bridges, one virtual serial pair::

    reliability(left) -> /left/lora/tx -> bridge(left) -> pty
                                                           |
    reliability(right) <- /right/lora/rx <- bridge(right) <-+

Slower than the pure tests -- real timers, real serial, real executor -- so
there are a handful of them, covering what only a running node can show.
"""

import os
import sys
import threading

import pytest
from rclpy.parameter import Parameter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_interfaces.msg import LoRaFrame  # noqa: E402
from rclpy.node import Node  # noqa: E402

from amiga_ros2_comms.codec import (  # noqa: E402
    Capability,
    Grant,
    ReasonCode,
    TaskAnnounce,
    decode,
    encode,
)
from amiga_ros2_comms.lora.airtime import RadioConfig  # noqa: E402
from amiga_ros2_comms.reliability import Outcome, ReliabilityParams  # noqa: E402
from amiga_ros2_comms.reliability.node import (  # noqa: E402
    TIMEOUT_SAFETY_FACTOR,
    ReliabilityNode,
    round_trip_floor_sec,
    timeout_shortfall,
)
from ros_harness import Harness, make_bridge, wait_until  # noqa: E402

LEFT = 1
RIGHT = 2


def make_reliability(node_id, namespace, **params):
    settings = {
        "node_id": node_id,
        "retransmit_timeout_sec": 0.5,
        "max_retries": 4,
        "retransmit_backoff": 1.0,
        "max_retransmit_timeout_sec": 5.0,
        "tick_period_sec": 0.05,
        "stats_period_sec": 0.0,
    }
    settings.update(params)
    return ReliabilityNode(
        namespace=namespace,
        parameter_overrides=[Parameter(k, value=v) for k, v in settings.items()],
    )


class Inbox:
    """Thread-safe collector for on_deliver, which runs on an executor thread."""

    def __init__(self):
        self._lock = threading.Lock()
        self.messages = []

    def __call__(self, msg):
        with self._lock:
            self.messages.append(msg)

    def __len__(self):
        with self._lock:
            return len(self.messages)

    def snapshot(self):
        with self._lock:
            return list(self.messages)


class LossyRelay(Node):
    """Carries frames between two namespaces, dropping some on request.

    Stands in for the pair of bridges when the test is about this layer rather
    than about the serial hop, and unlike a real radio it can be told exactly
    which frame to lose.
    """

    def __init__(self, left="left", right="right"):
        super().__init__("lossy_relay")
        self.drop_rule = lambda payload, sender: False
        self.carried = 0
        self.dropped = 0
        self._lock = threading.Lock()
        self._pubs = {
            "left": self.create_publisher(LoRaFrame, f"/{right}/lora/rx", 100),
            "right": self.create_publisher(LoRaFrame, f"/{left}/lora/rx", 100),
        }
        self.create_subscription(
            LoRaFrame, f"/{left}/lora/tx", lambda m: self._relay(m, "left"), 100
        )
        self.create_subscription(
            LoRaFrame, f"/{right}/lora/tx", lambda m: self._relay(m, "right"), 100
        )

    def _relay(self, msg, sender):
        payload = bytes(msg.data)
        with self._lock:
            if self.drop_rule(payload, sender):
                self.dropped += 1
                return
            self.carried += 1
        out = LoRaFrame()
        out.data = list(payload)
        self._pubs[sender].publish(out)

    def drop_first(self, count, kind=None):
        """Drop the first ``count`` frames, optionally only of one type."""
        seen = {"n": 0}

        def rule(payload, sender):
            if kind is not None and type(decode(payload)).__name__ != kind:
                return False
            seen["n"] += 1
            return seen["n"] <= count

        self.drop_rule = rule


def a_grant(winner=RIGHT, task_id=7):
    return Grant(src=0, seq=0, task_id=task_id, winner_id=winner)


def an_announce(task_id=7):
    return TaskAnnounce(
        src=0,
        seq=0,
        task_id=task_id,
        req_capability=Capability.SPRAY,
        grid_row=3,
        grid_col=4,
        priority=200,
        reason_code=ReasonCode.OPERATOR_REQUEST,
    )


# --------------------------------------------------------------------------
# Over topics, with a relay that can lose frames on demand
# --------------------------------------------------------------------------


def test_a_grant_is_delivered_and_confirmed_over_real_topics():
    left = make_reliability(LEFT, "left")
    right = make_reliability(RIGHT, "right")
    inbox = Inbox()
    right.set_on_deliver(inbox)
    relay = LossyRelay()
    harness = Harness(left, right, relay)
    try:
        future = left.send_reliable(RIGHT, a_grant())

        assert wait_until(future.done, timeout=5.0), "no outcome inside 5s"
        assert future.result() is Outcome.DELIVERED
        assert wait_until(lambda: len(inbox) == 1)
        assert inbox.snapshot()[0].task_id == 7
    finally:
        harness.close()


def test_the_retransmit_timer_fires_under_a_real_executor():
    left = make_reliability(LEFT, "left")
    right = make_reliability(RIGHT, "right")
    inbox = Inbox()
    right.set_on_deliver(inbox)
    relay = LossyRelay()
    relay.drop_first(2, kind="Grant")
    harness = Harness(left, right, relay)
    try:
        future = left.send_reliable(RIGHT, a_grant())

        assert wait_until(future.done, timeout=8.0)
        assert future.result() is Outcome.DELIVERED
        assert left.stats()["tx_retransmits"] >= 2
        assert len(inbox) == 1, "retransmits collapsed to one delivery"
    finally:
        harness.close()


def test_a_grant_nobody_can_hear_fails_rather_than_hangs():
    left = make_reliability(LEFT, "left", retransmit_timeout_sec=0.2, max_retries=2)
    right = make_reliability(RIGHT, "right")
    relay = LossyRelay()
    relay.drop_rule = lambda payload, sender: True
    harness = Harness(left, right, relay)
    try:
        future = left.send_reliable(RIGHT, a_grant())

        assert wait_until(future.done, timeout=8.0), "the node hung"
        assert future.result() is Outcome.FAILED
        assert left.stats()["tx_failed"] == 1
    finally:
        harness.close()


def test_a_broadcast_reaches_the_peer_and_is_never_acked():
    left = make_reliability(LEFT, "left")
    right = make_reliability(RIGHT, "right")
    inbox = Inbox()
    right.set_on_deliver(inbox)
    relay = LossyRelay()
    harness = Harness(left, right, relay)
    try:
        assert left.send_broadcast(an_announce())

        assert wait_until(lambda: len(inbox) == 1)
        # Give any (incorrect) ACK or retransmit time to appear.
        assert not wait_until(lambda: right.stats()["tx_acks"] > 0, timeout=1.0)
        assert left.stats()["tx_retransmits"] == 0
        assert left.session.pending == 0
    finally:
        harness.close()


def test_a_repeated_broadcast_is_delivered_once():
    left = make_reliability(LEFT, "left")
    right = make_reliability(RIGHT, "right")
    inbox = Inbox()
    right.set_on_deliver(inbox)
    relay = LossyRelay()
    harness = Harness(left, right, relay)
    try:
        # The same announce object re-sent gets a fresh seq each time, which is
        # correct but is not the case dedup is for. Replay the exact payload
        # instead, as a retransmitting peer would.
        announce = an_announce()
        left.send_broadcast(announce)
        assert wait_until(lambda: len(inbox) == 1)

        replay = right.create_publisher(LoRaFrame, "lora/rx", 10)
        frame = LoRaFrame()
        frame.data = list(encode(announce))
        for _ in range(5):
            replay.publish(frame)

        assert not wait_until(lambda: len(inbox) > 1, timeout=1.5)
        assert right.stats()["rx_duplicates"] >= 5
    finally:
        harness.close()


# --------------------------------------------------------------------------
# The whole spine: reliability -> codec -> bridge -> serial -> back up
# --------------------------------------------------------------------------


def test_a_grant_survives_the_full_transport_spine(serial_pair):
    left_bridge = make_bridge(serial_pair.port_a, "left")
    right_bridge = make_bridge(serial_pair.port_b, "right")
    left = make_reliability(LEFT, "left", retransmit_timeout_sec=1.0)
    right = make_reliability(RIGHT, "right", retransmit_timeout_sec=1.0)
    inbox = Inbox()
    right.set_on_deliver(inbox)
    harness = Harness(left_bridge, right_bridge, left, right)
    try:
        future = left.send_reliable(RIGHT, a_grant(task_id=1234))

        assert wait_until(future.done, timeout=15.0), "no outcome over real serial"
        assert future.result() is Outcome.DELIVERED

        assert wait_until(lambda: len(inbox) == 1)
        grant = inbox.snapshot()[0]
        assert grant.task_id == 1234
        assert grant.src == LEFT
        assert grant.winner_id == RIGHT
    finally:
        harness.close()


def test_many_messages_cross_the_spine_without_duplication(serial_pair):
    left_bridge = make_bridge(serial_pair.port_a, "left")
    right_bridge = make_bridge(serial_pair.port_b, "right")
    left = make_reliability(LEFT, "left")
    right = make_reliability(RIGHT, "right")
    inbox = Inbox()
    right.set_on_deliver(inbox)
    harness = Harness(left_bridge, right_bridge, left, right)
    try:
        for task_id in range(1, 21):
            left.send_broadcast(an_announce(task_id))

        assert wait_until(lambda: len(inbox) == 20, timeout=15.0)
        assert [m.task_id for m in inbox.snapshot()] == list(range(1, 21))
        assert right.stats()["rx_duplicates"] == 0
    finally:
        harness.close()


# --------------------------------------------------------------------------
# Parameters
# --------------------------------------------------------------------------


def test_a_node_id_outside_the_address_space_is_refused():
    # No safe default exists: a wrong ID corrupts other robots' dedup state as
    # well as this one's, so the node refuses to start rather than guess.
    with pytest.raises(ValueError, match="node_id"):
        make_reliability(0, "bad")


def test_the_round_trip_floor_grows_with_the_spreading_factor():
    # The floor under retransmit_timeout_sec is a property of the link, not of
    # the CPU, and it moves by more than an order of magnitude across the SF
    # range the fleet might actually use.
    fast = round_trip_floor_sec(RadioConfig(spreading_factor=7))
    slow = round_trip_floor_sec(RadioConfig(spreading_factor=12))

    assert 0 < fast < slow
    assert slow > 10 * fast


def test_a_timeout_below_the_round_trip_floor_is_complained_about():
    slow = RadioConfig(spreading_factor=12)
    # A timeout that would be generous on a fast link and hopeless on this one.
    assert timeout_shortfall(0.5, slow) is not None
    assert "round trip" in timeout_shortfall(0.5, slow)


def test_a_timeout_with_headroom_draws_no_complaint():
    fast = RadioConfig(spreading_factor=7)
    floor = round_trip_floor_sec(fast)
    assert timeout_shortfall(floor * TIMEOUT_SAFETY_FACTOR, fast) is None


def test_the_default_timeout_holds_across_the_practical_spreading_factors():
    # A fleet may be retuned for range after this node ships, so the default
    # has to survive the SF range it could plausibly land on. Time on air
    # doubles per step, so SF11 and SF12 cannot share a default with SF7 and
    # are expected to warn instead -- which is the point of the warning.
    default = ReliabilityParams().retransmit_timeout_sec

    for sf in range(7, 11):
        assert timeout_shortfall(default, RadioConfig(spreading_factor=sf)) is None, (
            f"the shipped default is too short at SF{sf}"
        )
    for sf in (11, 12):
        assert timeout_shortfall(default, RadioConfig(spreading_factor=sf)) is not None
