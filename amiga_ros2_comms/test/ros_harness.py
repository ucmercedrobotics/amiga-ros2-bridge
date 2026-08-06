#!/usr/bin/env python3
"""Shared scaffolding for the tests that spin real ROS nodes.

Kept out of conftest.py because these are classes and functions rather than
fixtures, and both the bridge acceptance tests and the simulator tests need
them.
"""

import os
import sys
import threading
import time

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from amiga_interfaces.msg import LoRaFrame

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.lora.bridge_node import LoRaBridge  # noqa: E402


class Harness:
    """Runs a set of nodes on a background executor for the life of a test."""

    def __init__(self, *nodes):
        self.nodes = nodes
        self._executor = MultiThreadedExecutor(num_threads=4)
        for node in nodes:
            self._executor.add_node(node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._running = True
        self._thread.start()

    def _spin(self):
        while self._running:
            self._executor.spin_once(timeout_sec=0.05)

    def close(self):
        self._running = False
        self._thread.join(timeout=3.0)
        for node in self.nodes:
            if isinstance(node, LoRaBridge):
                node.shutdown()
            self._executor.remove_node(node)
            node.destroy_node()


def make_bridge(port, namespace, **params):
    """A bridge on ``port``, namespaced so several can run at once."""
    settings = {
        "serial_port": port,
        "baud": 115200,
        "max_payload_bytes": 200,
        "tx_queue_depth": 32,
        "rx_queue_depth": 64,
        "stats_period_sec": 0.0,
    }
    settings.update(params)
    return LoRaBridge(
        namespace=namespace,
        parameter_overrides=[Parameter(k, value=v) for k, v in settings.items()],
    )


class Collector(Node):
    """Publishes to one bridge's /lora/tx and records another's /lora/rx."""

    def __init__(self, tx_namespace, rx_namespace, name="collector"):
        super().__init__(name)
        self.received = []
        self._lock = threading.Lock()
        self.pub = self.create_publisher(LoRaFrame, f"/{tx_namespace}/lora/tx", 100)
        self.create_subscription(
            LoRaFrame, f"/{rx_namespace}/lora/rx", self._on_rx, 200
        )

    def _on_rx(self, msg):
        with self._lock:
            self.received.append(
                (bytes(msg.data), msg.has_link_stats, msg.rssi, msg.snr)
            )

    def payloads(self):
        with self._lock:
            return [item[0] for item in self.received]

    def send(self, payload, priority=LoRaFrame.PRIORITY_URGENT):
        msg = LoRaFrame()
        msg.data = list(payload)
        msg.priority = priority
        self.pub.publish(msg)


def wait_until(predicate, timeout=10.0, interval=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()
