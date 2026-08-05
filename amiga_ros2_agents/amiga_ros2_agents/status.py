"""
status.py

Every agent exposes a status snapshot on `/agents/<node_name>/status` as JSON.

This replaces the per-agent A2A status endpoint. The topic is TRANSIENT_LOCAL
with depth 1, so a subscriber that attaches late still receives the last value —
that's what polling an HTTP endpoint gave us for free. It also makes the topic a
readiness signal: the startup snapshot latches, so seeing a message means the
node is up and its callbacks are wired.
"""

import json
from typing import Dict

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String

STATUS_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class StatusPublisher:
    """Publishes an agent's status dict as JSON on /agents/<name>/status."""

    def __init__(self, node: Node):
        self._pub = node.create_publisher(
            String, f"/agents/{node.get_name()}/status", STATUS_QOS
        )

    def publish(self, status: Dict) -> None:
        msg = String()
        # default=str so a stray non-serialisable value degrades to its repr
        # rather than throwing from whatever callback happened to update state.
        msg.data = json.dumps(status, default=str)
        self._pub.publish(msg)
