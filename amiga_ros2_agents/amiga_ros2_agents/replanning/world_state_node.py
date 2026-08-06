"""
world_state_node.py

Aggregates robot context from action feedback/status topics into a thread-safe
RobotContext, and republishes it as JSON on /world_state at TELEMETRY_RATE.

The mission planner subscribes to /world_state and keeps a rolling window of the
most recent frames, so a replan reads whatever telemetry has already arrived
instead of blocking on a request.
"""

import json
from dataclasses import asdict, dataclass, field
from threading import Lock

from action_msgs.msg import GoalStatusArray
from amiga_navigation_interfaces.action import TreeIDWaypoint
from kortex_interfaces.action import SegmentLeaves
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from std_msgs.msg import String

from ..runtime import spin
from ..runtime.status import StatusPublisher

# Publish rate in seconds — 1.0 = 1 Hz.
TELEMETRY_RATE = 1.0


@dataclass
class RobotContext:
    nav_pose: dict = field(default_factory=dict)
    nav_distance_remaining: float = 0.0
    nav_status: str = "idle"
    tree_distance_remaining: float = 0.0
    tree_status: str = "idle"
    segment_state: str = ""
    segment_status: str = "idle"
    mission_status: str = "idle"
    last_updated: str = ""


class WorldStateNode(Node):
    def __init__(self):
        super().__init__("world_state")
        self._lock = Lock()
        self._state = RobotContext()

        # Feedback subscriptions — actual types from dummy action servers
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._on_nav_feedback,
            10,
        )
        self.create_subscription(
            TreeIDWaypoint.Impl.FeedbackMessage,
            "/follow_tree_id_waypoint/_action/feedback",
            self._on_tree_feedback,
            10,
        )
        self.create_subscription(
            SegmentLeaves.Impl.FeedbackMessage,
            "/segment_leaves/_action/feedback",
            self._on_segment_feedback,
            10,
        )

        # Status subscriptions
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_nav_status,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/follow_tree_id_waypoint/_action/status",
            self._on_tree_status,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/segment_leaves/_action/status",
            self._on_segment_status,
            10,
        )

        self.create_subscription(String, "/mission_status", self._on_mission_status, 10)

        self.state_pub = self.create_publisher(String, "/world_state", 10)
        self.create_timer(TELEMETRY_RATE, self._publish_state)

        # Telemetry flows on /world_state; this is the one-shot latched snapshot
        # that makes readiness uniform across all four agents.
        StatusPublisher(self).publish(
            {"topic": "/world_state", "rate_hz": 1 / TELEMETRY_RATE}
        )

        self.get_logger().info(
            f"WorldStateNode started — publishing /world_state at {1 / TELEMETRY_RATE:.1f} Hz"
        )

    def get_state(self) -> dict:
        with self._lock:
            return asdict(self._state)

    def _publish_state(self):
        msg = String()
        msg.data = json.dumps(self.get_state())
        self.state_pub.publish(msg)

    def _on_nav_feedback(self, msg):
        with self._lock:
            p = msg.feedback.current_pose.pose.position
            self._state.nav_pose = {"x": p.x, "y": p.y, "z": p.z}
            self._state.nav_distance_remaining = msg.feedback.distance_remaining
            self._state.last_updated = str(msg.feedback.current_pose.header.stamp.sec)

    def _on_tree_feedback(self, msg):
        with self._lock:
            self._state.tree_distance_remaining = msg.feedback.dist

    def _on_segment_feedback(self, msg):
        with self._lock:
            self._state.segment_state = msg.feedback.current_state

    def _on_nav_status(self, msg):
        with self._lock:
            if msg.status_list:
                self._state.nav_status = self._status_str(msg.status_list[-1].status)

    def _on_tree_status(self, msg):
        with self._lock:
            if msg.status_list:
                self._state.tree_status = self._status_str(msg.status_list[-1].status)

    def _on_segment_status(self, msg):
        with self._lock:
            if msg.status_list:
                self._state.segment_status = self._status_str(
                    msg.status_list[-1].status
                )

    def _on_mission_status(self, msg):
        with self._lock:
            self._state.mission_status = msg.data

    @staticmethod
    def _status_str(code: int) -> str:
        return {
            1: "accepted",
            2: "executing",
            4: "succeeded",
            5: "canceled",
            6: "aborted",
        }.get(code, "unknown")


def main():
    spin.run(WorldStateNode)


if __name__ == "__main__":
    main()
