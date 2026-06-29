from dataclasses import dataclass, field, asdict
from threading import Lock, Thread

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from amiga_navigation_interfaces.action import TreeIDWaypoint
from kortex_interfaces.action import SegmentLeaves
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String

import uvicorn
from a2a.server.apps import A2AStarletteApplication
from a2a.server.request_handlers import DefaultRequestHandler
from a2a.server.tasks import InMemoryTaskStore

from .agent_card import AGENT_CARD
from .a2a_server import WorldStateHandler


@dataclass
class RobotContext:
    # NavigateToPose feedback
    nav_pose: dict = field(default_factory=dict)
    nav_distance_remaining: float = 0.0
    nav_status: str = "idle"
    # TreeIDWaypoint feedback + status
    tree_distance_remaining: float = 0.0
    tree_status: str = "idle"
    # SegmentLeaves feedback + status
    segment_state: str = ""
    segment_status: str = "idle"
    # bt_runner mission string
    mission_status: str = "idle"
    last_updated: str = ""


class WorldStateNode(Node):
    def __init__(self):
        super().__init__("world_state")
        self._lock = Lock()
        self._state = RobotContext()

        # --- Feedback subscriptions ---
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._on_nav_feedback, 10,
        )
        self.create_subscription(
            TreeIDWaypoint.Impl.FeedbackMessage,
            "/follow_tree_id_waypoint/_action/feedback",
            self._on_tree_feedback, 10,
        )
        self.create_subscription(
            SegmentLeaves.Impl.FeedbackMessage,
            "/segment_leaves/_action/feedback",
            self._on_segment_feedback, 10,
        )

        # --- Status subscriptions ---
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_nav_status, 10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/follow_tree_id_waypoint/_action/status",
            self._on_tree_status, 10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/segment_leaves/_action/status",
            self._on_segment_status, 10,
        )

        # --- Mission string from bt_runner ---
        self.create_subscription(
            String, "/mission_status", self._on_mission_status, 10
        )

        self.get_logger().info(
            "WorldStateNode started — listening on action feedback/status topics"
        )

    def get_state(self) -> dict:
        with self._lock:
            return asdict(self._state)

    # ------------------------------------------------------------------
    # Feedback callbacks — extract rich payload from dummy servers
    # ------------------------------------------------------------------

    def _on_nav_feedback(self, msg):
        with self._lock:
            p = msg.feedback.current_pose.pose.position
            self._state.nav_pose = {"x": p.x, "y": p.y, "z": p.z}
            self._state.nav_distance_remaining = msg.feedback.distance_remaining
            self._state.last_updated = str(
                msg.feedback.current_pose.header.stamp.sec
            )

    def _on_tree_feedback(self, msg):
        # dummy_tree_id_server publishes feedback->dist (float, metres remaining)
        with self._lock:
            self._state.tree_distance_remaining = msg.feedback.dist

    def _on_segment_feedback(self, msg):
        # dummy_segment_leaves_server publishes feedback->current_state (string)
        with self._lock:
            self._state.segment_state = msg.feedback.current_state

    # ------------------------------------------------------------------
    # Status callbacks — goal lifecycle (accepted / executing / succeeded …)
    # ------------------------------------------------------------------

    def _on_nav_status(self, msg):
        with self._lock:
            if msg.status_list:
                self._state.nav_status = self._status_str(
                    msg.status_list[-1].status
                )

    def _on_tree_status(self, msg):
        with self._lock:
            if msg.status_list:
                self._state.tree_status = self._status_str(
                    msg.status_list[-1].status
                )

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
    rclpy.init()
    node = WorldStateNode()

    Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    handler = WorldStateHandler(node)
    task_store = InMemoryTaskStore()
    app = A2AStarletteApplication(
        agent_card=AGENT_CARD,
        http_handler=DefaultRequestHandler(
            agent_executor=handler,
            task_store=task_store,
        ),
    )
    uvicorn.run(app.build(), host="0.0.0.0", port=10004, log_level="info")