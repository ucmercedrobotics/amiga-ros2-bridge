"""
arbiter_node.py

Arbiter agent that gates candidate mission edits:
- Subscribes to /mission/candidate_xml (candidates from the mission planner)
- Subscribes to /mission/xml (tracks the currently active plan)
- Validates each candidate:
    1. Well-formed XML
    2. Validates against the real BT.CPP XSD
    3. Semantic check: no orphaned SampleLeaf
    4. Edit-size limit: candidate must not rewrite the whole plan
    5. Rate limit: minimum interval between accepted plans
- Publishes ACCEPTED candidates to /mission/xml (sole writer)
- Publishes REJECTED candidates' reasons to /mission/rejection so the
  mission planner can try again with that feedback

Also serves a lightweight A2A status endpoint on port 10003.
"""

import json
import os
import time
from threading import Lock, Thread
from typing import Dict, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from lxml import etree
from rclpy.node import Node
from std_msgs.msg import String

import uvicorn
from a2a.server.apps import A2AStarletteApplication
from a2a.server.request_handlers import DefaultRequestHandler
from a2a.server.tasks import InMemoryTaskStore

from .agent_card import AGENT_CARD
from .a2a_server import ArbiterHandler

# ---------------------------------------------------------------------------
# Arbiter policy limits
# ---------------------------------------------------------------------------
MAX_CHANGED_LINE_RATIO = 0.5   # reject if > 50% of lines differ from active plan
MIN_ACCEPT_INTERVAL_SEC = 5.0  # reject if last accepted plan was < N sec ago

AMIGA_XSD_PATH = os.environ.get("AMIGA_XSD_PATH", "")  # optional override


class ArbiterNode(Node):
    """Gates candidate mission edits before they reach the robot."""

    def __init__(self):
        super().__init__("arbiter")
        self._lock = Lock()

        self.active_mission_xml: Optional[str] = None
        self._last_accept_time = 0.0
        self._publishing = False  # ignore our own /mission/xml echo
        self._last_status: Dict = {
            "accepted": 0,
            "rejected": 0,
            "last_rejection_reason": None,
            "last_decision": None,
        }

        self.xsd_schema = self._load_xsd()

        self.create_subscription(String, "/mission/candidate_xml", self._on_candidate, 10)
        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.mission_pub = self.create_publisher(String, "/mission/xml", 10)
        self.rejection_pub = self.create_publisher(String, "/mission/rejection", 10)

        self.get_logger().info("ArbiterNode started — gating /mission/candidate_xml")

    # ------------------------------------------------------------------
    # Public API (called from A2A handler)
    # ------------------------------------------------------------------

    def get_status(self) -> Dict:
        with self._lock:
            return dict(self._last_status)

    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------

    def _on_mission(self, msg: String):
        """Track the active plan (initial mission or our own accepted edit)."""
        if not self._publishing:
            with self._lock:
                self.active_mission_xml = msg.data

    def _on_candidate(self, msg: String):
        candidate = msg.data
        verdict, reason = self._evaluate(candidate)

        if verdict:
            out = String()
            out.data = candidate
            self._publishing = True
            self.mission_pub.publish(out)
            self._publishing = False
            with self._lock:
                self.active_mission_xml = candidate
                self._last_accept_time = time.monotonic()
                self._last_status["accepted"] += 1
                self._last_status["last_decision"] = "accepted"
            self.get_logger().info("ACCEPTED candidate — published to /mission/xml")
        else:
            with self._lock:
                self._last_status["rejected"] += 1
                self._last_status["last_rejection_reason"] = reason
                self._last_status["last_decision"] = "rejected"
            self.get_logger().warn(f"REJECTED candidate — {reason}")

            # Tell the mission planner why, so it can try again with feedback
            rej = String()
            rej.data = json.dumps({
                "reason": reason,
                "timestamp_ms": int(time.time() * 1000),
            })
            self.rejection_pub.publish(rej)

    # ------------------------------------------------------------------
    # Checks
    # ------------------------------------------------------------------

    def _evaluate(self, candidate: str) -> Tuple[bool, str]:
        """Run all checks in order. Returns (accepted, reason_if_rejected)."""

        # 1. Well-formed XML
        try:
            doc = etree.fromstring(candidate.encode("utf-8"))
        except etree.XMLSyntaxError as exc:
            return False, f"not well-formed XML: {exc}"

        # 2. XSD validation
        if self.xsd_schema is not None and not self.xsd_schema.validate(doc):
            return False, f"XSD validation failed: {self.xsd_schema.error_log}"

        # 3. Semantic: orphaned SampleLeaf
        ok, reason = self._check_no_orphan_sample(doc)
        if not ok:
            return False, reason

        # 4. Edit-size limit (only when we know the active plan)
        with self._lock:
            active = self.active_mission_xml
        if active is not None:
            ratio = self._changed_line_ratio(active, candidate)
            if ratio > MAX_CHANGED_LINE_RATIO:
                return False, (
                    f"edit too large: {ratio:.0%} of lines changed "
                    f"(limit {MAX_CHANGED_LINE_RATIO:.0%})"
                )

        # 5. Rate limit
        with self._lock:
            since = time.monotonic() - self._last_accept_time
        if self._last_accept_time > 0 and since < MIN_ACCEPT_INTERVAL_SEC:
            return False, (
                f"rate limited: last plan accepted {since:.1f}s ago "
                f"(minimum interval {MIN_ACCEPT_INTERVAL_SEC}s)"
            )

        return True, ""

    @staticmethod
    def _check_no_orphan_sample(doc) -> Tuple[bool, str]:
        """Every SampleLeaf must be preceded (within its parent Sequence) by a
        MoveToTreeID with approach_tree="true" — otherwise the robot would
        sample wherever it happens to be standing."""
        for sample in doc.iter("SampleLeaf"):
            parent = sample.getparent()
            approached = False
            for sibling in parent:
                if sibling is sample:
                    break
                if (sibling.tag == "MoveToTreeID"
                        and sibling.get("approach_tree", "").lower() == "true"):
                    approached = True
            if not approached:
                name = sample.get("name", "<unnamed>")
                return False, (
                    f"orphaned SampleLeaf '{name}': no preceding MoveToTreeID "
                    f'with approach_tree="true" in its Sequence'
                )
        return True, ""

    @staticmethod
    def _changed_line_ratio(original: str, edited: str) -> float:
        """Fraction of the edited plan's lines that don't appear in the original."""
        orig_lines = {line.strip() for line in original.splitlines() if line.strip()}
        edit_lines = [line.strip() for line in edited.splitlines() if line.strip()]
        if not edit_lines:
            return 1.0
        changed = sum(1 for line in edit_lines if line not in orig_lines)
        return changed / len(edit_lines)

    # ------------------------------------------------------------------
    # XSD — same single-source-of-truth loading as the mission planner
    # ------------------------------------------------------------------

    def _resolve_xsd_path(self) -> str:
        if AMIGA_XSD_PATH:
            return AMIGA_XSD_PATH
        try:
            share_dir = get_package_share_directory("amiga_ros2_behavior_tree")
        except Exception as exc:
            self.get_logger().error(
                f"Could not resolve amiga_ros2_behavior_tree share dir: {exc}"
            )
            return ""
        return os.path.join(share_dir, "schemas", "amiga_btcpp.xsd")

    def _load_xsd(self):
        path = self._resolve_xsd_path()
        if not path:
            return None
        try:
            with open(path, "rb") as f:
                return etree.XMLSchema(etree.parse(f))
        except (OSError, etree.XMLSyntaxError, etree.XMLSchemaParseError) as exc:
            self.get_logger().error(f"Failed to load mission XSD from {path}: {exc}")
            return None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = ArbiterNode()

    Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    handler = ArbiterHandler(node)
    task_store = InMemoryTaskStore()
    app = A2AStarletteApplication(
        agent_card=AGENT_CARD,
        http_handler=DefaultRequestHandler(
            agent_executor=handler,
            task_store=task_store,
        ),
    )
    uvicorn.run(app.build(), host="0.0.0.0", port=10003, log_level="info")


if __name__ == "__main__":
    main()