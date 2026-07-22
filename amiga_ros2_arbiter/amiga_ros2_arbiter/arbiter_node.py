"""
arbiter_node.py

Arbiter agent that gates candidate mission edits before they reach the robot.

For each candidate (from /mission/candidate_xml) it checks, in order:
    1. Well-formed XML
    2. Validates against the real BT.CPP XSD
    3. No orphaned SampleLeaf (semantic)
    4. Objective preservation / mission viability (semantic):
         - each NEW dropped tree must be justified (permanent failure)
         - total dropped trees must stay within a model-determined viability
           budget; exceeding it ABORTS the mission instead of retrying
    5. Edit-size limit (candidate must not rewrite the whole plan)
    6. Rate limit (min interval between ACCEPTED plans; skipped mid-retry)

Outcomes:
    - ACCEPTED  -> published to /mission/xml (sole writer)
    - REJECTED  -> reason published to /mission/rejection (planner retries)
    - ABORTED   -> reason published to /mission/abort (planner halts replanning)

The viability budget is obtained once, up front, via a single LLM call when the
pristine mission is first seen. Also serves an A2A status endpoint on port 10003.
"""

import json
import os
import time
from threading import Lock, Thread
from typing import Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from lxml import etree
from rclpy.node import Node
from std_msgs.msg import String

from litellm import completion

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
MAX_DROPPED_TREES = 0          # trees an edit may drop with no justification
PERMANENT_DROP_ALLOWANCE = 1   # extra NEW drops allowed when the failure is permanent
PERMANENT_KEYWORDS = ("permanent", "removed", "unavailable", "does not exist")

DEFAULT_VIABILITY_BUDGET = 2   # fallback total-drop budget if the model call fails
OPENAI_MODEL = os.environ.get("OPENAI_MODEL", "gpt-4o")

VIABILITY_SYSTEM = (
    "You assess agricultural mission viability. Given a mission and its target "
    "trees, decide how many of those trees could be skipped before the mission "
    "is no longer worth completing. Reply with ONLY a single integer."
)

AMIGA_XSD_PATH = os.environ.get("AMIGA_XSD_PATH", "")  # optional override


class ArbiterNode(Node):
    """Gates candidate mission edits before they reach the robot."""

    def __init__(self):
        super().__init__("arbiter")
        self._lock = Lock()

        self.active_mission_xml: Optional[str] = None
        self.original_objectives: Optional[set] = None   # tree set of the pristine mission
        self.justified_drops: set = set()                # trees already accepted as dropped
        self.max_droppable: Optional[int] = None         # model-determined viability budget
        self._last_failure_reason: str = ""
        self._last_accept_time = 0.0
        self._awaiting_retry = False                     # True between a reject and next accept
        self._publishing = False                         # ignore our own /mission/xml echo
        self._last_status: Dict = {
            "accepted": 0,
            "rejected": 0,
            "last_rejection_reason": None,
            "last_decision": None,
        }

        self.xsd_schema = self._load_xsd()

        self.create_subscription(String, "/mission/candidate_xml", self._on_candidate, 10)
        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.create_subscription(String, "/bt/status_change", self._on_bt_failure, 10)
        self.mission_pub = self.create_publisher(String, "/mission/xml", 10)
        self.rejection_pub = self.create_publisher(String, "/mission/rejection", 10)
        self.abort_pub = self.create_publisher(String, "/mission/abort", 10)

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
        """Track the active plan, and — the first time we see a plan — capture the
        ORIGINAL mission's objective tree set and request a viability budget."""
        if self._publishing:
            return
        with self._lock:
            self.active_mission_xml = msg.data
            if self.original_objectives is None:
                try:
                    doc = etree.fromstring(msg.data.encode("utf-8"))
                except etree.XMLSyntaxError:
                    return
                self.original_objectives = self._objective_tree_ids(doc)
                self.justified_drops = set()
                mission_text = doc.findtext("Mission", default="")
                trees = sorted(self.original_objectives)
                self.get_logger().info(
                    f"Captured original mission objectives: trees {trees}"
                )
                Thread(
                    target=self._compute_viability_budget,
                    args=(mission_text, trees),
                    daemon=True,
                ).start()

    def _on_candidate(self, msg: String):
        candidate = msg.data
        verdict, reason, abort = self._evaluate(candidate)

        if verdict:
            out = String()
            out.data = candidate
            self._publishing = True
            self.mission_pub.publish(out)
            self._publishing = False
            with self._lock:
                self.active_mission_xml = candidate
                self._last_accept_time = time.monotonic()
                self._awaiting_retry = False
                if self.original_objectives:
                    try:
                        cand_doc = etree.fromstring(candidate.encode("utf-8"))
                        cand_objs = self._objective_tree_ids(cand_doc)
                        self.justified_drops |= (self.original_objectives - cand_objs)
                    except etree.XMLSyntaxError:
                        pass
                self._last_status["accepted"] += 1
                self._last_status["last_decision"] = "accepted"
            self.get_logger().info("ACCEPTED candidate — published to /mission/xml")

        elif abort:
            with self._lock:
                self._awaiting_retry = False
                self._last_status["last_decision"] = "aborted"
                self._last_status["last_rejection_reason"] = reason
            self.get_logger().error(f"ABORTING MISSION — {reason}")
            ab = String()
            ab.data = json.dumps({"reason": reason, "timestamp_ms": int(time.time() * 1000)})
            self.abort_pub.publish(ab)

        else:
            with self._lock:
                self._awaiting_retry = True
                self._last_status["rejected"] += 1
                self._last_status["last_rejection_reason"] = reason
                self._last_status["last_decision"] = "rejected"
            self.get_logger().warn(f"REJECTED candidate — {reason}")
            rej = String()
            rej.data = json.dumps({"reason": reason, "timestamp_ms": int(time.time() * 1000)})
            self.rejection_pub.publish(rej)

    def _on_bt_failure(self, msg: String):
        """The arbiter listens to failures only to know WHY the planner is
        editing — used to decide whether a drop is permanently justified."""
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(event, dict):
            with self._lock:
                self._last_failure_reason = str(event.get("reason", ""))

    # ------------------------------------------------------------------
    # Viability budget — one LLM call when the pristine mission is captured
    # ------------------------------------------------------------------

    def _compute_viability_budget(self, mission_text: str, trees: List):
        prompt = (
            f"Mission: {mission_text}\n"
            f"Target trees: {trees} ({len(trees)} total)\n"
            f"How many of these trees can be skipped before the mission is no "
            f"longer worth completing? Answer with a single integer 0-{len(trees)}."
        )
        try:
            cmp = completion(
                model=OPENAI_MODEL,
                messages=[
                    {"role": "system", "content": VIABILITY_SYSTEM},
                    {"role": "user", "content": prompt},
                ],
                temperature=0.0,
                max_tokens=10,
            )
            text = cmp.choices[0].message.content.strip()
            n = int("".join(c for c in text if c.isdigit()))
        except Exception as exc:
            n = DEFAULT_VIABILITY_BUDGET
            self.get_logger().warn(f"Viability call failed ({exc}); default budget {n}")
        with self._lock:
            self.max_droppable = n
        self.get_logger().info(f"Model viability budget: up to {n} tree(s) may be skipped")

    # ------------------------------------------------------------------
    # Checks
    # ------------------------------------------------------------------

    def _evaluate(self, candidate: str) -> Tuple[bool, str, bool]:
        """Returns (accepted, reason_if_not, abort). abort=True means the mission
        is no longer viable and replanning should stop, not retry."""

        # 1. Well-formed XML
        try:
            doc = etree.fromstring(candidate.encode("utf-8"))
        except etree.XMLSyntaxError as exc:
            return False, f"not well-formed XML: {exc}", False

        # 2. XSD validation
        if self.xsd_schema is not None and not self.xsd_schema.validate(doc):
            return False, f"XSD validation failed: {self.xsd_schema.error_log}", False

        # 3. Semantic: orphaned SampleLeaf
        ok, reason = self._check_no_orphan_sample(doc)
        if not ok:
            return False, reason, False

        # 4. Semantic: objective preservation / viability
        ok, reason, abort = self._check_objective_preserved(doc)
        if not ok:
            return False, reason, abort

        # 5. Edit-size limit (only when we know the active plan)
        with self._lock:
            active = self.active_mission_xml
        if active is not None:
            ratio = self._changed_line_ratio(active, candidate)
            if ratio > MAX_CHANGED_LINE_RATIO:
                return False, (
                    f"edit too large: {ratio:.0%} of lines changed "
                    f"(limit {MAX_CHANGED_LINE_RATIO:.0%})"
                ), False

        # 6. Rate limit — skipped while in a reject→retry cycle
        with self._lock:
            since = time.monotonic() - self._last_accept_time
            awaiting_retry = self._awaiting_retry
        if (not awaiting_retry
                and self._last_accept_time > 0
                and since < MIN_ACCEPT_INTERVAL_SEC):
            return False, (
                f"rate limited: last plan accepted {since:.1f}s ago "
                f"(minimum interval {MIN_ACCEPT_INTERVAL_SEC}s)"
            ), False

        return True, "", False

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

    @staticmethod
    def _objective_tree_ids(doc) -> set:
        """The mission's objectives = the set of tree IDs the robot is meant to
        approach (approach_tree="true"). Transit moves (approach_tree="false")
        are not objectives and are ignored."""
        ids = set()
        for mv in doc.iter("MoveToTreeID"):
            if mv.get("approach_tree", "").lower() == "true":
                tid = mv.get("id")
                if tid is not None:
                    ids.add(tid)
        return ids

    def _check_objective_preserved(self, doc) -> Tuple[bool, str, bool]:
        """Two gates:
          - Gate 1 (fixable → reject): each NEWLY dropped tree must be justified
            by a permanent failure. Only new drops count; already-justified drops
            are remembered so removals accumulate across a mission.
          - Gate 2 (unfixable → abort): total dropped trees must stay within the
            model-determined viability budget."""
        with self._lock:
            original = self.original_objectives
            reason = self._last_failure_reason.lower()
            already_justified = set(self.justified_drops)
            budget = self.max_droppable
        if not original:
            return True, "", False
        if budget is None:
            budget = DEFAULT_VIABILITY_BUDGET  # call not back yet

        candidate_objs = self._objective_tree_ids(doc)
        dropped = original - candidate_objs
        newly_dropped = dropped - already_justified

        # Gate 1: each NEW drop must be justified — fixable → normal rejection
        allowed_new = MAX_DROPPED_TREES
        if any(kw in reason for kw in PERMANENT_KEYWORDS):
            allowed_new = max(allowed_new, PERMANENT_DROP_ALLOWANCE)
        if len(newly_dropped) > allowed_new:
            return False, (
                f"unjustified drop: {sorted(newly_dropped)} not permitted "
                f"(already justified: {sorted(already_justified)}; "
                f"allows {allowed_new} new this failure)"
            ), False

        # Gate 2: total drops exceed viability budget — unfixable → ABORT
        if len(dropped) > budget:
            return False, (
                f"mission no longer viable: {len(dropped)} trees dropped "
                f"{sorted(dropped)} exceeds viability budget of {budget}"
            ), True

        return True, "", False

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