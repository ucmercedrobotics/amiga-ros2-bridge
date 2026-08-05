"""
mission_planner_node.py

ROS2 mission planner that:
- Subscribes to /mission/xml, /rosout, /bt/status_change, /mission/rejection,
  /mission/abort and /world_state
- On BT failure: uses the last WORLD_STATE_FRAMES world-state frames + log context
- Calls an LLM (via llm.complete) to edit the XML plan by 1-4 lines
- Publishes the edit as a CANDIDATE to /mission/candidate_xml (the Arbiter gates it)
- On arbiter rejection: retries with the reason as feedback (bounded)
- On arbiter abort: halts replanning for the rest of the mission
- Keeps a compact memory of prior attempts

Edits are validated against the real BT.CPP XSD (amiga_ros2_behavior_tree's
installed schemas/amiga_btcpp.xsd) before being submitted.

Prompts live in prompts/mission_planner/.
"""

import json
import re
import textwrap
from collections import deque
from threading import Lock, Thread
from typing import Dict, List, Optional

from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String

from . import llm, prompts, spin, xsd
from .status import StatusPublisher

# ---------------------------------------------------------------------------
# Context-window limits
# ---------------------------------------------------------------------------
LOG_WINDOW_SEC = 30.0  # rolling /rosout window kept in memory
FAILURE_CONTEXT_SEC = 3.0  # log slice sent to LLM (±N sec around failure)
RESULT_HISTORY_CHARS = 1000  # max chars of any single result in memory
MAX_RETRIES = 20  # max planning attempts before giving up (= MAX_STEPS)
MAX_REJECTION_RETRIES = 3  # max arbiter-rejection retries per single BT failure
COMPRESS_AFTER = 3  # compress memory beyond the last N entries
WORLD_STATE_FRAMES = 3  # /world_state frames kept for prompt context

# A whole XML plan comes back in one reply, so this must NOT fall back to
# llm.MAX_TOKENS (2048) — a truncated plan fails XSD validation every time.
PLANNER_MAX_TOKENS = 8192

# ---------------------------------------------------------------------------
# Mission grammar — injected into both prompt templates
# ---------------------------------------------------------------------------
VALID_LEAVES = [
    {
        "tag": "MoveToTreeID",
        "attrs": 'name, action_name="follow_tree_id_waypoint", id (int), approach_tree (true/false)',
    },
    {
        "tag": "SampleLeaf",
        "attrs": 'name, action_name="segment_leaves"',
    },
]
VALID_CONTROLS = [
    "Sequence",
    "ReactiveSequence",
    "Fallback",
    "RetryUntilSuccessful (with required num_attempts attr)",
]
MAX_EDIT_LINES = 4

ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
LEVEL_MAP = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


class MissionPlannerNode(Node):
    """ROS2 node that replans on BT failure with minimal XML edits."""

    def __init__(self):
        super().__init__("mission_planner")
        self._lock = Lock()

        # State
        self.current_mission_xml: Optional[str] = None
        self.log_buffer: List[Dict] = []
        self.memory: List[Dict] = []  # across-session history
        self.world_state_frames: deque = deque(maxlen=WORLD_STATE_FRAMES)
        self._last_status: Dict = {
            "mission_xml_received": False,
            "sessions": 0,
            "last_event": None,
            "last_edit_summary": None,
        }

        # BT.CPP XSD — loaded once at startup from amiga_ros2_behavior_tree's
        # installed share directory (single source of truth with the BT executor)
        self.xsd_schema = xsd.load_schema(self.get_logger())
        self.xsd_text = xsd.read_text(self.get_logger())

        self.system_prompt = prompts.render(
            "mission_planner/system.j2",
            valid_leaves=VALID_LEAVES,
            valid_controls=VALID_CONTROLS,
            max_edit_lines=MAX_EDIT_LINES,
        )

        # Subscriptions
        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.create_subscription(String, "/bt/status_change", self._on_bt_failure, 10)
        self.create_subscription(String, "/mission/rejection", self._on_rejection, 10)
        self.create_subscription(String, "/mission/abort", self._on_abort, 10)
        self.create_subscription(String, "/world_state", self._on_world_state, 10)

        # Publisher — candidates only; the Arbiter owns /mission/xml
        self.mission_pub = self.create_publisher(String, "/mission/candidate_xml", 10)
        # Terminal planner outcomes that aren't a candidate (e.g. gave up after
        # exhausting arbiter-rejection retries) — makes the end state observable
        # instead of a silent stall. Does NOT halt the mission (unlike /mission/abort).
        self.status_pub = self.create_publisher(String, "/mission/planner_status", 10)

        # Arbiter feedback state
        self._rejection_retries = 0  # counts retries for the *current* BT failure
        self._last_failure_event = None  # the event we're currently trying to fix
        self._last_rejection_reason = None  # feedback to inject into the retry prompt
        self._mission_aborted = False  # set when arbiter declares non-viable

        self.status = StatusPublisher(self)
        self.status.publish(self.get_status())

        self.get_logger().info("MissionPlannerNode started — waiting for /mission/xml")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_status(self) -> Dict:
        with self._lock:
            return dict(self._last_status)

    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------

    def _on_mission(self, msg: String):
        """Track the active plan. The planner never publishes /mission/xml — the
        arbiter is its sole writer — so there is no echo to guard against here."""
        with self._lock:
            self.current_mission_xml = msg.data
            self._last_status["mission_xml_received"] = True
        self.status.publish(self.get_status())
        self.get_logger().info("Received mission XML")

    def _on_world_state(self, msg: String):
        """Keep a rolling window of the most recent world-state frames, so a
        replan reads whatever has already arrived instead of blocking on a fetch."""
        try:
            frame = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Could not parse /world_state payload")
            return
        with self._lock:
            self.world_state_frames.append(frame)

    def _on_log(self, msg: Log):
        if "BTStatusPublisher" in msg.msg:
            return
        with self._lock:
            self.log_buffer.append(
                {
                    "stamp": self._stamp_to_sec(msg.stamp),
                    "level": LEVEL_MAP.get(msg.level, str(msg.level)),
                    "node": msg.name,
                    "msg": ANSI_ESCAPE.sub("", msg.msg),
                }
            )
            now = self._stamp_to_sec(self.get_clock().now().to_msg())
            self.log_buffer = [
                e for e in self.log_buffer if now - e["stamp"] < LOG_WINDOW_SEC
            ]

    def _on_bt_failure(self, msg: String):
        if self._mission_aborted:
            self.get_logger().warn("Mission aborted — ignoring further BT failures")
            return
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse /bt/status_change payload")
            return

        if not isinstance(event, dict):
            self.get_logger().error(
                f"/bt/status_change payload must be a JSON object, got {type(event).__name__}"
            )
            return

        failure_sec = event.get("timestamp_ms", 0) / 1000.0
        with self._lock:
            log_context = [
                e
                for e in self.log_buffer
                if failure_sec - FAILURE_CONTEXT_SEC
                <= e["stamp"]
                <= failure_sec + FAILURE_CONTEXT_SEC
            ]

        # Run planner in a background thread so the spin loop stays responsive
        Thread(target=self._run_planner, args=(event, log_context), daemon=True).start()

    def _on_rejection(self, msg: String):
        """Arbiter rejected our last candidate — retry with the reason as feedback,
        bounded by MAX_REJECTION_RETRIES per BT failure."""
        try:
            rejection = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse /mission/rejection payload")
            return

        reason = rejection.get("reason", "unknown")

        with self._lock:
            # Correct the memory record — this edit was never actually published
            if self.memory:
                self.memory[-1]["outcome"] = f"rejected by arbiter: {reason}"

            event = self._last_failure_event
            if event is None or self._mission_aborted:
                return  # nothing to retry

            if self._rejection_retries >= MAX_REJECTION_RETRIES:
                self.get_logger().error(
                    f"Arbiter rejected {MAX_REJECTION_RETRIES} candidates for "
                    f"node '{event.get('node')}' — giving up on this failure"
                )
                self._last_status["last_edit_summary"] = (
                    f"gave up after {MAX_REJECTION_RETRIES} rejections: {reason}"
                )
                gave_up_node = event.get("node")
                status = String()
                status.data = json.dumps(
                    {
                        "outcome": "gave_up",
                        "cause": "rejection_retries_exhausted",
                        "node": gave_up_node,
                        "last_reason": reason,
                    }
                )
                self.status_pub.publish(status)
                gave_up = True
            else:
                self._rejection_retries += 1
                self._last_rejection_reason = reason
                attempt = self._rejection_retries
                gave_up = False

        if gave_up:
            self.status.publish(self.get_status())
            return

        self.get_logger().warn(
            f"Candidate rejected ({reason}) — retry {attempt}/{MAX_REJECTION_RETRIES}"
        )
        Thread(target=self._run_planner, args=(event, []), daemon=True).start()

    def _on_abort(self, msg: String):
        """Arbiter declared the mission non-viable — stop replanning entirely.
        Clearing _last_failure_event makes any in-flight rejection a no-op."""
        try:
            info = json.loads(msg.data)
        except json.JSONDecodeError:
            info = {}
        reason = info.get("reason", "unknown")
        with self._lock:
            self._last_failure_event = None
            self._rejection_retries = 0
            self._mission_aborted = True
            self._last_status["last_edit_summary"] = f"MISSION ABORTED: {reason}"
        self.status.publish(self.get_status())
        self.get_logger().error(
            f"Mission aborted by arbiter — {reason}. Halting replanning."
        )

    # ------------------------------------------------------------------
    # Planner
    # ------------------------------------------------------------------

    def _run_planner(self, event: Dict, log_context: List[Dict]):
        """One planning session: build context → call LLM → publish candidate."""
        with self._lock:
            xml = self.current_mission_xml
            sessions_done = self._last_status["sessions"]
            aborted = self._mission_aborted

        if aborted:
            return

        if xml is None:
            self.get_logger().warn("No mission XML yet — skipping replan")
            return

        if sessions_done >= MAX_RETRIES:
            self.get_logger().warn(
                f"Reached MAX_RETRIES ({MAX_RETRIES}) — no more replanning"
            )
            return

        # Track which failure this session addresses. A genuinely new failure
        # (different node) resets the arbiter-rejection counter.
        with self._lock:
            prev = self._last_failure_event
            if prev is None or prev.get("node") != event.get("node"):
                self._rejection_retries = 0
                self._last_rejection_reason = None
            self._last_failure_event = event

        self.get_logger().info(_box(f"Mission Planner — session {sessions_done + 1}"))

        # 1. World state — whatever the /world_state subscription has collected
        with self._lock:
            world_state = list(self.world_state_frames)

        # 2. Compact memory relevant to this failure node
        failure_node = event.get("node", "unknown")
        relevant = [m for m in self.memory if m["event"].get("node") == failure_node]
        memory_summary = [
            {
                "attempt": i + 1,
                "edit_summary": m.get("edit_summary", "—"),
                "outcome": m.get("outcome", "—"),
            }
            for i, m in enumerate(relevant[-COMPRESS_AFTER:])
        ]

        # 3. Build prompt
        log_excerpt = json.dumps(log_context, indent=2)
        if len(log_excerpt) > RESULT_HISTORY_CHARS:
            log_excerpt = log_excerpt[:RESULT_HISTORY_CHARS] + "\n… [truncated]"

        with self._lock:
            rejection_note = self._last_rejection_reason

        prompt = prompts.render(
            "mission_planner/replan_user.j2",
            rejection_reason=rejection_note,
            event=json.dumps(event, indent=2),
            mission_xml=xml,
            xsd_text=self.xsd_text,
            world_state=json.dumps(world_state, indent=2),
            world_state_count=len(world_state),
            log_excerpt=log_excerpt,
            failure_node=failure_node,
            memory_summary=json.dumps(memory_summary, indent=2),
            failure_context_sec=FAILURE_CONTEXT_SEC,
            valid_leaves=VALID_LEAVES,
            max_edit_lines=MAX_EDIT_LINES,
        )

        self.get_logger().info(
            f"  Calling model ({llm.MODEL}) — "
            f"world_state={len(world_state)} frames, logs={len(log_context)} entries"
        )

        # 4. Call LLM
        try:
            edited_xml = llm.complete(
                self.system_prompt, prompt, max_tokens=PLANNER_MAX_TOKENS
            )
        except Exception as exc:
            self.get_logger().error(f"  LLM call failed: {exc}")
            return

        edited_xml = llm.strip_code_fence(edited_xml)

        # 5. Validate against the real BT.CPP XSD
        is_valid, xsd_error = xsd.validate(self.xsd_schema, edited_xml)
        if not is_valid:
            self.get_logger().error(
                "  LLM edit failed XSD validation — not publishing\n"
                f"  {xsd_error}\n"
                f"  Response preview: {edited_xml[:200]}"
            )
            return

        # 6. Summarise what changed
        edit_summary = _summarize_edit(xml, edited_xml)
        self.get_logger().info(
            f"  Edit: {textwrap.shorten(edit_summary, 120, placeholder='…')}"
        )

        # 7. Publish candidate to the Arbiter
        out_msg = String()
        out_msg.data = edited_xml
        self.mission_pub.publish(out_msg)
        self.get_logger().info("  Published candidate XML to /mission/candidate_xml")

        # 8. Store in memory and update status
        with self._lock:
            self.memory.append(
                {
                    "event": event,
                    "edit_summary": edit_summary,
                    "outcome": "submitted",
                }
            )
            self._last_status["sessions"] += 1
            self._last_status["last_event"] = event
            self._last_status["last_edit_summary"] = edit_summary
        self.status.publish(self.get_status())

        self.get_logger().info("─" * 60)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------


def _box(title: str, width: int = 60) -> str:
    bar = "─" * (width - 2)
    return f"┌{bar}┐\n│ {title:<{width - 3}}│\n└{bar}┘"


def _summarize_edit(original: str, edited: str) -> str:
    """Return a compact description of what changed between the two XML strings."""
    orig_lines = original.splitlines()
    edit_lines = edited.splitlines()
    changes = []
    for i, (a, b) in enumerate(zip(orig_lines, edit_lines)):
        if a.strip() != b.strip():
            changes.append(f"L{i + 1}: {b.strip()[:80]}")
    if len(edit_lines) != len(orig_lines):
        changes.append(f"line count {len(orig_lines)} → {len(edit_lines)}")
    return "; ".join(changes[:3]) if changes else "no visible change"


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    spin.run(MissionPlannerNode)


if __name__ == "__main__":
    main()
