"""
mission_planner_node.py

ROS2 mission planner that:
- Subscribes to /mission/xml, /rosout, /bt/status_change
- On BT failure: fetches last 3 world-state frames + log context
- Calls Ollama/Gemma4 to edit the XML plan by 1-3 lines
- Republishes the edited XML to /mission/xml
- Keeps a compact memory of prior attempts (same limits as amiga_ros2_agents)

Also serves a lightweight A2A status endpoint on port 10001.
"""

import json
import os
import re
import textwrap
from threading import Lock, Thread
from typing import Dict, List, Optional

import requests
import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String

import uvicorn
from a2a.server.apps import A2AStarletteApplication
from a2a.server.request_handlers import DefaultRequestHandler
from a2a.server.tasks import InMemoryTaskStore

from .agent_card import AGENT_CARD
from .a2a_server import MissionPlannerHandler

# ---------------------------------------------------------------------------
# Context-window limits — kept identical to amiga_ros2_agents
# ---------------------------------------------------------------------------
LOG_WINDOW_SEC = 30.0        # rolling /rosout window kept in memory
FAILURE_CONTEXT_SEC = 3.0    # log slice sent to LLM (±N sec around failure)
RESULT_HISTORY_CHARS = 1000  # max chars of any single result in memory
MAX_RETRIES = 20             # max planning attempts (= MAX_STEPS in amiga_ros2_agents)
COMPRESS_AFTER = 3           # keep only last N prior attempts in the prompt
WORLD_STATE_FRAMES = 3       # SSE frames to collect from the world-state agent

OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://localhost:11434/api/chat")
OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "gemma4")
WORLD_STATE_URL = os.environ.get("WORLD_STATE_URL", "http://localhost:10004/")

ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
LEVEL_MAP = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

# ---------------------------------------------------------------------------
# System prompt
# ---------------------------------------------------------------------------
SYSTEM_PROMPT = """\
You are a mission planner for an autonomous agricultural robot called Amiga.
The robot executes BT.CPP XML behavior-tree mission plans.

A behavior-tree node has failed. Your job is to make a MINIMAL edit to the \
active XML plan so the robot can recover and continue.

## Hard rules
- Return ONLY the complete corrected XML. No explanation, no code fences, no markdown.
- Change at most 1-3 lines from the original XML.
- Never rewrite the whole plan — only the smallest change that addresses the failure.
- Preserve all XML structure, indentation, and attribute quoting exactly.
- Valid leaf node types: MoveToTreeID, SampleLeaf.
  - MoveToTreeID attrs: name, action_name="follow_tree_id_waypoint", id (int), approach_tree (true/false)
  - SampleLeaf attrs: name, action_name="segment_leaves"
- Valid control nodes: Sequence, Fallback, Retry (with num_cycles attr).
- If the same edit was already tried (see ## Prior attempts), try a DIFFERENT fix.
- If the robot has a navigation failure, prefer skipping the failed tree node or \
  reordering to visit a reachable tree first.
- If segmentation failed, try wrapping the SampleLeaf in a Retry node (num_cycles="2").
"""


def _box(title: str, width: int = 60) -> str:
    bar = "─" * (width - 2)
    return f"┌{bar}┐\n│ {title:<{width - 3}}│\n└{bar}┘"


def _summarize_edit(original: str, edited: str) -> str:
    """One-line description of what changed between two XML strings."""
    orig_lines = original.splitlines()
    edit_lines = edited.splitlines()
    changes = []
    for i, (a, b) in enumerate(zip(orig_lines, edit_lines)):
        if a.strip() != b.strip():
            changes.append(f"L{i + 1}: {b.strip()[:80]}")
    if len(edit_lines) != len(orig_lines):
        changes.append(f"line count {len(orig_lines)} → {len(edit_lines)}")
    return "; ".join(changes[:3]) if changes else "no visible change"


class MissionPlannerNode(Node):
    """ROS2 node that replans on BT failure with minimal XML edits."""

    def __init__(self):
        super().__init__("mission_planner")
        self._lock = Lock()

        # State
        self.current_mission_xml: Optional[str] = None
        self.log_buffer: List[Dict] = []
        self.memory: List[Dict] = []       # across-session history
        self._republishing = False         # echo-loop guard
        self._sessions = 0
        self._last_event: Optional[Dict] = None
        self._last_edit_summary: Optional[str] = None

        # Subscriptions
        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.create_subscription(String, "/bt/status_change", self._on_bt_failure, 10)

        # Publisher
        self.mission_pub = self.create_publisher(String, "/mission/xml", 10)

        self.get_logger().info(
            "MissionPlannerNode started — waiting for /mission/xml and /bt/status_change"
        )

    # ------------------------------------------------------------------
    # A2A status snapshot (called from MissionPlannerHandler)
    # ------------------------------------------------------------------

    def get_status(self) -> Dict:
        with self._lock:
            return {
                "mission_xml_received": self.current_mission_xml is not None,
                "sessions": self._sessions,
                "last_event": self._last_event,
                "last_edit_summary": self._last_edit_summary,
            }

    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------

    def _on_mission(self, msg: String):
        if not self._republishing:
            with self._lock:
                self.current_mission_xml = msg.data
            self.get_logger().info("Received mission XML")

    def _on_log(self, msg: Log):
        # Same filter as amiga_ros2_agents — skip the status publisher's own messages
        if "BTStatusPublisher" in msg.msg:
            return
        with self._lock:
            self.log_buffer.append({
                "stamp": self._stamp_to_sec(msg.stamp),
                "level": LEVEL_MAP.get(msg.level, str(msg.level)),
                "node": msg.name,
                "msg": ANSI_ESCAPE.sub("", msg.msg),
            })
            now = self._stamp_to_sec(self.get_clock().now().to_msg())
            self.log_buffer = [
                e for e in self.log_buffer if now - e["stamp"] < LOG_WINDOW_SEC
            ]

    def _on_bt_failure(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse /bt/status_change payload")
            return

        failure_sec = event.get("timestamp_ms", 0) / 1000.0
        with self._lock:
            log_context = [
                e for e in self.log_buffer
                if failure_sec - FAILURE_CONTEXT_SEC <= e["stamp"]
                <= failure_sec + FAILURE_CONTEXT_SEC
            ]

        # Planner runs in a daemon thread — Ollama calls can take 30-60 s
        # and we must not block the ROS2 spin loop
        Thread(
            target=self._run_planner, args=(event, log_context), daemon=True
        ).start()

    # ------------------------------------------------------------------
    # Planner — one session per BT failure
    # ------------------------------------------------------------------

    def _run_planner(self, event: Dict, log_context: List[Dict]):
        with self._lock:
            xml = self.current_mission_xml
            sessions_done = self._sessions

        if xml is None:
            self.get_logger().warn("No mission XML yet — skipping replan")
            return

        if sessions_done >= MAX_RETRIES:
            self.get_logger().warn(
                f"Reached MAX_RETRIES ({MAX_RETRIES}) — no more replanning"
            )
            return

        self.get_logger().info(_box(f"Mission Planner — session {sessions_done + 1}"))

        # 1. Fetch world state (last WORLD_STATE_FRAMES SSE frames)
        world_state = self._fetch_world_state()

        # 2. Compact memory for this specific failure node
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

        # 3. Build prompt — same field ordering as amiga_ros2_agents initial_user
        log_excerpt = json.dumps(log_context, indent=2)
        if len(log_excerpt) > RESULT_HISTORY_CHARS:
            log_excerpt = log_excerpt[:RESULT_HISTORY_CHARS] + "\n… [truncated]"

        prompt = (
            f"## Failure event\n{json.dumps(event, indent=2)}\n\n"
            f"## Active mission XML\n{xml}\n\n"
            f"## World state (last {len(world_state)} updates)\n"
            f"{json.dumps(world_state, indent=2)}\n\n"
            f"## Recent ROS logs (±{FAILURE_CONTEXT_SEC}s around failure)\n"
            f"{log_excerpt}\n\n"
            f"## Prior attempts for node '{failure_node}' (do not repeat)\n"
            f"{json.dumps(memory_summary, indent=2)}\n\n"
            "Return the corrected XML (1-3 line edit only)."
        )

        self.get_logger().info(
            f"  Calling {OLLAMA_MODEL} — "
            f"{len(world_state)} world-state frames, {len(log_context)} log entries"
        )

        # 4. Call LLM
        try:
            edited_xml = self._call_ollama(prompt)
        except Exception as exc:
            self.get_logger().error(f"  LLM call failed: {exc}")
            return

        # 5. Validate response looks like BT XML
        if "<BehaviorTree" not in edited_xml or "<root" not in edited_xml:
            self.get_logger().error(
                "  LLM did not return valid BT XML — not publishing\n"
                f"  Preview: {edited_xml[:200]}"
            )
            return

        # 6. Summarise the edit (first 3 changed lines)
        edit_summary = _summarize_edit(xml, edited_xml)
        self.get_logger().info(
            f"  Edit: {textwrap.shorten(edit_summary, 120, placeholder='…')}"
        )

        # 7. Publish — guarded by _republishing so _on_mission ignores it
        out = String()
        out.data = edited_xml
        self._republishing = True
        self.mission_pub.publish(out)
        self._republishing = False
        self.get_logger().info("  Published edited mission XML → /mission/xml")

        # 8. Update memory and status counters
        with self._lock:
            self.memory.append({
                "event": event,
                "edit_summary": edit_summary,
                "outcome": "published",
            })
            self._sessions += 1
            self._last_event = event
            self._last_edit_summary = edit_summary

        self.get_logger().info("─" * 60)

    # ------------------------------------------------------------------
    # World state client — same SSE pattern as test_llm_world_state.py
    # ------------------------------------------------------------------

    def _fetch_world_state(self) -> List[Dict]:
        frames: List[Dict] = []
        try:
            with requests.post(
                WORLD_STATE_URL,
                json={
                    "jsonrpc": "2.0",
                    "id": "mp-ws",
                    "method": "message/stream",
                    "params": {
                        "message": {
                            "role": "user",
                            "messageId": "mp-ws-1",
                            "parts": [{"type": "text", "text": "get context"}],
                        }
                    },
                },
                stream=True,
                timeout=10,
            ) as resp:
                resp.raise_for_status()
                for line in resp.iter_lines():
                    if not line:
                        continue
                    raw = line.decode("utf-8")
                    if not raw.startswith("data:"):
                        continue
                    try:
                        payload = json.loads(raw[5:].strip())
                        for part in payload["result"]["parts"]:
                            if "data" in part:
                                frames.append(part["data"])
                    except (KeyError, TypeError, json.JSONDecodeError):
                        pass
                    if len(frames) >= WORLD_STATE_FRAMES:
                        break
        except Exception as exc:
            self.get_logger().warn(f"World state fetch failed: {exc}")
        return frames

    # ------------------------------------------------------------------
    # Ollama client — same pattern as test_llm_world_state.py
    # ------------------------------------------------------------------

    def _call_ollama(self, prompt: str) -> str:
        response = requests.post(
            OLLAMA_URL,
            json={
                "model": OLLAMA_MODEL,
                "messages": [
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": prompt},
                ],
                "stream": False,
            },
            timeout=90,
        )
        response.raise_for_status()
        return response.json()["message"]["content"].strip()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = MissionPlannerNode()

    # Spin in a daemon thread so uvicorn can hold the main thread
    Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    handler = MissionPlannerHandler(node)
    task_store = InMemoryTaskStore()
    app = A2AStarletteApplication(
        agent_card=AGENT_CARD,
        http_handler=DefaultRequestHandler(
            agent_executor=handler,
            task_store=task_store,
        ),
    )
    uvicorn.run(app.build(), host="0.0.0.0", port=10001, log_level="info")