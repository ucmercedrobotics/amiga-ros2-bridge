"""
mission_planner_node.py

Pure ROS2 node — no a2a-sdk / uvicorn imports here on purpose.
On BT failure: fetches last 3 world-state frames + log context, calls a
local LLM (Ollama/Gemma4) to edit the active XML plan by 1-3 lines, and
republishes it. Keeps a compact memory of prior attempts.

A2A status serving (port 10001) lives in mission_planner_a2a_main.py so
this module can be imported/run with plain ROS2 python — no a2a-sdk or
its newer protobuf pin required.
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

# ---------------------------------------------------------------------------
# Context-window limits — kept identical to amiga_ros2_agents
# ---------------------------------------------------------------------------
LOG_WINDOW_SEC = 30.0
FAILURE_CONTEXT_SEC = 3.0
RESULT_HISTORY_CHARS = 1000
MAX_RETRIES = 20
COMPRESS_AFTER = 3
WORLD_STATE_FRAMES = 3

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
- If the robot has low battery or a navigation failure, prefer skipping trees or \
  shortening the route over adding new steps.
"""


class MissionPlannerNode(Node):
    """ROS2 node that replans on BT failure with minimal XML edits."""

    def __init__(self):
        super().__init__("mission_planner")
        self._lock = Lock()

        self.current_mission_xml: Optional[str] = None
        self.log_buffer: List[Dict] = []
        self.memory: List[Dict] = []
        self._republishing = False
        self._last_status: Dict = {
            "mission_xml_received": False,
            "sessions": 0,
            "last_event": None,
            "last_edit_summary": None,
        }

        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.create_subscription(String, "/bt/status_change", self._on_bt_failure, 10)

        self.mission_pub = self.create_publisher(String, "/mission/xml", 10)

        self.get_logger().info("MissionPlannerNode started — waiting for /mission/xml")

    def get_status(self) -> Dict:
        with self._lock:
            return dict(self._last_status)

    def _on_mission(self, msg: String):
        if not self._republishing:
            with self._lock:
                self.current_mission_xml = msg.data
                self._last_status["mission_xml_received"] = True
            self.get_logger().info("Received mission XML")

    def _on_log(self, msg: Log):
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
                if failure_sec - FAILURE_CONTEXT_SEC <= e["stamp"] <= failure_sec + FAILURE_CONTEXT_SEC
            ]

        Thread(target=self._run_planner, args=(event, log_context), daemon=True).start()

    def _run_planner(self, event: Dict, log_context: List[Dict]):
        with self._lock:
            xml = self.current_mission_xml
            sessions_done = self._last_status["sessions"]

        if xml is None:
            self.get_logger().warn("No mission XML yet — skipping replan")
            return

        if sessions_done >= MAX_RETRIES:
            self.get_logger().warn(f"Reached MAX_RETRIES ({MAX_RETRIES}) — no more replanning")
            return

        self.get_logger().info(_box(f"Mission Planner — session {sessions_done + 1}"))

        world_state = self._fetch_world_state()

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
            f"  Calling Ollama ({OLLAMA_MODEL}) — "
            f"world_state={len(world_state)} frames, logs={len(log_context)} entries"
        )

        try:
            edited_xml = self._call_ollama(prompt)
        except Exception as exc:
            self.get_logger().error(f"  LLM call failed: {exc}")
            return

        if "<BehaviorTree" not in edited_xml or "<root" not in edited_xml:
            self.get_logger().error(
                "  LLM did not return valid BT XML — not publishing\n"
                f"  Response preview: {edited_xml[:200]}"
            )
            return

        edit_summary = _summarize_edit(xml, edited_xml)
        self.get_logger().info(
            f"  Edit: {textwrap.shorten(edit_summary, 120, placeholder='…')}"
        )

        out_msg = String()
        out_msg.data = edited_xml
        self._republishing = True
        self.mission_pub.publish(out_msg)
        self._republishing = False
        self.get_logger().info("  Published edited mission XML to /mission/xml")

        with self._lock:
            self.memory.append({
                "event": event,
                "edit_summary": edit_summary,
                "outcome": "published",
            })
            self._last_status["sessions"] += 1
            self._last_status["last_event"] = event
            self._last_status["last_edit_summary"] = edit_summary

        self.get_logger().info("─" * 60)

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

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


def _box(title: str, width: int = 60) -> str:
    bar = "─" * (width - 2)
    return f"┌{bar}┐\n│ {title:<{width - 3}}│\n└{bar}┘"


def _summarize_edit(original: str, edited: str) -> str:
    orig_lines = original.splitlines()
    edit_lines = edited.splitlines()
    changes = []
    for i, (a, b) in enumerate(zip(orig_lines, edit_lines)):
        if a.strip() != b.strip():
            changes.append(f"L{i + 1}: {b.strip()[:80]}")
    if len(edit_lines) != len(orig_lines):
        changes.append(f"line count {len(orig_lines)} → {len(edit_lines)}")
    return "; ".join(changes[:3]) if changes else "no visible change"


def main():
    rclpy.init()
    node = MissionPlannerNode()

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