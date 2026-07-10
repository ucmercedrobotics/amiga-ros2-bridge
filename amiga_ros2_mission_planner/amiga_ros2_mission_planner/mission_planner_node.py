"""
mission_planner_node.py

ROS2 mission planner that:
- Subscribes to /mission/xml, /rosout, /bt/status_change
- On BT failure: fetches last 3 world-state frames + log context
- Calls OpenAI's gpt-4o to edit the XML plan by 1-3 lines
- Republishes the edited XML to /mission/xml
- Keeps a compact memory of prior attempts (same limits as amiga_ros2_agents)

Edits are validated against the real BT.CPP XSD (amiga_ros2_behavior_tree's
installed schemas/amiga_btcpp.xsd) before being published — an invalid
edit is dropped rather than sent to the robot.

Also serves a lightweight A2A status endpoint on port 10001.
"""

import json
import os
import time
import re
import textwrap
from threading import Lock, Thread
from typing import Dict, List, Optional, Tuple

import requests
import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from lxml import etree
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String

import litellm
from litellm import completion

import uvicorn
from a2a.server.apps import A2AStarletteApplication
from a2a.server.request_handlers import DefaultRequestHandler
from a2a.server.tasks import InMemoryTaskStore

from .agent_card import AGENT_CARD
from .a2a_server import MissionPlannerHandler

litellm.drop_params = True

# ---------------------------------------------------------------------------
# Context-window limits — kept identical to amiga_ros2_agents
# ---------------------------------------------------------------------------
LOG_WINDOW_SEC = 30.0        # rolling /rosout window kept in memory
FAILURE_CONTEXT_SEC = 3.0    # log slice sent to LLM (±N sec around failure)
RESULT_HISTORY_CHARS = 1000  # max chars of any single result in memory
MAX_RETRIES = 20             # max planning attempts before giving up (= MAX_STEPS)
COMPRESS_AFTER = 3           # compress memory beyond the last N entries
WORLD_STATE_FRAMES = 3       # SSE frames to collect from the world-state agent

CLOUD_MODEL = os.environ.get("CLOUD_MODEL", "gpt-4o")
CLOUD_MODEL_TEMPERATURE = float(os.environ.get("CLOUD_MODEL_TEMPERATURE", "0.2"))
CLOUD_MODEL_MAX_TOKENS = int(os.environ.get("CLOUD_MODEL_MAX_TOKENS", "4096"))
ENV_FILE_PATH = os.environ.get("ENV_FILE_PATH", "/amiga-ros2-bridge/.env")
WORLD_STATE_URL = os.environ.get("WORLD_STATE_URL", "http://localhost:10004/")
AMIGA_XSD_PATH = os.environ.get("AMIGA_XSD_PATH", "")  # optional override

load_dotenv(ENV_FILE_PATH)

ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
LEVEL_MAP = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

# ---------------------------------------------------------------------------
# System prompt — tight constraints on what the LLM is allowed to do
# ---------------------------------------------------------------------------
SYSTEM_PROMPT = """\
You are a mission planner for an autonomous agricultural robot called Amiga.
The robot executes BT.CPP XML behavior-tree mission plans.

A behavior-tree node has failed. Your job is to make a MINIMAL edit to the \
active XML plan so the robot can recover and continue.

## Hard rules
- Return ONLY the complete corrected XML. No explanation, no code fences, no markdown.
- The file created MUST NOT start with ``` xml and MUST NOT end with ```.
- Change at most 1-3 lines from the original XML.
- Never rewrite the whole plan — only the smallest change that addresses the failure.
- Preserve all XML structure, indentation, and attribute quoting exactly.
- Valid leaf node types: MoveToTreeID, SampleLeaf — and ONLY these two. Never use <Action>, <Task>, or any other tag name for a leaf.
  - MoveToTreeID attrs: name, action_name="follow_tree_id_waypoint", id (int), approach_tree (true/false)
  - SampleLeaf attrs: name, action_name="segment_leaves"
- Valid control nodes: Sequence, Fallback, Retry (with num_cycles attr).
- If the same edit was already tried (see ## Prior attempts), try a DIFFERENT fix.
- If the robot has low battery or a navigation failure, prefer skipping trees or \
  shortening the route over adding new steps.
- Your edit MUST validate against the XSD schema provided below. Pay close \
  attention to required attributes and fixed action_name values.
"""


class MissionPlannerNode(Node):
    """ROS2 node that replans on BT failure with minimal XML edits."""

    def __init__(self):
        super().__init__("mission_planner")
        self._lock = Lock()

        # State
        self.current_mission_xml: Optional[str] = None
        self.log_buffer: List[Dict] = []
        self.memory: List[Dict] = []           # across-session history
        self._republishing = False             # echo-loop guard (same as amiga_ros2_agents)
        self._last_status: Dict = {            # exposed via A2A
            "mission_xml_received": False,
            "sessions": 0,
            "last_event": None,
            "last_edit_summary": None,
        }

        # BT.CPP XSD — loaded once at startup from amiga_ros2_behavior_tree's
        # installed share directory (single source of truth with the BT executor)
        self.xsd_schema = self._load_xsd()
        self.xsd_text = self._read_xsd_text()

        # Subscriptions
        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.create_subscription(String, "/bt/status_change", self._on_bt_failure, 10)

        # Publisher
        self.mission_pub = self.create_publisher(String, "/mission/xml", 10)

        self.get_logger().info("MissionPlannerNode started — waiting for /mission/xml")

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

        # Run planner in a background thread so the spin loop stays responsive
        # (Ollama calls can take 30-60 s)
        Thread(target=self._run_planner, args=(event, log_context), daemon=True).start()

    # ------------------------------------------------------------------
    # Planner
    # ------------------------------------------------------------------

    def _run_planner(self, event: Dict, log_context: List[Dict]):
        """One planning session: fetch context → call LLM → publish edit."""
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

        # 1. Fetch world state
        world_state = self._fetch_world_state()

        # 2. Compact memory relevant to this failure node (last COMPRESS_AFTER entries)
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

        # 3. Build prompt (same field order as amiga_ros2_agents initial_user)
        log_excerpt = json.dumps(log_context, indent=2)
        if len(log_excerpt) > RESULT_HISTORY_CHARS:
            log_excerpt = log_excerpt[:RESULT_HISTORY_CHARS] + "\n… [truncated]"

        prompt = (
            f"## Failure event\n{json.dumps(event, indent=2)}\n\n"
            f"## Active mission XML\n{xml}\n\n"
            f"## Mission XML Schema (XSD) — the edit MUST validate against this\n"
            f"{self.xsd_text}\n\n"
            f"## World state (last {len(world_state)} updates)\n"
            f"{json.dumps(world_state, indent=2)}\n\n"
            f"## Recent ROS logs (±{FAILURE_CONTEXT_SEC}s around failure)\n"
            f"{log_excerpt}\n\n"
            f"## Prior attempts for node '{failure_node}' (do not repeat)\n"
            f"{json.dumps(memory_summary, indent=2)}\n\n"
            "Return ONLY the corrected XML. Do not use code fences or markdown. "
            "The edited XML MUST NOT start with ``` and MUST NOT end with ```"
            "The ONLY valid leaf elements are <MoveToTreeID> and <SampleLeaf> — never <Action>, <Task>, or any other tag name."
            "Change at most 1-3 lines from the original XML shown above."
        )
        self.get_logger().info(
            f"  Calling OpenAI ({CLOUD_MODEL}) — "
            f"world_state={len(world_state)} frames, logs={len(log_context)} entries"
        )

        # 4. Call LLM
        try:
            edited_xml = self._call_openai(prompt)
        except Exception as exc:
             self.get_logger().error(f"  LLM call failed: {exc}")
             return  

        edited_xml = _strip_code_fence(edited_xml)

        # 5. Validate the response against the real BT.CPP XSD
        is_valid, xsd_error = self._validate_xml(edited_xml)
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

        # 7. Publish
        out_msg = String()
        out_msg.data = edited_xml
        self._republishing = True
        self.mission_pub.publish(out_msg)
        self._republishing = False
        self.get_logger().info("  Published edited mission XML to /mission/xml")

        # 8. Store in memory and update status
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

    # ------------------------------------------------------------------
    # World state client — identical SSE pattern from test_llm_world_state.py
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
    # LLM client
    # ------------------------------------------------------------------
    # 

    def _call_openai(self, prompt: str) -> str:
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": prompt},
        ]
        answered = False
        cmp = None
        while not answered:
            try:
                cmp = completion(
                    model=CLOUD_MODEL,
                    messages=messages,
                    temperature=CLOUD_MODEL_TEMPERATURE,
                    max_tokens=CLOUD_MODEL_MAX_TOKENS,
                )
                answered = True
            except litellm.exceptions.RateLimitError as exc:
                self.get_logger().warn(f"  OpenAI rate limited, retrying in 1s: {exc}")
                time.sleep(1)
        return cmp.choices[0].message.content.strip()

    # ------------------------------------------------------------------
    # XSD schema — loaded once from amiga_ros2_behavior_tree's installed share dir
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

    def _load_xsd(self) -> Optional["etree.XMLSchema"]:
        path = self._resolve_xsd_path()
        if not path:
            return None
        try:
            with open(path, "rb") as f:
                return etree.XMLSchema(etree.parse(f))
        except (OSError, etree.XMLSyntaxError, etree.XMLSchemaParseError) as exc:
            self.get_logger().error(f"Failed to load mission XSD from {path}: {exc}")
            return None

    def _read_xsd_text(self) -> str:
        path = self._resolve_xsd_path()
        if not path:
            return ""
        try:
            with open(path, "r") as f:
                return f.read()
        except OSError as exc:
            self.get_logger().error(f"Failed to read mission XSD text from {path}: {exc}")
            return ""

    def _validate_xml(self, xml_str: str) -> Tuple[bool, str]:
        """Returns (is_valid, error_message)."""
        if self.xsd_schema is None:
            return True, "schema unavailable — skipped validation"
        try:
            doc = etree.fromstring(xml_str.encode("utf-8"))
        except etree.XMLSyntaxError as exc:
            return False, f"not well-formed XML: {exc}"
        if self.xsd_schema.validate(doc):
            return True, ""
        return False, str(self.xsd_schema.error_log)

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

def _strip_code_fence(text: str) -> str:
    """Remove a leading/trailing markdown code fence if the LLM added one anyway."""
    text = text.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        lines = lines[1:]  # drop opening ``` or ```xml
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        text = "\n".join(lines).strip()
    return text


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = MissionPlannerNode()

    # Spin in a background thread so uvicorn can run in the main thread
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


if __name__ == "__main__":
    main()