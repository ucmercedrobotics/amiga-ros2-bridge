#!/usr/bin/env python3
"""
amiga_autonomy_agent.py

ROS2 autonomy agent that:
- Listens to /rosout and /bt/status_change
- Uses LiteLLM for planning (tool-based loop)
- Understands BOTH running system and available codebase
- Can:
    * inspect ROS graph
    * search codebase
    * run existing nodes
    * modify files
    * publish missions
    * generate new capabilities (last resort)

Dependencies:
  pip install litellm lxml

Env:
  LITELLM_MODEL (default: claude-sonnet-4-6)
  API keys for provider
  AMIGA_XSD_PATH (optional)
"""

import json
import os
import re
import subprocess
import textwrap
import warnings
from typing import Any, Dict, List, Optional

from dotenv import load_dotenv
import litellm
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Suppress noisy warnings
# ---------------------------------------------------------------------------

warnings.filterwarnings("ignore", category=UserWarning, module="pydantic")

# ---------------------------------------------------------------------------

ANSI_ESCAPE = re.compile(r'\x1b\[[0-9;]*m')
LEVEL_MAP = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

MODEL = os.environ.get("LITELLM_MODEL", "claude-sonnet-4-6")

LOG_WINDOW_SEC = 30.0
FAILURE_CONTEXT_SEC = 3.0
MAX_STEPS = 20

# How many chars of tool output are sent back to the LLM per turn.
# Kept intentionally small — the LLM should read in chunks, not dumps.
# File reads use FILE_READ_CHARS; everything else uses RESULT_HISTORY_CHARS.
RESULT_HISTORY_CHARS = 1000
FILE_READ_CHARS      = 800   # ~20-30 lines — enough to see structure

# How many recent turns to keep verbatim; older ones are compressed to 1 line.
COMPRESS_AFTER_STEPS = 3

CODEBASE_PATH = "/amiga-ros2-bridge"

# ---------------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------------

def _box(title: str, width: int = 60) -> str:
    bar = "─" * (width - 2)
    return f"┌{bar}┐\n│ {title:<{width - 3}}│\n└{bar}┘"

def _section(label: str, value: str, indent: int = 4) -> str:
    pad = " " * indent
    wrapped = textwrap.fill(value, width=80 - indent,
                            initial_indent=pad, subsequent_indent=pad)
    return f"  [{label}]\n{wrapped}"

def _result_status(output: str, success: bool) -> str:
    icon = "✓" if success else "✗"
    return f"{icon} {'OK' if success else 'FAILED'}"

# ---------------------------------------------------------------------------


class AmigaAutonomyAgent(Node):

    def __init__(self):
        super().__init__("amiga_autonomy_agent")

        self.log_buffer: List[Dict] = []
        self.memory: List[Dict] = []
        self.current_mission_xml: Optional[str] = None
        self._republishing = False
        self._tool_cache: Dict[str, Dict] = {}   # keyed by "tool|args_json"

        self.xsd_path = os.environ.get(
            "AMIGA_XSD_PATH",
            "/amiga-ros2-bridge/amiga_ros2_behavior_tree/schemas/amiga_btcpp.xsd"
        )
        self.xsd_schema = self._load_xsd()

        # Cache available packages at startup
        self.available_packages = self.tool_list_packages()

        # ROS subscriptions / publishers
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.create_subscription(String, '/mission/xml', self.on_mission, 10)
        self.create_subscription(String, '/bt/status_change', self.on_bt_failure, 10)

        self.mission_pub = self.create_publisher(String, '/mission/xml', 10)

        load_dotenv(".env")

        self.get_logger().info(_box("Amiga Autonomy Agent — ready"))

    # -----------------------------------------------------------------------
    # LLM
    # -----------------------------------------------------------------------

    def _llm_json(self, messages: List[Dict], max_tokens: int = 10_000) -> dict:
        resp = litellm.completion(
            model=MODEL,
            messages=messages,
            max_tokens=max_tokens,
            temperature=0.2,
        )

        content = resp["choices"][0]["message"]["content"]

        try:
            return json.loads(content)
        except Exception:
            try:
                start = content.find("{")
                end = content.rfind("}") + 1
                return json.loads(content[start:end])
            except Exception as e:
                self.get_logger().error(f"JSON parse failed:\n{content[:400]}")
                raise e

    # -----------------------------------------------------------------------
    # Tools
    # -----------------------------------------------------------------------

    def tool_ros2_cli(self, args: str = None, cmd: str = None, command: str = None) -> Dict:
        cmd_str = args or cmd or command
        if not cmd_str:
            return {"success": False, "output": "error: no command provided"}
        try:
            out = subprocess.check_output(
                f"ros2 {cmd_str}", shell=True, stderr=subprocess.STDOUT
            )
            return {"success": True, "output": out.decode()}
        except subprocess.CalledProcessError as e:
            return {"success": False, "output": e.output.decode()}

    def tool_find_files(self, pattern: str,
                        path: str = "/",
                        max_results: int = 20) -> Dict:
        """
        Find files whose name matches a glob pattern under path.
        pattern supports '*' wildcards, e.g. '*.engine' or 'yolo*.pt'.
        search is NOT shell-expanded — no injection possible.
        """
        try:
            out = subprocess.check_output(
                ["find", path, "-name", pattern],
                stderr=subprocess.DEVNULL,
            )
            lines = [l for l in out.decode().splitlines() if l]
            truncated = len(lines) > max_results
            result = "\n".join(lines[:max_results])
            if truncated:
                result += f"\n… ({len(lines) - max_results} more — narrow path or pattern)"
            return {"success": True, "output": result or "(no matches)"}
        except subprocess.CalledProcessError as e:
            return {"success": False, "output": e.output.decode()}

    def tool_list_packages(self) -> str:
        return subprocess.check_output("ros2 pkg list", shell=True).decode()

    def tool_list_executables(self, package: str) -> Dict:
        try:
            out = subprocess.check_output(
                f"ros2 pkg executables {package}", shell=True
            )
            return {"success": True, "output": out.decode()}
        except subprocess.CalledProcessError as e:
            return {"success": False, "output": e.output.decode()}

    def tool_run_node(self, package: str, executable: str,
                      params: Optional[Dict[str, Any]] = None,
                      args: Optional[str] = None) -> Dict:
        """
        Launch a ROS2 node and wait briefly to see if it crashes.
        Returns a structured result with success/failure and any output.

        Args:
            package:    ROS2 package name.
            executable: Executable name within the package.
            params:     Optional dict of ROS2 parameter overrides, e.g.
                        {"yolo_model": "/path/to/model.engine"}.
            args:       Raw extra CLI arguments appended verbatim (e.g.
                        "--ros-args -p key:=value").
        """
        cmd = ["ros2", "run", package, executable]

        # Build --ros-args parameter overrides
        if params:
            param_args = []
            for k, v in params.items():
                param_args += ["-p", f"{k}:={v}"]
            cmd += ["--ros-args"] + param_args

        if args:
            cmd += args.split()

        cmd_str = " ".join(cmd)

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )

            # Wait up to 5 s; if it's still alive after that, assume it's running
            try:
                out, _ = proc.communicate(timeout=5)
                exit_code = proc.returncode

                if exit_code != 0:
                    return {
                        "success": False,
                        "exit_code": exit_code,
                        "output": out,
                        "note": (
                            "Node exited with a non-zero exit code. "
                            "Read the output above carefully — it likely contains "
                            "the error (missing file, bad parameter, etc.)."
                        ),
                    }
                return {"success": True, "exit_code": 0, "output": out}

            except subprocess.TimeoutExpired:
                # Still running after 5 s — that's the happy path for a daemon node
                return {
                    "success": True,
                    "output": "node is still running after 5 s (daemon behaviour — normal)",
                    "pid": proc.pid,
                }

        except Exception as e:
            return {"success": False, "output": str(e)}

    def tool_search_codebase(self, query: str, path: str = CODEBASE_PATH) -> Dict:
        try:
            out = subprocess.check_output(
                f'grep -Rn "{query}" {path} | head -n 80',
                shell=True,
            )
            return {"success": True, "output": out.decode()}
        except subprocess.CalledProcessError as e:
            return {"success": False, "output": e.output.decode()}

    def tool_list_launch_files(self, path: str = CODEBASE_PATH) -> Dict:
        try:
            out = subprocess.check_output(
                f"find {path} -name '*.launch.py'", shell=True
            )
            return {"success": True, "output": out.decode()}
        except subprocess.CalledProcessError as e:
            return {"success": False, "output": e.output.decode()}

    def tool_read_file(self, path: str,
                       start_line: Optional[int] = None,
                       end_line: Optional[int] = None) -> Dict:
        try:
            with open(path, "r") as f:
                lines = f.readlines()
            total = len(lines)
            lo = (start_line - 1) if start_line else 0
            hi = end_line if end_line else total
            lo, hi = max(0, lo), min(total, hi)
            content = "".join(lines[lo:hi])
            meta = f"[{total} lines total | showing {lo+1}–{hi}]"
            return {"success": True, "output": f"{meta}\n{content}", "total_lines": total}
        except Exception as e:
            return {"success": False, "output": str(e)}

    def tool_write_file(self, path: str, content: str) -> Dict:
        try:
            with open(path, "w") as f:
                f.write(content)
            return {"success": True, "output": f"wrote {len(content)} bytes to {path}"}
        except Exception as e:
            return {"success": False, "output": str(e)}

    def tool_publish(self, topic: str, data: str) -> Dict:
        if topic == "/mission/xml":
            msg = String()
            msg.data = data
            self._republishing = True
            self.mission_pub.publish(msg)
            self._republishing = False
            return {"success": True, "output": f"published {len(data)} chars to {topic}"}
        return {"success": False, "output": f"unsupported topic: {topic}"}

    def tool_generate_capability(self, description: str) -> Dict:
        prompt = f"""
Create a new ROS2 capability.

Description: {description}

Return JSON:
{{"xsd_extension":"...","ros2_node_code":"...","bt_xml_fragment":"...","node_executable_name":"..."}}
"""
        try:
            gen = self._llm_json(
                [{"role": "system", "content": "Return ONLY valid JSON."},
                 {"role": "user", "content": prompt}],
                max_tokens=3000,
            )
            result = self._deploy_generated(gen)
            return {"success": True, "output": result}
        except Exception as e:
            return {"success": False, "output": str(e)}

    # -----------------------------------------------------------------------

    # Tools that are safe to cache within a planning session.
    # write_file / publish / run_node are intentionally excluded.
    _CACHEABLE = {"ros2_cli", "find_files", "list_packages", "list_executables",
                  "search_codebase", "list_launch_files", "read_file"}

    def execute_tool(self, name: str, args: Dict[str, Any]) -> Dict:
        """Dispatch tool call; cache read-only calls within the session."""
        cache_key = f"{name}|{json.dumps(args, sort_keys=True)}"

        if name in self._CACHEABLE and cache_key in self._tool_cache:
            cached = self._tool_cache[cache_key]
            self.get_logger().info("  │  ♻ cache hit")
            return {**cached, "cached": True}

        try:
            if name == "ros2_cli":
                result = self.tool_ros2_cli(**args)
            elif name == "find_files":
                result = self.tool_find_files(**args)
            elif name == "list_packages":
                result = {"success": True, "output": self.tool_list_packages()}
            elif name == "list_executables":
                result = self.tool_list_executables(**args)
            elif name == "run_node":
                result = self.tool_run_node(**args)
            elif name == "search_codebase":
                result = self.tool_search_codebase(**args)
            elif name == "list_launch_files":
                result = self.tool_list_launch_files(**args)
            elif name == "read_file":
                result = self.tool_read_file(**args)
            elif name == "write_file":
                result = self.tool_write_file(**args)
            elif name == "publish":
                result = self.tool_publish(**args)
            elif name == "generate_capability":
                result = self.tool_generate_capability(**args)
            else:
                result = {"success": False, "output": f"unknown tool: {name}"}
        except Exception as e:
            result = {"success": False, "output": f"dispatch error: {e}"}

        if name in self._CACHEABLE and result.get("success"):
            self._tool_cache[cache_key] = result

        return result

    @staticmethod
    def _truncate_for_llm(tool: str, output: str) -> str:
        """Return a token-budget-friendly slice of tool output."""
        cap = FILE_READ_CHARS if tool == "read_file" else RESULT_HISTORY_CHARS
        if len(output) <= cap:
            return output
        return output[:cap] + f"\n… [truncated — {len(output) - cap} more chars]"

    @staticmethod
    def _compress_turn(tool: str, args: Dict, success: bool, output: str) -> str:
        """One-line summary of a completed turn for older history slots."""
        status = "✓" if success else "✗"
        key_arg = (
            args.get("path") or args.get("query") or args.get("args") or
            args.get("cmd") or args.get("package") or ""
        )
        snippet = output.splitlines()[0][:80] if output else ""
        return f"{status} {tool}({key_arg!r}) → {snippet}"

    # -----------------------------------------------------------------------
    # Planner
    # -----------------------------------------------------------------------

    SYSTEM_PROMPT = """\
You are a ROS2 autonomy agent. Your job is to resolve failures by reasoning
carefully about what you observe, then taking the minimal necessary action.

## Decision order (strict — do not skip steps)
1. Inspect the running system to understand what's happening.
2. Search the codebase to understand existing capabilities and configuration.
3. Attempt to run an existing node — but ONLY after you know its requirements
   (model files, parameters, topics, etc.) from step 2.
4. Generate a new capability ONLY if nothing in the codebase can solve the problem.

## Handling tool results
- Every tool result includes "success": true/false.
- If success is false, read "output" carefully before choosing the next action.
- A failed tool call means you must adapt — do NOT retry the same call unchanged.
- Common pattern: node fails → search codebase for parameter defaults → re-run
  with corrected params dict.

## Tool reference
ros2_cli         {"args": "node list"}
find_files       {"pattern": "*.engine"}                       <- find by filename glob
find_files       {"pattern": "yolo*.pt", "path": "/amiga-ros2-bridge"}  <- scoped search
list_packages    {}
list_executables {"package": "amiga_navigation"}
search_codebase  {"query": "yolo_model", "path": "/amiga-ros2-bridge"}  <- grep in source
list_launch_files {}
read_file        {"path": "/abs/path"}                          <- first ~25 lines + total count
read_file        {"path": "/abs/path", "start_line": 50, "end_line": 100}
write_file       {"path": "...", "content": "..."}
run_node         {"package": "...", "executable": "...",
                  "params": {"param_name": "value"},
                  "args": "--extra-cli-args"}
publish          {"topic": "/mission/xml", "data": "..."}
generate_capability {"description": "..."}

## Output truncation
Tool outputs are capped before being sent to you. If a file is truncated, use
start_line/end_line to read the specific section you need. Never re-read the
same file with the same arguments — use a different range instead.

## Response format
Return ONLY valid JSON — no preamble, no markdown.

Next action:
{"thought": "...", "tool": "...", "args": {...}, "done": false}

All done (nothing more to do):
{"thought": "...", "done": true}
"""

    def run_planner(self, event: dict, context: List[dict]):
        """
        Observe → Reason → Act loop.

        Token budget controls:
        - Tool outputs are capped before being sent to the LLM.
        - Turns older than COMPRESS_AFTER_STEPS are collapsed to a single
          summary line so the context window doesn't balloon.
        - Read-only tool calls are cached; duplicate calls never hit the LLM.
        """
        self._tool_cache.clear()   # fresh cache per planning session

        self.get_logger().info("\n" + _box(f"▶ PLANNER  event={event.get('node','?')}"))

        # Compact memory: just the outcome of each prior session, not full history
        memory_summary = [
            {"event": m["event"].get("node", "?"),
             "steps": len(m["history"]),
             "last": m["history"][-1]["tool"] if m["history"] else "—"}
            for m in self.memory[-3:]
        ]

        initial_user = (
            f"## Failure event\n{json.dumps(event, indent=2)}\n\n"
            f"## Recent ROS logs (±{FAILURE_CONTEXT_SEC}s around failure)\n"
            f"{json.dumps(context, indent=2)}\n\n"
            f"## Available ROS2 packages (excerpt)\n"
            f"{self.available_packages[:1500]}\n\n"
            f"## Prior sessions (compact)\n"
            f"{json.dumps(memory_summary, indent=2)}\n\n"
            "Begin. Inspect the system, then decide your first action."
        )

        # ── conversation history ──────────────────────────────────────────
        # Each entry is {"role": ..., "content": ..., "_summary": ...}
        # _summary is used when the turn gets compressed.
        turns: List[Dict] = []   # alternating assistant / user result pairs

        messages_base: List[Dict] = [
            {"role": "system", "content": self.SYSTEM_PROMPT},
            {"role": "user",   "content": initial_user},
        ]

        history_for_memory = []

        for step in range(MAX_STEPS):

            # Build the message list: base + compressed old turns + recent verbatim
            recent_cutoff = max(0, len(turns) - COMPRESS_AFTER_STEPS * 2)
            compressed = []
            for t in turns[:recent_cutoff]:
                if t.get("_summary"):
                    compressed.append({"role": t["role"], "content": t["_summary"]})
            verbatim = [{"role": t["role"], "content": t["content"]}
                        for t in turns[recent_cutoff:]]

            messages = messages_base + compressed + verbatim

            self.get_logger().info(f"  ┌─ step {step + 1}/{MAX_STEPS} — asking LLM …")

            try:
                decision = self._llm_json(messages, max_tokens=400)
            except Exception as e:
                self.get_logger().error(f"  └─ LLM error: {e}")
                break

            thought = decision.get("thought", "")
            self.get_logger().info(
                f"  │  💭 {textwrap.shorten(thought, width=120, placeholder='…')}"
            )

            if decision.get("done"):
                self.get_logger().info("  └─ ✓ planner done")
                break

            tool = decision.get("tool", "")
            args = decision.get("args", {})

            self.get_logger().info(
                f"  │  🔧 tool={tool}  args={json.dumps(args, separators=(',',':'))}"
            )

            result   = self.execute_tool(tool, args)
            success  = result.get("success", False)
            output   = result.get("output", "")
            cached   = result.get("cached", False)

            status_icon = "✓" if success else "✗"
            cache_tag   = " (cached)" if cached else ""
            self.get_logger().info(f"  │  {status_icon} {'OK' if success else 'FAILED'}{cache_tag}")

            for line in output.splitlines()[:30]:
                self.get_logger().info(f"  │      {line}")
            if len(output.splitlines()) > 30:
                self.get_logger().info(
                    f"  │      … (+{len(output.splitlines()) - 30} more lines)"
                )
            self.get_logger().info("  │")

            history_for_memory.append({
                "step": step + 1, "tool": tool, "args": args,
                "success": success,
                "output_excerpt": output[:400],
            })

            # What the LLM actually receives (capped)
            llm_output = self._truncate_for_llm(tool, output)
            result_msg = (
                f"## Tool result  (tool={tool}, success={success})\n\n"
                f"```\n{llm_output}\n```\n\n"
                + ("The tool FAILED. Adapt — do NOT retry with the same arguments.\n"
                   if not success else
                   "Succeeded. Next step, or done=true if resolved.\n")
            )

            summary = self._compress_turn(tool, args, success, output)

            turns.append({
                "role": "assistant",
                "content": json.dumps(decision),
                "_summary": f"[step {step+1}] thought: {textwrap.shorten(thought, 60)}",
            })
            turns.append({
                "role": "user",
                "content": result_msg,
                "_summary": f"[step {step+1} result] {summary}",
            })

        self.memory.append({"event": event, "history": history_for_memory})
        self.get_logger().info("─" * 60 + "\n")

    # -----------------------------------------------------------------------
    # ROS Callbacks
    # -----------------------------------------------------------------------

    def on_mission(self, msg: String):
        if not self._republishing:
            self.current_mission_xml = msg.data

    def on_log(self, msg: Log):
        if "BTStatusPublisher" in msg.msg:
            return

        self.log_buffer.append({
            "stamp": self._stamp_to_sec(msg.stamp),
            "level": LEVEL_MAP.get(msg.level, str(msg.level)),
            "node": msg.name,
            "msg": ANSI_ESCAPE.sub("", msg.msg),
        })

        now = self._stamp_to_sec(self.get_clock().now().to_msg())
        self.log_buffer = [
            l for l in self.log_buffer if now - l["stamp"] < LOG_WINDOW_SEC
        ]

    def on_bt_failure(self, msg: String):
        event = json.loads(msg.data)
        failure_sec = event["timestamp_ms"] / 1000.0

        context = [
            l for l in self.log_buffer
            if failure_sec - FAILURE_CONTEXT_SEC <= l["stamp"] <= failure_sec
        ]

        self.run_planner(event, context)

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _stamp_to_sec(self, stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _load_xsd(self) -> str:
        if os.path.exists(self.xsd_path):
            with open(self.xsd_path, "r") as f:
                return f.read()
        return ""

    def _deploy_generated(self, gen: dict) -> str:
        name = gen.get("node_executable_name", "generated_node")
        node_path = f"/tmp/{name}.py"
        with open(node_path, "w") as f:
            f.write(gen["ros2_node_code"])
        os.chmod(node_path, 0o755)
        subprocess.Popen(["python3", node_path])
        return f"generated and deployed: {node_path}"

    def destroy_node(self):
        super().destroy_node()


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = AmigaAutonomyAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()