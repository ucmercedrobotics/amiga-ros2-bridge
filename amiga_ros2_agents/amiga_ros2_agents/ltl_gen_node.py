"""
ltl_gen_node.py

LTL generation agent (A2A, port 20001).

Takes a mission in plain English — "go visit trees 1 through 3 and sample the
leaves at each" — and returns an LTL formula in Promela/SPIN syntax.

Two ways in:
    - A2A:  send a text message to http://localhost:20001/ (see tools/ltl_client_demo.py)
    - ROS:  publish a std_msgs/String to /mission/text

Either way the formula is published to /mission/ltl.
"""

import asyncio
from threading import Lock, Thread
from typing import Dict

import rclpy
from a2a.server.agent_execution import AgentExecutor, RequestContext
from a2a.server.events import EventQueue
from rclpy.node import Node
from std_msgs.msg import String

from . import llm
from .a2a_server import agent_message, serve_agent
from .agent_card import LTL_AGENT_CARD

A2A_PORT = 20001

# A reply has to contain at least one of these to count as a formula rather than
# prose. Bare "!" is deliberately excluded — it matches ordinary exclamations
# ("Sure! Here is...") and a pure negation isn't a temporal property anyway.
LTL_TOKENS = ("[]", "<>", " U ", "X ", "&&", "||", "->", "<->")

# Punctuation that means the model answered in sentences. "." alone is allowed
# mid-token so Promela struct access (robot.at_tree_1) keeps working once the
# typedefs land; only sentence-shaped usage is rejected.
PROSE_MARKERS = (". ", ".\n", ", ", "?", ";")

# ---------------------------------------------------------------------------
# System prompt
#
# TODO: once the Promela typedefs exist, inject them here as the atomic
# proposition vocabulary and drop the "invent readable identifiers" rule.
# ---------------------------------------------------------------------------
LTL_SYSTEM_PROMPT = """\
You translate missions for an autonomous agricultural robot into Linear \
Temporal Logic.

The formula must be valid Promela/SPIN LTL — it has to parse with `spin -f`.

## Operators (use ONLY these)
- []   always
- <>   eventually
- U    strong until
- X    next
- &&   and
- ||   or
- !    not
- ->   implies
- <->  equivalence

## Atomic propositions
- Must be plain Promela identifiers: lowercase letters, digits and underscores only.
- No spaces, no arguments, no dots. Write `visit_tree_3`, never `visit(tree 3)` \
or `visit tree 3`.
- Invent readable identifiers from the mission text and reuse the same \
identifier for the same fact throughout the formula.

## Output
- Return ONLY the formula on a single line.
- No `ltl name { ... }` wrapper, no code fences, no markdown, no comments, no \
explanation, no trailing period.

## Examples
Mission: "visit tree 1 and then tree 2"
<>(at_tree_1 && <>at_tree_2)

Mission: "sample every tree in the row, trees 1 to 3"
<>sampled_tree_1 && <>sampled_tree_2 && <>sampled_tree_3

Mission: "never enter the mud patch"
[]!in_mud_patch

Mission: "keep patrolling the row until the battery is low"
[](patrolling U battery_low)
"""


class LtlGenNode(Node):
    """ROS node holding the LTL agent's state and its /mission/ltl publisher."""

    def __init__(self):
        super().__init__("ltl_gen")
        self._lock = Lock()
        self._status: Dict = {
            "model": llm.MODEL,
            "missions_translated": 0,
            "last_mission": None,
            "last_formula": None,
            "last_error": None,
        }

        self.ltl_pub = self.create_publisher(String, "/mission/ltl", 10)
        self.create_subscription(String, "/mission/text", self._on_mission_text, 10)

        self.get_logger().info(
            f"LtlGenNode started — model={llm.MODEL} api_base={llm.API_BASE or 'provider default'}"
        )

    # ------------------------------------------------------------------
    # Public API (called from the A2A executor and the ROS subscription)
    # ------------------------------------------------------------------

    def generate(self, mission: str) -> Dict:
        """Translate `mission` to LTL, publish it, and return the result.

        Never raises — a failure comes back as {"ok": False, "error": ...} so the
        A2A caller always gets an answer.
        """
        mission = mission.strip()
        result = {
            "mission": mission,
            "formula": None,
            "model": llm.MODEL,
            "ok": False,
            "error": None,
        }

        if not mission:
            result["error"] = "empty mission"
            return self._record(result)

        self.get_logger().info(f"Generating LTL for: {mission}")

        try:
            reply = llm.complete(LTL_SYSTEM_PROMPT, mission)
        except Exception as exc:
            result["error"] = f"LLM call failed: {exc}"
            self.get_logger().error(f"  {result['error']}")
            return self._record(result)

        formula = _clean_formula(reply)
        ok, reason = _looks_like_ltl(formula)
        if not ok:
            result["error"] = f"{reason}; model returned: {reply[:200]}"
            self.get_logger().error(f"  Rejected reply — {result['error']}")
            return self._record(result)

        result["formula"] = formula
        result["ok"] = True

        msg = String()
        msg.data = formula
        self.ltl_pub.publish(msg)
        self.get_logger().info(f"  Published to /mission/ltl: {formula}")

        return self._record(result)

    def get_status(self) -> Dict:
        with self._lock:
            return dict(self._status)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_mission_text(self, msg: String):
        """Run the LLM off the spin thread so callbacks stay responsive."""
        Thread(target=self.generate, args=(msg.data,), daemon=True).start()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _record(self, result: Dict) -> Dict:
        with self._lock:
            self._status["last_mission"] = result["mission"]
            if result["ok"]:
                self._status["missions_translated"] += 1
                self._status["last_formula"] = result["formula"]
                self._status["last_error"] = None
            else:
                self._status["last_error"] = result["error"]
        return result


class LtlGenExecutor(AgentExecutor):
    """A2A entry point — reads the caller's mission text and answers with LTL."""

    def __init__(self, ros_node: LtlGenNode):
        self.node = ros_node

    async def execute(self, context: RequestContext, event_queue: EventQueue) -> None:
        mission = (context.get_user_input() or "").strip()
        if not mission:
            await event_queue.enqueue_event(
                agent_message(
                    text="No mission text in the request — send the mission as a text part.",
                    data={"ok": False, "error": "empty mission"},
                )
            )
            return

        # llm.complete() blocks; keep it off the event loop or one request stalls
        # the whole server for the duration of the model call.
        loop = asyncio.get_running_loop()
        result = await loop.run_in_executor(None, self.node.generate, mission)

        await event_queue.enqueue_event(
            agent_message(text=result["formula"] or result["error"] or "", data=result)
        )

    async def cancel(self, context: RequestContext, event_queue: EventQueue) -> None:
        pass


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------


def _clean_formula(reply: str) -> str:
    """Reduce a model reply to a single-line candidate formula."""
    text = llm.strip_code_fence(reply)
    # Some models still wrap the answer as `ltl name { ... }` despite the prompt
    if text.startswith("ltl") and "{" in text and text.rstrip().endswith("}"):
        text = text[text.index("{") + 1 : text.rstrip().rindex("}")]
    # Collapse to one line — SPIN accepts multi-line, but a single line keeps the
    # /mission/ltl payload and the logs readable.
    return " ".join(text.split()).strip()


def _looks_like_ltl(formula: str) -> tuple:
    """Cheap smoke test — is this a formula or did the model answer in prose?

    Returns (ok, reason). Not a substitute for `spin -f`; it only exists to keep
    obvious garbage off /mission/ltl.
    """
    if not formula:
        return False, "empty formula"
    if formula.count("(") != formula.count(")"):
        return False, "unbalanced parentheses"
    if not any(token in f" {formula} " for token in LTL_TOKENS):
        return False, "no LTL or boolean operator found"
    if formula.endswith(".") or any(m in formula for m in PROSE_MARKERS):
        return False, "reply reads as prose, not a bare formula"
    return True, ""


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    rclpy.init()
    node = LtlGenNode()
    serve_agent(node, LtlGenExecutor(node), LTL_AGENT_CARD, A2A_PORT)


if __name__ == "__main__":
    main()
