"""
ltl_gen_node.py

LTL generation agent.

Takes a mission in plain English — "go visit trees 1 through 3 and sample the
leaves at each" — and returns an LTL formula in Promela/SPIN syntax.

Two ways in:
    - service:  /mission/generate_ltl (amiga_interfaces/srv/GenerateLTL) — returns
                the formula to the caller
    - topic:    publish a std_msgs/String to /mission/text — fire and forget

Either way the formula is published to /mission/ltl.

The system prompt lives in prompts/ltl_gen/system.j2. Set the `ap_vocabulary`
parameter to pin the atomic propositions the model may use; left empty (the
default) the model invents its own identifiers.
"""

from threading import Lock, Thread
from typing import Dict

from amiga_interfaces.srv import GenerateLTL
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from . import llm, prompts, spin
from .status import StatusPublisher

# A reply has to contain at least one of these to count as a formula rather than
# prose. Bare "!" is deliberately excluded — it matches ordinary exclamations
# ("Sure! Here is...") and a pure negation isn't a temporal property anyway.
LTL_TOKENS = ("[]", "<>", " U ", "X ", "&&", "||", "->", "<->")

# Punctuation that means the model answered in sentences. "." alone is allowed
# mid-token so Promela struct access (robot.at_tree_1) keeps working once the
# typedefs land; only sentence-shaped usage is rejected.
PROSE_MARKERS = (". ", ".\n", ", ", "?", ";")


class LtlGenNode(Node):
    """ROS node holding the LTL agent's state, its service and its publisher."""

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

        # Empty vocabulary => the model invents identifiers (the historical
        # behaviour). Populate it from the Promela typedefs once they exist and
        # the prompt switches to a fixed AP vocabulary with no code change.
        #
        # Two rclpy quirks force the shape of this:
        #   - the default is [""] and not [], because rclpy infers a parameter's
        #     type from its default and infers BYTE_ARRAY from an empty list,
        #     which then rejects any string-array override;
        #   - an override of [] leaves the parameter *uninitialized* rather than
        #     empty, so plain get_parameter() raises.
        # get_parameter_or covers both; the comprehension drops the placeholder.
        self.declare_parameter("ap_vocabulary", [""])
        ap_param = self.get_parameter_or(
            "ap_vocabulary", Parameter("ap_vocabulary", Parameter.Type.STRING_ARRAY, [])
        )
        ap_vocabulary = [
            ap for ap in ap_param.get_parameter_value().string_array_value if ap
        ]
        self.system_prompt = prompts.render(
            "ltl_gen/system.j2", ap_vocabulary=ap_vocabulary
        )

        # llm.complete() blocks for seconds. A reentrant group plus the
        # multithreaded executor in main() keeps a service call from stalling the
        # /mission/text subscription and vice versa.
        group = ReentrantCallbackGroup()

        self.ltl_pub = self.create_publisher(String, "/mission/ltl", 10)
        self.create_subscription(
            String, "/mission/text", self._on_mission_text, 10, callback_group=group
        )
        self.create_service(
            GenerateLTL,
            "/mission/generate_ltl",
            self._on_generate_request,
            callback_group=group,
        )

        self.status = StatusPublisher(self)
        self.status.publish(self.get_status())

        self.get_logger().info(
            f"LtlGenNode started — model={llm.MODEL} "
            f"api_base={llm.API_BASE or 'provider default'} "
            f"ap_vocabulary={ap_vocabulary or 'model-invented'}"
        )

    # ------------------------------------------------------------------
    # Public API (called from the service callback and the ROS subscription)
    # ------------------------------------------------------------------

    def generate(self, mission: str) -> Dict:
        """Translate `mission` to LTL, publish it, and return the result.

        Never raises — a failure comes back as {"ok": False, "error": ...} so the
        caller always gets an answer.
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
            reply = llm.complete(self.system_prompt, mission)
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

    def _on_generate_request(self, request, response):
        """Service entry point — blocks for the duration of the model call."""
        result = self.generate(request.mission)
        response.ok = result["ok"]
        response.formula = result["formula"] or ""
        response.error = result["error"] or ""
        response.model = result["model"]
        return response

    def _on_mission_text(self, msg: String):
        """Run the LLM off the callback so the executor stays responsive."""
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
        self.status.publish(self.get_status())
        return result


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
    spin.run(LtlGenNode, multithreaded=True)


if __name__ == "__main__":
    main()
