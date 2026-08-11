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

The system prompt lives in prompts/ltl_gen/system.j2, and the translation itself
in ltl.py -- shared with the arbiter, which needs a formula inside the gate that
decides whether a replanned mission may be published and cannot make a service
call from there.

Left empty (the default), `ap_vocabulary` lets the model derive propositions
from the mission text using the naming scheme the template mandates
(`at_tree_<id>`, `sampled_tree_<id>`). That scheme is the contract with
promela.py, which emits the same names from the behaviour tree. Setting the
parameter pins the vocabulary instead; do not populate it from a plan, which
would tell the model what the plan already does and make verification vacuous.
"""

from threading import Lock, Thread
from typing import Dict

from amiga_interfaces.srv import GenerateLTL
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from ..runtime import llm, spin
from ..runtime.status import StatusPublisher
from . import ltl

# The formula handling lives in ltl.py so the arbiter can reach it without a
# service call. Re-exported here because this module is where callers look.
LTL_TOKENS = ltl.LTL_TOKENS
PROSE_MARKERS = ltl.PROSE_MARKERS
_clean_formula = ltl.clean_formula
_looks_like_ltl = ltl.looks_like_ltl


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
        self.system_prompt = ltl.system_prompt(ap_vocabulary)

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

        formula = ltl.generate(mission, self.system_prompt)
        if not formula.ok:
            result["error"] = formula.error
            self.get_logger().error(f"  Rejected reply — {result['error']}")
            return self._record(result)

        result["formula"] = formula.text
        result["ok"] = True

        msg = String()
        msg.data = formula.text
        self.ltl_pub.publish(msg)
        self.get_logger().info(f"  Published to /mission/ltl: {formula.text}")

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
# Entry point
# ---------------------------------------------------------------------------


def main():
    spin.run(LtlGenNode, multithreaded=True)


if __name__ == "__main__":
    main()
