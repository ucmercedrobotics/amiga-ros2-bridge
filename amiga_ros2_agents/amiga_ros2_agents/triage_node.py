"""
triage_node.py

The first of the two reasoning points: `interpret_anomaly`.

The behaviour tree is the catalyst. A node fails, `/bt/status_change` fires, and
the existing self-correction loop (mission_planner -> arbiter -> /mission/xml)
tries to edit its way out of it. Most faults end there. This node is for the
ones that do not.

Two jobs, and they are deliberately different in kind:

**Escalation is deterministic.** "Has local recovery run out?" needs no model.
The planner already answers it: the arbiter publishes /mission/abort when the
mission is no longer viable, and the planner publishes /mission/planner_status
when it gives up on a fault. Either one means the robot cannot fix this itself,
and this node forwards it to the coordinator on /coordination/infeasible. A
model asked to re-derive that would only add latency and a way to be wrong.

**The decision is not.** "What should be done about it?" is a judgement over
unstructured evidence -- the fault event, the /rosout lines around it, where the
robot is, what was already tried, who else is out there. That is the service
this node serves, and it is where the model earns its place.

The answer is constrained to three typed actions (re_delegate, add_task,
drop_task) by InterpretAnomaly.srv, and the coordinator refuses anything else.
That constraint is the whole design: the model chooses *among decisions the
state machine already knows how to execute*, so every path below it stays
deterministic and testable with this node swapped for a stub.

    /bt/status_change ─┐
    /rosout ───────────┤
    /world_state ──────┼──▶ [context buffers]
    /mission/xml ──────┤            │
    /mission/abort ────┤            ├──▶ /coordination/interpret_anomaly (srv)
    /mission/planner_status ────────┴──▶ /coordination/infeasible (topic)

Prompts live in prompts/triage/.
"""

import json
import re
import time
from collections import deque
from threading import Lock
from typing import Dict, List, Optional

from amiga_interfaces.srv import InterpretAnomaly
from lxml import etree
from rcl_interfaces.msg import Log
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from . import llm, prompts, spin
from .status import StatusPublisher

# ---------------------------------------------------------------------------
# Context-window limits
#
# Same shape as the mission planner's, and for the same reason: a model given
# the whole log buffer reasons about a fault from four minutes ago.
# ---------------------------------------------------------------------------
LOG_WINDOW_SEC = 30.0  # rolling /rosout window kept in memory
FAULT_CONTEXT_SEC = 5.0  # log slice sent to the model (±N sec around the fault)
MAX_LOG_LINES = 60  # hard cap, so one chatty node cannot fill the prompt
WORLD_STATE_FRAMES = 3  # /world_state frames kept for prompt context
ATTEMPT_HISTORY = 6  # local recovery attempts remembered per mission

# The closed set. Kept here as well as in the coordinator's schema.py because
# this node must refuse a bad answer *before* it goes on the wire -- an invalid
# action that reaches the coordinator is only ever going to be discarded there,
# and by then the fault has cost a model call and gone unanswered.
VALID_ACTIONS = ("re_delegate", "add_task", "drop_task")
VALID_DISPOSITIONS = ("drop", "hold", "request_human")

# Matches the coordinator's ReasonCode enum (amiga_ros2_comms/codec). Only the
# values a triage decision can justify are offered; the model picks a label and
# never a number.
REASON_CODES = {
    "unspecified": 0,
    "low_battery": 1,
    "unreachable": 2,
    "task_failed": 3,
    "capability_missing": 4,
    "operator": 5,
}

ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
LEVEL_MAP = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

# The tree publishes its faults TRANSIENT_LOCAL so an agent that starts late
# still sees the last one. Subscribing with anything else silently receives
# nothing from it.
LATCHED = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TriageNode(Node):
    """Decides what to do about a fault the robot could not fix itself."""

    def __init__(self):
        super().__init__("triage")
        self._lock = Lock()

        self.log_buffer: List[Dict] = []
        self.world_state_frames: deque = deque(maxlen=WORLD_STATE_FRAMES)
        self.attempts: deque = deque(maxlen=ATTEMPT_HISTORY)
        self.last_fault: Optional[Dict] = None
        self.mission_xml: Optional[str] = None
        self._last_status: Dict = {
            "model": llm.MODEL,
            "faults_seen": 0,
            "escalations": 0,
            "interpretations": 0,
            "refused": 0,
            "last_action": None,
            "last_error": None,
        }

        self.system_prompt = prompts.render(
            "triage/system.j2",
            valid_actions=VALID_ACTIONS,
            valid_dispositions=VALID_DISPOSITIONS,
            reason_codes=sorted(REASON_CODES),
        )

        # llm.complete() blocks for seconds. A reentrant group plus the
        # multithreaded executor in main() keeps the service call from stalling
        # the subscriptions that feed it context, and vice versa.
        group = ReentrantCallbackGroup()

        self.create_subscription(
            String, "/bt/status_change", self._on_fault, LATCHED, callback_group=group
        )
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.create_subscription(String, "/world_state", self._on_world_state, 10)
        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.create_subscription(String, "/mission/abort", self._on_abort, 10)
        self.create_subscription(
            String, "/mission/planner_status", self._on_planner_status, 10
        )

        # The escalation. Deterministic, and the coordinator's only entry point.
        self.infeasible_pub = self.create_publisher(
            String, "/coordination/infeasible", 10
        )

        self.create_service(
            InterpretAnomaly,
            "/coordination/interpret_anomaly",
            self._on_interpret,
            callback_group=group,
        )

        self.status = StatusPublisher(self)
        self.status.publish(self.get_status())

        self.get_logger().info(
            f"TriageNode started — model={llm.MODEL}, serving "
            f"/coordination/interpret_anomaly"
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_status(self) -> Dict:
        with self._lock:
            return dict(self._last_status)

    # ------------------------------------------------------------------
    # Context gathering
    # ------------------------------------------------------------------

    def _on_log(self, msg: Log):
        stamp = self._stamp_to_sec(msg.stamp)
        with self._lock:
            self.log_buffer.append(
                {
                    "stamp": stamp,
                    "level": LEVEL_MAP.get(msg.level, str(msg.level)),
                    "name": msg.name,
                    "msg": ANSI_ESCAPE.sub("", msg.msg),
                }
            )
            now = self._stamp_to_sec(self.get_clock().now().to_msg())
            self.log_buffer = [
                e for e in self.log_buffer if now - e["stamp"] < LOG_WINDOW_SEC
            ]

    def _on_world_state(self, msg: String):
        with self._lock:
            self.world_state_frames.append(msg.data)

    def _on_mission(self, msg: String):
        with self._lock:
            self.mission_xml = msg.data

    def _on_fault(self, msg: String):
        event = self._parse(msg.data, "/bt/status_change")
        if event is None:
            return
        with self._lock:
            self.last_fault = event
            self._last_status["faults_seen"] += 1
        self.get_logger().info(
            f"BT fault noted: node={event.get('node')!r} "
            f"source={event.get('source', '?')}"
        )
        self.status.publish(self.get_status())

    def _on_planner_status(self, msg: String):
        """The local loop reporting on itself. Give-ups are escalations."""
        info = self._parse(msg.data, "/mission/planner_status")
        if info is None:
            return
        with self._lock:
            self.attempts.append(info)
        event = str(info.get("event", ""))
        # The planner publishes this topic for terminal outcomes only, but not
        # every terminal outcome is a give-up -- match on the ones that are.
        if "gave_up" in event or "give_up" in event or "max_retries" in event:
            self._escalate(
                detail=f"local replanning gave up: {info.get('reason', event)}",
                cause="planner_gave_up",
            )

    def _on_abort(self, msg: String):
        """The arbiter has declared the mission non-viable. Hard escalation."""
        info = self._parse(msg.data, "/mission/abort") or {}
        with self._lock:
            self.attempts.append({"event": "aborted", **info})
        self._escalate(
            detail=f"arbiter aborted the mission: {info.get('reason', 'unknown')}",
            cause="arbiter_abort",
        )

    # ------------------------------------------------------------------
    # Escalation — deterministic
    # ------------------------------------------------------------------

    def _escalate(self, detail: str, cause: str) -> None:
        """Tell the coordinator this robot cannot recover on its own.

        No model call. The planner and the arbiter have already made the only
        judgement this step needs -- that local recovery is exhausted -- and
        re-deriving it here would add a second opinion nobody asked for.
        """
        with self._lock:
            fault = dict(self.last_fault) if self.last_fault else {}
            mission_xml = self.mission_xml

        task_id = self._task_id_for(fault, mission_xml)
        payload = {
            "cause": cause,
            "detail": detail,
            "task_id": task_id,
            "fault": fault,
            "at": time.time(),
        }
        self.infeasible_pub.publish(String(data=json.dumps(payload)))
        with self._lock:
            self._last_status["escalations"] += 1
        self.get_logger().warn(
            f"escalating to coordination: {detail} (task_id={task_id})"
        )
        self.status.publish(self.get_status())

    def _task_id_for(self, fault: Dict, mission_xml: Optional[str]) -> int:
        """Resolve the failing BT node to a coordinator task id.

        The bridge between the two worlds. A mission is BT XML whose leaves
        carry tree ids; a task on the radio is a 16-bit number. The failing
        node's name comes back on the fault event, so the id is a lookup in the
        plan that is actually running -- not a guess and not a second registry
        to keep in sync.

        0 means "no task attached", which the coordinator handles: it will
        still interpret the anomaly, it just has nothing to announce.
        """
        name = str(fault.get("node", ""))
        if not name or not mission_xml:
            return 0
        try:
            root = etree.fromstring(mission_xml.encode())
        except etree.XMLSyntaxError:
            return 0
        for element in root.iter():
            if element.get("name") != name:
                continue
            raw = element.get("id")
            if raw is None:
                continue
            try:
                value = int(raw)
            except ValueError:
                return 0
            # The wire carries a uint16. An id outside it is a plan this
            # fleet could not have announced in the first place.
            return value if 0 <= value <= 0xFFFF else 0
        return 0

    # ------------------------------------------------------------------
    # Interpretation — the model
    # ------------------------------------------------------------------

    def _on_interpret(self, request, response):
        """Answer one anomaly with one typed action.

        Never raises: a service that throws leaves the coordinator with a
        failed future, which it treats as "no interpretation available" and so
        leaves the task exactly as it was. That is the right outcome, but a
        populated `error` says why, and a bare exception does not.
        """
        with self._lock:
            self._last_status["interpretations"] += 1

        try:
            fault, logs, world, attempts = self._assemble(request)
            user_prompt = prompts.render(
                "triage/user.j2",
                fault=json.dumps(fault, indent=2) if fault else "(none reported)",
                log_context=logs or "(no log lines in the window)",
                world_state=world or "(no world-state frame)",
                local_attempts=attempts or "(none)",
                task_id=int(request.task_id),
                required_capability=int(request.required_capability),
                grid_row=int(request.grid_row),
                grid_col=int(request.grid_col),
                priority=int(request.priority),
                battery_percent=int(request.battery_percent),
                peers=request.peers_json or "[]",
            )
            reply = llm.complete(self.system_prompt, user_prompt)
            decision = self._parse_decision(reply)
        except Exception as exc:  # noqa: BLE001 - a model, a parser, a timeout
            self.get_logger().error(f"interpretation failed: {exc}")
            with self._lock:
                self._last_status["refused"] += 1
                self._last_status["last_error"] = str(exc)
            response.ok = False
            response.error = str(exc)
            response.action = ""
            self.status.publish(self.get_status())
            return response

        response.ok = True
        response.error = ""
        response.action = decision["action"]
        response.reason_code = decision["reason_code"]
        response.fallback = decision["fallback"]
        response.disposition = decision["disposition"]
        # For re_delegate and drop_task these fields just echo the task the
        # caller asked about, so the response reads as self-contained. For
        # add_task they must NOT: the whole point of add_task is *different*
        # work, and substituting the failed task's id here would turn "I found
        # something else to do" into "re-add the thing that just failed", with
        # nothing downstream able to tell the difference. Leaving them at what
        # the model actually said means an add_task with no task is a 0, and
        # the coordinator's client refuses it.
        echo = decision["action"] != "add_task"
        response.task_id = decision["task_id"] or (int(request.task_id) if echo else 0)
        response.required_capability = decision["required_capability"] or (
            int(request.required_capability) if echo else 0
        )
        response.grid_row = decision["grid_row"] or (
            int(request.grid_row) if echo else 0
        )
        response.grid_col = decision["grid_col"] or (
            int(request.grid_col) if echo else 0
        )
        response.priority = decision["priority"] or (
            int(request.priority) if echo else 0
        )
        response.rationale = decision["rationale"]
        response.model = llm.MODEL

        with self._lock:
            self._last_status["last_action"] = decision["action"]
            self._last_status["last_error"] = None
        self.get_logger().info(
            f"interpretation: {decision['action']} — {decision['rationale'][:160]}"
        )
        self.status.publish(self.get_status())
        return response

    def _assemble(self, request):
        """Prompt context: what the caller sent, else what we have been watching.

        The coordinator knows about tasks, peers and batteries and nothing about
        behaviour trees or /rosout. Rather than make it carry evidence it cannot
        interpret, this node gathers that half itself and the request fields are
        overrides -- which is also what lets the service be driven by hand.
        """
        with self._lock:
            fault = (
                self._parse(request.fault_json, "request.fault_json")
                if request.fault_json
                else (dict(self.last_fault) if self.last_fault else None)
            )
            world = request.world_state or (
                self.world_state_frames[-1] if self.world_state_frames else ""
            )
            attempts = request.local_attempts or (
                json.dumps(list(self.attempts), indent=2) if self.attempts else ""
            )
            logs = request.log_context or self._log_slice(fault)
        return fault, logs, world, attempts

    def _log_slice(self, fault: Optional[Dict]) -> str:
        """The /rosout lines around the fault, newest last.

        Centred on the fault rather than on now, because the pipeline that got
        here -- planner retries, an arbiter call or two -- can take tens of
        seconds, by which time the lines that explain the fault have aged out
        of any window anchored to the present.
        """
        if fault is None:
            centre = self._stamp_to_sec(self.get_clock().now().to_msg())
        else:
            centre = float(fault.get("timestamp_ms", 0)) / 1000.0
        window = [
            entry
            for entry in self.log_buffer
            if centre - FAULT_CONTEXT_SEC
            <= entry["stamp"]
            <= centre + FAULT_CONTEXT_SEC
        ]
        # Errors and warnings first when the window has to be truncated: an
        # arbitrary tail of a debug flood explains nothing.
        if len(window) > MAX_LOG_LINES:
            severe = [e for e in window if e["level"] in ("ERROR", "FATAL", "WARN")]
            window = (severe + window)[:MAX_LOG_LINES]
            window.sort(key=lambda e: e["stamp"])
        return "\n".join(
            f"[{entry['level']}] {entry['name']}: {entry['msg']}" for entry in window
        )

    def _parse_decision(self, reply: str) -> Dict:
        """Turn the model's reply into a validated decision, or raise.

        Everything about this is deliberately unforgiving. A half-understood
        answer is the failure mode worth being loud about: an unrecognised
        action silently coerced to a default is a robot that drops a task
        because a model mentioned the word.
        """
        text = llm.strip_code_fence(reply)
        # Models like to wrap the object in a sentence. Take the outermost
        # braces and parse that; anything else is a parse failure, not a hint.
        start, end = text.find("{"), text.rfind("}")
        if start == -1 or end <= start:
            raise ValueError(f"no JSON object in the reply: {text[:200]!r}")
        # A reply shaped as a list of actions is ambiguous -- which one? Taking
        # the first would be exactly the half-succeeded parse this guard exists
        # to prevent, and the robot would act on a decision nobody made.
        bracket = text.find("[")
        if bracket != -1 and bracket < start:
            raise ValueError("expected one action object, got a list of them")
        decision = json.loads(text[start : end + 1])
        if not isinstance(decision, dict):
            raise ValueError(f"expected a JSON object, got {type(decision).__name__}")

        action = str(decision.get("action", "")).strip().lower()
        if action not in VALID_ACTIONS:
            raise ValueError(
                f"action {action!r} is not one of {', '.join(VALID_ACTIONS)}"
            )

        fallback = str(decision.get("fallback", "hold")).strip().lower()
        disposition = str(decision.get("disposition", "drop")).strip().lower()
        if action == "re_delegate" and fallback not in VALID_DISPOSITIONS:
            raise ValueError(f"fallback {fallback!r} is not a valid disposition")
        if action == "drop_task" and disposition not in VALID_DISPOSITIONS:
            raise ValueError(f"disposition {disposition!r} is not a valid disposition")

        reason = str(decision.get("reason_code", "unspecified")).strip().lower()
        if reason not in REASON_CODES:
            raise ValueError(
                f"reason_code {reason!r} is not one of {sorted(REASON_CODES)}"
            )

        return {
            "action": action,
            "reason_code": REASON_CODES[reason],
            "fallback": fallback if action == "re_delegate" else "",
            "disposition": disposition if action == "drop_task" else "",
            "task_id": self._bounded(decision.get("task_id"), 0xFFFF),
            "required_capability": self._bounded(decision.get("capability"), 0xFFFF),
            "grid_row": self._bounded(decision.get("grid_row"), 0xFFFF),
            "grid_col": self._bounded(decision.get("grid_col"), 0xFFFF),
            "priority": self._bounded(decision.get("priority"), 0xFF),
            "rationale": str(decision.get("rationale", "")).strip(),
        }

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _bounded(value, maximum: int) -> int:
        """An out-of-range or non-numeric field reads as absent, not as an error.

        These are only ever used for add_task, and the caller substitutes the
        request's own values when they come back 0. A model that omits them for
        a re_delegate -- which is most of the time -- must not fail the parse.
        """
        try:
            number = int(value)
        except (TypeError, ValueError):
            return 0
        return number if 0 <= number <= maximum else 0

    def _parse(self, payload: str, source: str) -> Optional[Dict]:
        try:
            parsed = json.loads(payload)
        except json.JSONDecodeError:
            self.get_logger().error(f"could not parse {source} payload")
            return None
        if not isinstance(parsed, dict):
            self.get_logger().error(
                f"{source} payload must be a JSON object, got {type(parsed).__name__}"
            )
            return None
        return parsed

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


def main():
    spin.run(TriageNode, multithreaded=True)


if __name__ == "__main__":
    main()
