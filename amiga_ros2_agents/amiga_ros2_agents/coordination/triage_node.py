"""
triage_node.py

The first of the two reasoning points: `interpret_anomaly`.

The behaviour tree is the catalyst. A node fails, `/bt/status_change` fires, and
something has to decide what happens to that work. This node decides, and it
decides *first* -- before the plan is edited, not after the editing has run out.

Three jobs, and one body of evidence all three read from.

**Routing is a judgement, and it comes first.** "Can this robot plan its way
out of this?" used to be answered by a counter: the mission planner replanned
until MAX_RETRIES ran out, and exhausting the budget was taken to mean the
robot was incapable. It does not mean that. A dead depth camera is beyond any
edit, and three sessions spent proving it are three sessions during which the
plan grows retry wrappers around a sensor that is never coming back -- and the
node the escalation would have named has been edited out from under it. So the
first FAILURE on a leaf gets one model call, over the fault and the /rosout
lines behind it, and the answer is `repair` or `escalate` on
/mission/fault_route. The planner waits for it. One call per failing node per
mission, cached, so repeated ticks of the same dead leaf are free.

**Escalation is still deterministic.** Whether the route said so, or the
planner published a give-up on /mission/planner_status, or the arbiter aborted
on /mission/abort -- all three mean local recovery is over, and this node
forwards them to the coordinator on /coordination/infeasible unchanged. The
give-up paths remain as backstops: routing says "don't bother", the budget says
"we bothered and it didn't work", and both need to reach the fleet.

**The decision about the shed work is not.** "What should the fleet do with
it?" is a judgement over unstructured evidence -- the fault, the logs, where the
robot is, what was tried, who else is out there. That is the service this node
serves, and the answer is constrained to three typed actions (re_delegate,
add_task, drop_task) by InterpretAnomaly.srv, which the coordinator refuses
anything outside of. The model chooses *among decisions the state machine
already knows how to execute*, so every path below it stays deterministic and
testable with this node swapped for a stub.

**The evidence is latched when the fault arrives, not when a prompt is built.**
`_capture` takes the log slice, the world-state frame current at that instant
and one call to the camera, and both decisions read that snapshot. The logs were
always fault-centred; the other two were not, and the interpretation call
arrives minutes later -- once local recovery has run out -- so a world frame or
a picture from *then* described a different moment than the fault they were
offered as evidence of. Latching also means one camera call per fault rather
than one per decision.

The camera only ever describes. Whether anything is in the way, whether a path
is passable, what the robot should do: all of that is this node's model to
decide, from the description plus everything else it holds. Off unless
`use_vlm` is set, and absent, slow or broken it costs a couple of seconds and
the verdict comes out as it always did.

    /bt/status_change ─┐              ┌──▶ /mission/fault_route (topic)
    /rosout ───────────┤              │
    /world_state ──────┼──▶ [evidence]├──▶ /coordination/interpret_anomaly (srv)
    /mission/xml ──────┤       ▲      │
    /mission/abort ────┤       │      └──▶ /coordination/infeasible (topic)
    /mission/planner_status ───┤
                               └── /vlm/ask (srv) — what the camera sees

Prompts live in prompts/triage/.
"""

import json
import re
import time
from collections import deque
from threading import Lock, Thread
from typing import Dict, Optional

from amiga_interfaces.srv import InterpretAnomaly
from amiga_ros2_comms.codec import (
    CAPABILITY_BY_ELEMENT,
    PRIORITY_MAX,
    TASK_ID_MAX,
    ReasonCode,
    Target,
    TargetKind,
)
from amiga_ros2_comms.reliability.notes import (
    MAX_NOTE_FRAGMENTS,
    text_bytes_per_fragment,
)
from rcl_interfaces.msg import Log
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ..mission import mission_tasks
from ..mission.mission_tasks import MissionTask
from ..runtime import llm, prompts, spin
from ..runtime.status import StatusPublisher
from . import vlm_client

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

# The routing verdict. Two values because there are exactly two things that can
# happen to a fault: this robot's planner has another go, or the fleet is told.
VALID_ROUTES = ("repair", "escalate")

#: What the routing prompt calls the camera when nothing says otherwise. The
#: front Oak-D, which is what every demo but the blinded-detector one shows.
DEFAULT_CAMERA_DESCRIPTION = "the front camera"

#: Where the verdict goes. The planner blocks on this before it opens a
#: planning session, so the name is part of the contract between the two nodes.
ROUTE_TOPIC = "/mission/fault_route"

#: A hint handed to the planner with a `repair` verdict, capped so one verbose
#: reply cannot crowd the mission XML out of the planner's prompt. Cut rather
#: than refused, unlike a note: this text never leaves the robot, so a clipped
#: sentence costs a little context and not a fleet-wide misunderstanding.
MAX_GUIDANCE_CHARS = 500

# What a note may weigh, computed from the radio rather than written down again.
# 328 bytes at the 50-byte payload budget: eight fragments, and every one of
# them is a packet the fleet pays for whether or not the note changes any bid.
NOTE_MAX_BYTES = MAX_NOTE_FRAGMENTS * text_bytes_per_fragment()

# Built from the codec's ReasonCode rather than restated. The previous version
# of this was a second hand-written table that claimed to match and did not:
# every non-zero label travelled as the wrong code, so "low battery" arrived at
# the fleet as "an operator asked for this". Deriving it means the two cannot
# disagree again -- the model picks a label, and the label is the enum's own
# name.
REASON_CODES = {code.name.lower(): int(code) for code in ReasonCode}

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

        #: Whether to ask the camera what it sees before deciding. Off by
        #: default: it needs a VLM endpoint and the amiga_vlm stack built, and
        #: neither is required for anything else this node does.
        self.declare_parameter("use_vlm", False)
        self.use_vlm = bool(
            self.get_parameter("use_vlm").get_parameter_value().bool_value
        )
        self.declare_parameter("vlm_service", vlm_client.DEFAULT_SERVICE)
        self.declare_parameter("vlm_timeout_sec", vlm_client.DEFAULT_TIMEOUT_SEC)
        self.declare_parameter("describe_frame", False)
        self.describe_frame = bool(
            self.get_parameter("describe_frame").get_parameter_value().bool_value
        )
        self.declare_parameter("camera_description", DEFAULT_CAMERA_DESCRIPTION)
        self.camera_description = str(
            self.get_parameter("camera_description").get_parameter_value().string_value
            or DEFAULT_CAMERA_DESCRIPTION
        )

        self.log_buffer: deque = deque()
        self.world_state_frames: deque = deque(maxlen=WORLD_STATE_FRAMES)
        self.attempts: deque = deque(maxlen=ATTEMPT_HISTORY)
        self.last_fault: Optional[Dict] = None
        #: Evidence latched at the last fault -- logs, world frame, camera. The
        #: interpretation path reads this rather than re-pulling, because by
        #: then the fault is minutes old.
        self.last_evidence: Optional[Dict] = None
        self.mission_xml: Optional[str] = None
        #: Routing verdicts, keyed by failing node name. One model call per
        #: node per mission: a permanently dead leaf re-ticks several times
        #: under the plan's own retry decorators, and asking again each time
        #: would spend a call to re-learn what the first one established.
        self.routes: Dict[str, Dict] = {}
        self._routing: set = set()
        self._last_status: Dict = {
            "model": llm.MODEL,
            "faults_seen": 0,
            "routes": 0,
            "escalations": 0,
            "interpretations": 0,
            "refused": 0,
            #: Frames described, not calls attempted. 0 while `vlm` reads
            #: true is the difference between "switched off" and "switched on
            #: and not answering"; the verdict looks the same either way.
            "vlm_looks": 0,
            "last_route": None,
            "last_action": None,
            "last_error": None,
        }

        # A model call blocks for seconds. A reentrant group plus the
        # multithreaded executor in main() keeps the service call from stalling
        # the subscriptions that feed it context, and vice versa.
        group = ReentrantCallbackGroup()

        #: The camera, if this robot has one. Same reentrant group as the two
        #: blocking callbacks, so the executor can deliver the response while
        #: one of them waits.
        self.vlm = (
            vlm_client.VlmClient(
                self,
                service_name=self.get_parameter("vlm_service").value,
                timeout_sec=float(self.get_parameter("vlm_timeout_sec").value),
                callback_group=group,
            )
            if self.use_vlm
            else None
        )
        self._last_status["vlm"] = self.vlm is not None

        self.route_system_prompt = prompts.render(
            "triage/route_system.j2",
            valid_routes=VALID_ROUTES,
            reason_codes=sorted(REASON_CODES),
        )
        self.system_prompt = prompts.render(
            "triage/system.j2",
            valid_actions=VALID_ACTIONS,
            valid_dispositions=VALID_DISPOSITIONS,
            reason_codes=sorted(REASON_CODES),
            # The behaviour-tree actions the model may name, taken from the
            # capability vocabulary rather than written into the prompt. A
            # hand-written list would be a third place the schema is restated,
            # and the one nobody would think to update.
            actions=sorted(CAPABILITY_BY_ELEMENT),
            # From the codec, so the prompt cannot disagree with the parser
            # about what fits -- and so lowering the payload budget tightens
            # what the model is asked for, not just what it is refused.
            note_max_bytes=NOTE_MAX_BYTES,
        )

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

        # The routing verdict. Not latched: a stale verdict replayed to a
        # planner that restarted mid-mission would stand down a session for a
        # fault nobody is looking at any more.
        self.route_pub = self.create_publisher(String, ROUTE_TOPIC, 10)

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
            f"TriageNode started — model={llm.MODEL}, "
            f"vlm={self.vlm.service_name if self.vlm else 'off'}, serving "
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
        """One /rosout line into the rolling window.

        A deque pruned from the left, not a list rebuilt on every line. This
        runs at whatever rate the whole stack logs -- Nav2 alone can produce
        tens of lines a second under load -- and it holds the lock that a
        diagnosis needs to read its evidence. Rebuilding the buffer per line
        made that lock's hold time grow with the buffer.
        """
        entry = {
            "stamp": self._stamp_to_sec(msg.stamp),
            "level": LEVEL_MAP.get(msg.level, str(msg.level)),
            "name": msg.name,
            "msg": ANSI_ESCAPE.sub("", msg.msg),
        }
        now = self._stamp_to_sec(self.get_clock().now().to_msg())
        with self._lock:
            self.log_buffer.append(entry)
            while (
                self.log_buffer and now - self.log_buffer[0]["stamp"] >= LOG_WINDOW_SEC
            ):
                self.log_buffer.popleft()

    # ------------------------------------------------------------------
    # The camera
    # ------------------------------------------------------------------

    def _look(self) -> str:
        """What the camera sees, for the prompt. Never raises.

        Fetched on every fault rather than when the model asks for it: the
        decision is made from all the evidence at once, and a model that has
        to decide whether it wants a piece of evidence is making a judgement
        before it has the thing it would judge with.

        Returns "" when there is no camera, which the prompts render as no
        section at all.
        """
        if self.vlm is None:
            return ""

        try:
            answer = self.vlm.ask(vlm_client.describe_question(self.describe_frame))
        except Exception as exc:  # noqa: BLE001 - a service, a socket, a model
            self.get_logger().warn(f"the camera failed ({exc})")
            return "(the camera did not answer — decide from the rest)"
        if not answer:
            self.get_logger().warn("the camera did not answer")
            return "(the camera did not answer — decide from the rest)"

        with self._lock:
            self._last_status["vlm_looks"] += 1
        self.get_logger().info(f"camera: {answer}")
        # Capped here as well as in the client, the same way a note is refused
        # at both ends: the client bounds what the service may return, this
        # bounds what the prompt spends.
        return answer[: vlm_client.MAX_ANSWER_CHARS]

    def _on_world_state(self, msg: String):
        with self._lock:
            self.world_state_frames.append(msg.data)

    def _on_mission(self, msg: String):
        with self._lock:
            self.mission_xml = msg.data

    def _on_fault(self, msg: String):
        """A leaf failed. Note it, then decide where it goes.

        Two filters before anything else, and both were bugs.

        ``/bt/status_change`` reports a leaf reaching SUCCESS as well as
        FAILURE. Recording those as ``last_fault`` meant the escalation named
        whichever leaf happened to succeed last -- typically the *approach*
        that worked, not the sample that did not.

        The tree also reports its own final outcome as ``<tree>``, and that
        arrives after the leaf fault that caused it. It is not a unit of work:
        ``task_for_node`` finds nothing for it, so letting it overwrite
        ``last_fault`` is how an escalation goes out with ``task_id: 0`` and
        the coordinator has nothing to announce.
        """
        event = self._parse(msg.data, "/bt/status_change")
        if event is None:
            return
        # No "status" key is the older hand-written shape, which only ever
        # meant FAILURE.
        status = str(event.get("status") or "FAILURE").upper()
        if status != "FAILURE":
            return
        name = str(event.get("node") or "")
        whole_tree = name in ("", "<tree>") or event.get("source") == "tree"

        with self._lock:
            self._last_status["faults_seen"] += 1
            if not whole_tree:
                self.last_fault = event
            cached = self.routes.get(name)
            start = not whole_tree and cached is None and name not in self._routing
            if start:
                self._routing.add(name)

        self.get_logger().info(
            f"BT fault noted: node={name!r} source={event.get('source', '?')}"
        )
        self.status.publish(self.get_status())

        if whole_tree:
            return
        if cached is not None:
            # Same leaf failing again under its retry decorator. Re-publish the
            # verdict rather than re-deriving it: the planner may have come up
            # since, and this costs a message instead of a model call.
            self._publish_route(cached)
            return
        if start:
            Thread(target=self._route_fault, args=(event,), daemon=True).start()

    # ------------------------------------------------------------------
    # Routing — the model, before the plan is touched
    # ------------------------------------------------------------------

    def _route_fault(self, event: Dict) -> None:
        """Decide whether this fault is worth replanning for. One model call.

        Runs before the mission planner opens a session, which is the whole
        point: the planner blocks on ``ROUTE_TOPIC``, so until this publishes,
        ``/mission/xml`` still holds the plan that was running when the leaf
        failed. That is what makes an ``escalate`` verdict announceable -- the
        failing node is still in the plan to be resolved against.

        Fails open. A model that is down, slow or incoherent must not stop a
        robot recovering from a fault it could have planned around, so any
        error here becomes ``repair`` with the error as the rationale -- which
        is exactly the behaviour this node replaced.
        """
        name = str(event.get("node") or "")
        evidence = self._capture(event)
        try:
            decision = self._decide_route(evidence)
        except Exception as exc:  # noqa: BLE001 - a model, a parser, a timeout
            self.get_logger().warn(
                f"routing {name!r} failed ({exc}) — defaulting to local repair"
            )
            decision = {
                "route": "repair",
                "reason_code": REASON_CODES["unspecified"],
                "rationale": f"routing failed ({exc}); defaulting to local repair",
                "guidance": "",
            }

        payload = {
            "node": name,
            "timestamp_ms": event.get("timestamp_ms", 0),
            "at": time.time(),
            **decision,
        }
        with self._lock:
            self.routes[name] = payload
            self._routing.discard(name)
            self._last_status["routes"] += 1
            self._last_status["last_route"] = decision["route"]

        # Published before the escalation, deliberately. The planner has to
        # stand down before the coordinator starts committing edits to the same
        # plan, or the two of them are writing over each other.
        self._publish_route(payload)
        self.get_logger().info(
            f"routed {name!r}: {decision['route']} — {decision['rationale'][:160]}"
        )
        self.status.publish(self.get_status())

        if decision["route"] == "escalate":
            self._escalate(
                detail=f"triage routed the fault to the fleet: {decision['rationale']}",
                cause="triage_route",
            )

    def _publish_route(self, payload: Dict) -> None:
        self.route_pub.publish(String(data=json.dumps(payload)))

    def _capture(self, event: Dict) -> Dict:
        """Everything the decisions will reason from, taken at the fault.

        The logs were already fault-centred; the world frame and the camera
        were not -- both were read whenever a prompt happened to be built. For
        routing that was milliseconds later and harmless. For interpretation it
        is minutes: the coordinator only asks once local recovery has run out,
        and by then "the latest world frame" and "what the camera sees" belong
        to a different moment than the fault they are offered as evidence of.

        Captured once, here, and reused by both decisions. That also means one
        camera call per fault rather than one per decision.
        """
        with self._lock:
            logs = self._log_slice(event)
            world = self.world_state_frames[-1] if self.world_state_frames else ""
            mission_xml = self.mission_xml
        # Off the lock: this blocks on a service for up to vlm_timeout_sec, and
        # _on_log takes the same lock on every line that arrives meanwhile.
        evidence = {
            "fault": event,
            "logs": logs,
            "world": world,
            "mission_xml": mission_xml,
            "visual": self._look(),
            "at": time.time(),
        }
        with self._lock:
            self.last_evidence = evidence
        return evidence

    def _decide_route(self, evidence: Dict) -> Dict:
        event = evidence["fault"]
        mission_xml = evidence["mission_xml"]
        with self._lock:
            attempts = (
                json.dumps(list(self.attempts), indent=2) if self.attempts else ""
            )
        task = self._task_for(event, mission_xml)
        prompt = prompts.render(
            "triage/route_user.j2",
            fault=json.dumps(event, indent=2),
            log_context=evidence["logs"] or "(no log lines in the window)",
            visual_context=evidence["visual"],
            camera_description=self.camera_description,
            world_state=evidence["world"] or "(no world-state frame)",
            local_attempts=attempts or "(none)",
            mission_xml=mission_xml or "(no plan received yet)",
            # What the fleet would be offered if this escalates. Naming it here
            # is the difference between "should we give up on this leaf" and
            # "should we give this unit of work to somebody else".
            task=(
                f"{task.name!r} — "
                f"{', '.join(mission_tasks.capability_names(task.capabilities))} "
                f"at {task.target}"
                if task is not None
                else "(this fault does not resolve to a unit of work the fleet "
                "could be offered)"
            ),
        )
        return self._parse_route(llm.complete(self.route_system_prompt, prompt))

    def _parse_route(self, reply: str) -> Dict:
        """Turn the routing reply into a verdict, or raise.

        Same unforgiving shape as ``_parse_decision`` and for the same reason,
        with one difference: the caller catches. A refused verdict here means
        local repair, not a stalled fault.
        """
        text = llm.strip_code_fence(reply)
        start, end = text.find("{"), text.rfind("}")
        if start == -1 or end <= start:
            raise ValueError(f"no JSON object in the reply: {text[:200]!r}")
        verdict = json.loads(text[start : end + 1])
        if not isinstance(verdict, dict):
            raise ValueError(f"expected a JSON object, got {type(verdict).__name__}")

        route = str(verdict.get("route", "")).strip().lower()
        if route not in VALID_ROUTES:
            raise ValueError(f"route {route!r} is not one of {', '.join(VALID_ROUTES)}")
        reason = str(verdict.get("reason_code", "unspecified")).strip().lower()
        if reason not in REASON_CODES:
            raise ValueError(
                f"reason_code {reason!r} is not one of {sorted(REASON_CODES)}"
            )
        return {
            "route": route,
            "reason_code": REASON_CODES[reason],
            "rationale": str(verdict.get("rationale", "")).strip(),
            # Only meaningful for repair -- there is no local planner to advise
            # once the work has left the robot.
            "guidance": (
                str(verdict.get("guidance", "")).strip()[:MAX_GUIDANCE_CHARS]
                if route == "repair"
                else ""
            ),
        }

    def _on_planner_status(self, msg: String):
        """The local loop reporting on itself. Give-ups are escalations."""
        info = self._parse(msg.data, "/mission/planner_status")
        if info is None:
            return
        with self._lock:
            self.attempts.append(info)
        # The planner names its terminal state in `outcome` and which budget ran
        # out in `cause`. `event` is what a hand-written status uses -- the
        # escalate CLI, a scenario script, a test poking this by hand -- and it
        # is read too rather than picked between: this is the single hinge
        # between local recovery and the fleet, and it failing closed because
        # the producer named a key differently is the most expensive way for it
        # to be wrong. Both sides are pinned by tests now.
        outcome = " ".join(
            str(info.get(key, "")) for key in ("outcome", "event", "cause")
        )
        # The planner publishes this topic for terminal outcomes only, but not
        # every terminal outcome is a give-up -- match on the ones that are.
        if not any(word in outcome for word in ("gave_up", "give_up", "max_retries")):
            return
        # `last_reason` is the arbiter's own words for why the final candidate
        # was refused, which is the most useful sentence available here; the
        # others are fallbacks for a payload that did not carry one.
        reason = info.get("last_reason") or info.get("reason") or outcome.strip()
        self._escalate(
            detail=f"local replanning gave up: {reason}",
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

        No model call of its own -- the judgement has already been made by
        whichever of the three callers got here: routing decided the fault was
        not worth planning around, the planner ran out of budget, or the
        arbiter declared the mission non-viable. This step only carries it.

        The routing caller is the one that can name the work. It runs before
        any edit has been committed, so ``self.mission_xml`` is still the plan
        that was executing when the leaf failed and ``_task_for`` resolves
        against it. The two give-up callers arrive after the planner has been
        rewriting that plan for a minute or more, which is when the failing
        node has been edited away and this falls back to ``task_id: 0``.
        """
        with self._lock:
            fault = dict(self.last_fault) if self.last_fault else {}
            mission_xml = self.mission_xml

        task = self._task_for(fault, mission_xml)
        payload = {
            "cause": cause,
            "detail": detail,
            "fault": fault,
            "at": time.time(),
        }
        if task is not None:
            # The whole descriptor, not just the id. This node is the only
            # thing in the system that reads the mission XML, so it is the only
            # thing that can say what the failed work actually *is* -- which
            # actions it needs and where it happens. Sending only an id would
            # leave the coordinator to invent both, and it would announce a
            # task at the origin needing a capability nobody chose.
            payload.update(task.as_payload())
        else:
            payload["task_id"] = 0

        self.infeasible_pub.publish(String(data=json.dumps(payload)))
        with self._lock:
            self._last_status["escalations"] += 1
        if task is None:
            self.get_logger().warn(
                f"escalating to coordination: {detail} (no task resolved from "
                f"the running plan; it can be interpreted but not announced)"
            )
        else:
            self.get_logger().warn(
                f"escalating to coordination: {detail} "
                f"(task {task.task_id} {task.name!r}, "
                f"{mission_tasks.capability_names(task.capabilities)} "
                f"at {task.target})"
            )
        self.status.publish(self.get_status())

    def _task_for(self, fault: Dict, mission_xml) -> Optional[MissionTask]:
        """Resolve the failing BT node to the unit of work it belongs to.

        The bridge between the two worlds, and it resolves to a *subtree*
        rather than a leaf. ``/bt/status_change`` names the leaf that failed;
        what the fleet can be offered is the whole unit that leaf belongs to --
        approaching tree 60 and sampling it, not the sample on its own, which
        would arrive somewhere else with nothing to sample.

        None means the anomaly has no task attached, which the coordinator
        handles: it will still interpret it, it just has nothing to announce.
        """
        name = str(fault.get("node", ""))
        if not name:
            return None
        return mission_tasks.task_for_node(mission_xml, name)

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
            fault, logs, world, attempts, visual = self._assemble(request)
            user_prompt = prompts.render(
                "triage/user.j2",
                fault=json.dumps(fault, indent=2) if fault else "(none reported)",
                log_context=logs or "(no log lines in the window)",
                visual_context=visual,
                world_state=world or "(no world-state frame)",
                local_attempts=attempts or "(none)",
                task_id=int(request.task_id),
                # Element names rather than a mask, so the model reads the same
                # words as the mission XML it is reasoning about. A number here
                # would be a number it has to be told how to decode, in a
                # prompt, every time.
                required_actions=", ".join(
                    mission_tasks.capability_names(int(request.required_capabilities))
                )
                or "(none stated)",
                where=self._where(request),
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
        response.required_capabilities = decision["capabilities"] or (
            int(request.required_capabilities) if echo else 0
        )
        target = decision["target"]
        if target is None and echo:
            response.target_kind = int(request.target_kind)
            response.target_a = int(request.target_a)
            response.target_b = int(request.target_b)
        else:
            target = target or Target.none()
            response.target_kind = int(target.kind)
            response.target_a = int(target.a)
            response.target_b = int(target.b)
        response.priority = decision["priority"] or (
            int(request.priority) if echo else 0
        )
        response.rationale = decision["rationale"]
        response.note = decision["note"]
        response.model = llm.MODEL

        with self._lock:
            self._last_status["last_action"] = decision["action"]
            self._last_status["last_error"] = None
        self.get_logger().info(
            f"interpretation: {decision['action']} — {decision['rationale'][:160]}"
        )
        self.status.publish(self.get_status())
        return response

    @staticmethod
    def _where(request) -> str:
        """The task's place, in words. ``Target.__str__`` already reads well."""
        try:
            return str(
                Target(
                    kind=TargetKind(int(request.target_kind)),
                    a=int(request.target_a),
                    b=int(request.target_b),
                )
            )
        except (ValueError, TypeError):
            return "(not stated)"

    def _assemble(self, request):
        """Prompt context: what the caller sent, else the latched evidence.

        The coordinator knows about tasks, peers and batteries and nothing about
        behaviour trees or /rosout. Rather than make it carry evidence it cannot
        interpret, this node gathers that half itself and the request fields are
        overrides -- which is also what lets the service be driven by hand.

        The evidence comes from `_capture`, taken when the leaf failed, not from
        the buffers as they stand now. This call arrives once local recovery has
        run out -- minutes after the fault -- and a world frame or a camera
        image from *now* describes a different moment than the fault it is
        offered as evidence of. Only if there is no latched evidence (a service
        driven by hand, a fault this node never saw) does it fall back to
        reading the buffers.
        """
        with self._lock:
            latched = dict(self.last_evidence) if self.last_evidence else None
            fault = (
                self._parse(request.fault_json, "request.fault_json")
                if request.fault_json
                else (dict(self.last_fault) if self.last_fault else None)
            )
            attempts = request.local_attempts or (
                json.dumps(list(self.attempts), indent=2) if self.attempts else ""
            )
            if latched is None:
                world = request.world_state or (
                    self.world_state_frames[-1] if self.world_state_frames else ""
                )
                logs = request.log_context or self._log_slice(fault)
                visual = ""
            else:
                world = request.world_state or latched["world"]
                logs = request.log_context or latched["logs"]
                visual = latched["visual"]
        if latched is None:
            # No fault was ever captured, so nothing has looked yet.
            visual = self._look()
        return fault, logs, world, attempts, visual

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
            "task_id": self._bounded(decision.get("task_id"), TASK_ID_MAX),
            "capabilities": self._capabilities(decision.get("actions")),
            "target": self._target(decision.get("target")),
            "priority": self._bounded(decision.get("priority"), PRIORITY_MAX),
            "rationale": str(decision.get("rationale", "")).strip(),
            "note": self._note(decision.get("note"), action),
        }

    @staticmethod
    def _note(note, action: str) -> str:
        """The free text that rides with an announcement. Empty unless shed.

        Refused rather than truncated when it is too long for the radio, for
        the same reason ``split_note`` refuses: where to cut a sentence is a
        decision with meaning, and neither the codec nor this parser knows what
        the sentence was for. Refusing sends the whole interpretation back as
        failed, and the coordinator then leaves the task alone -- which is
        louder than an announcement that quietly lost half its context.

        A note on anything but re_delegate is dropped without comment: there is
        no announcement for it to ride on, so it would be text addressed to
        nobody.
        """
        text = str(note or "").strip()
        if not text or action != "re_delegate":
            return ""
        size = len(text.encode("utf-8"))
        if size > NOTE_MAX_BYTES:
            raise ValueError(
                f"note is {size} UTF-8 bytes; the radio carries at most "
                f"{NOTE_MAX_BYTES}. Say less, or say nothing."
            )
        return text

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _capabilities(actions) -> int:
        """A list of behaviour-tree action names, as a capability mask.

        An unrecognised name is refused rather than dropped. Silently ignoring
        it would build a mask describing *less* work than the model asked for,
        which is how a task gets announced as feasible for robots that cannot
        do it -- and the announcement would look perfectly ordinary.
        """
        if actions is None:
            return 0
        if isinstance(actions, str):
            actions = [actions]
        if not isinstance(actions, list):
            raise ValueError(
                f"'actions' must be a list of action names, got {actions!r}"
            )
        mask = 0
        for name in actions:
            capability = CAPABILITY_BY_ELEMENT.get(str(name).strip())
            if capability is None:
                raise ValueError(
                    f"action {name!r} is not in the mission schema; known: "
                    f"{', '.join(sorted(CAPABILITY_BY_ELEMENT))}"
                )
            mask |= 1 << int(capability)
        return mask

    @staticmethod
    def _target(target) -> Optional[Target]:
        """The place an add_task names, or None when it named none.

        Refused rather than defaulted, for the same reason as the actions: a
        target quietly coerced to "here" is work announced at a place every
        listener resolves differently.
        """
        if target is None:
            return None
        if not isinstance(target, dict):
            raise ValueError(f"'target' must be an object, got {type(target).__name__}")
        kind = str(target.get("kind", "")).strip().lower()
        try:
            if kind in ("", "none"):
                return Target.none()
            if kind == "tree":
                return Target.tree(int(target["index"]))
            if kind == "aisle":
                return Target.aisle(int(target["index"]))
            if kind == "gps":
                return Target.gps(float(target["latitude"]), float(target["longitude"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"target {target!r} is not a place: {exc}") from None
        raise ValueError(f"target kind {kind!r} is not one of: none, tree, aisle, gps")

    @staticmethod
    def _bounded(value, maximum: int) -> int:
        """An out-of-range or non-numeric field reads as absent, not as an error.

        Only used for the two plain numbers, and the caller substitutes the
        request's own values when they come back 0. A model that omits them for
        a re_delegate -- which is most of the time -- must not fail the parse.
        The structured fields above are stricter, because a half-understood
        action set or place is a decision rather than a blank.
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
