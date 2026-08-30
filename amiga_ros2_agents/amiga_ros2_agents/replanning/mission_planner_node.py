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
- On running out of either budget: publishes a terminal give-up to
  /mission/planner_status, which is what triage escalates on. Every path that
  stops replanning has to report itself, or the robot stalls holding work it
  will never do and nothing outside this node can tell.
- Keeps a compact memory of prior attempts

Edits are validated against the real BT.CPP XSD (amiga_ros2_behavior_tree's
installed schemas/amiga_btcpp.xsd) before being submitted.

Prompts live in prompts/mission_planner/.
"""

import json
import re
import textwrap
import time
from collections import deque
from threading import Condition, Lock, Thread
from typing import Dict, List, Optional, Set

from amiga_ros2_comms.codec import Target, TargetKind
from lxml import etree
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import String

from ..mission import mission_tasks, ontology, orchard, xsd
from ..runtime import llm, prompts, spin
from ..runtime.status import StatusPublisher

# ---------------------------------------------------------------------------
# Context-window limits
# ---------------------------------------------------------------------------
LOG_WINDOW_SEC = 30.0  # rolling /rosout window kept in memory
FAILURE_CONTEXT_SEC = 3.0  # log slice sent to LLM (±N sec around failure)
RESULT_HISTORY_CHARS = 1000  # max chars of any single result in memory
MAX_RETRIES = 3  # max planning sessions for the whole mission before giving up
MAX_REJECTION_RETRIES = 3  # max arbiter-rejection retries per single BT failure
COMPRESS_AFTER = 3  # compress memory beyond the last N entries
WORLD_STATE_FRAMES = 3  # /world_state frames kept for prompt context

# Same document, same topic, as the arbiter's own orchard subscription
# (arbiter_node.py's ORCHARD_TOPIC) -- both sides answer from the same map.
ORCHARD_TOPIC = "/orchard/tree_info_json"

# Triage's routing verdict, and the same name as triage_node.ROUTE_TOPIC --
# written out rather than imported, because importing the coordination package
# from here would pull the service definitions and the codec into every planner
# process to learn one string. A test pins the two spellings together.
ROUTE_TOPIC = "/mission/fault_route"

# How long a fault waits for a verdict before the session opens anyway.
#
# This gate must fail open. Triage not running, a model endpoint that is down, a
# reply that will not parse -- none of those should leave a robot sitting on a
# fault it could have planned around, so the timeout restores exactly the
# behaviour this gate replaced: replan and find out.
#
# Sized for triage's slowest honest path, not its usual one. With `use_vlm` set
# a verdict costs three round-trips -- the look gate, the camera, and the
# decision -- which measured 15-22 s against gpt-oss-120b with a vision model
# beside it. At 25 s that was inside the budget only until the endpoint had
# other work, and a verdict arriving late is worse than none: the planner has
# already opened a session on a fault triage was about to say was hardware.
#
# Raising it costs nothing when triage is healthy, because it publishes as soon
# as it decides and this returns immediately. It is only how long the planner
# waits on a triage that is never going to answer.
ROUTE_TIMEOUT_SEC = 45.0

# A whole XML plan comes back in one reply, so this must NOT fall back to
# llm.MAX_TOKENS (2048) — a truncated plan fails XSD validation every time.
PLANNER_MAX_TOKENS = 8192

# ---------------------------------------------------------------------------
# Mission grammar — injected into both prompt templates
# ---------------------------------------------------------------------------
# Read from the installed XSD at startup, never listed here. The literal this
# replaces held two actions of the schema's nine, and MoveToAisleHead was not
# one of them — so the replanner could not write an aisle move into a plan that
# was full of them, and every replan flattened the prerequisite chain the rest
# of this package now depends on.
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
        # The tree -> aisle map, once the orchard arrives. Same lifetime as the
        # arbiter's own copy (arbiter_node.py's self.orchard): None until the
        # first frame, latched after.
        self.orchard: Optional[orchard.Orchard] = None
        # Trees this robot has actually sampled this mission (SUCCESS half of
        # /bt/status_change) and the fault reasons raised so far. Both latch
        # for the node's lifetime -- same "first mission XML is the mission"
        # assumption the arbiter's original_objectives already makes.
        self.completed_trees: Set[str] = set()
        # Every leaf that reported SUCCESS, in order, by name. The general
        # ledger: bt_runner publishes this for *any* action, so adding a new
        # one (SprayTree, PruneBranch) needs no code here and no new fact type.
        # completed_trees above stays because pruning removes work by tree id,
        # which is the one place a tree id is genuinely the right key.
        self.succeeded_nodes: List[str] = []
        # Targets a peer won from this robot at auction. Not "done" -- done
        # somewhere else, which for planning purposes means the same thing:
        # sending this robot there too puts it where another one is already
        # going.
        #
        # Needed because the plan alone cannot say it. `remove_task` takes the
        # transferred objective out of the XML, but the <Mission> text is a
        # ratchet: absorbing a task extends it, transferring one deliberately
        # leaves it alone (that asymmetry is what catches a plan quietly
        # abandoning work). So after a hand-off the plan says "sample tree 26"
        # while the mission still says "sample trees 20 and 26", and the next
        # replan reads the mission, sees 20 missing, and restores it. Seen end
        # to end: amiga3 gave tree 20 to amiga2, dropped it correctly for two
        # plans, then re-added it -- and the two robots deadlocked in aisle 2,
        # one of them standing on the waypoint the other needed.
        self.transferred_targets: Set[Target] = set()
        self.fault_constraints: List[Dict] = []
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
        self.valid_leaves = mission_tasks.action_grammar(
            xsd.resolve_path(self.get_logger())
        )

        self.system_prompt = prompts.render(
            "mission_planner/system.j2",
            valid_leaves=self.valid_leaves,
            valid_controls=VALID_CONTROLS,
            max_edit_lines=MAX_EDIT_LINES,
            # What the actions mean to each other. Without this the model can
            # write every leaf the schema allows and still produce a plan that
            # samples from the aisle head.
            ontology_rules=ontology.RULES,
        )

        # Subscriptions
        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        self.create_subscription(String, "/bt/status_change", self._on_bt_failure, 10)
        self.create_subscription(String, "/mission/rejection", self._on_rejection, 10)
        self.create_subscription(String, "/mission/abort", self._on_abort, 10)
        self.create_subscription(String, "/world_state", self._on_world_state, 10)
        self.create_subscription(String, ORCHARD_TOPIC, self._on_orchard, 10)
        self.create_subscription(
            String, "/mission/replan_request", self._on_replan_request, 10
        )
        self.create_subscription(String, ROUTE_TOPIC, self._on_fault_route, 10)

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
        # (template, extra) of the session in flight, so a rejection retries the
        # same question rather than the default one.
        self._last_planner_call = ("mission_planner/replan_user.j2", {}, "")
        # (cause, node) pairs already reported as a give-up. Every later BT
        # failure re-enters _run_planner and would re-announce an exhausted
        # budget on each one, which downstream reads as a fresh escalation.
        self._gave_up: Set[tuple] = set()
        # Routing verdicts from triage, keyed by failing node name, and the
        # condition a waiting session blocks on.
        self._routes: Dict[str, Dict] = {}
        self._route_cv = Condition()

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

    def _on_orchard(self, msg: String):
        """Latch the tree -> aisle map, same as the arbiter's own copy.

        Not reset on a later frame vs. merged with it -- the document is a
        full snapshot each time, so the newest one simply replaces the last.
        """
        parsed = orchard.parse(msg.data)
        with self._lock:
            first = self.orchard is None
            self.orchard = parsed
        if first and parsed:
            self.get_logger().info(
                f"Orchard layout received — {len(parsed)} trees mapped"
            )

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

        # No "status" key is the synthetic `make fleet-fault` shape, which has
        # only ever meant FAILURE. A real event carries it explicitly, and now
        # that FaultReporter reports a leaf reaching SUCCESS too (not just
        # FAILURE), this is what keeps that from starting a replan session.
        status = str(event.get("status") or "FAILURE").upper()
        if status == "SUCCESS":
            self._on_leaf_success(event)
            return
        if status != "FAILURE":
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

        # Run planner in a background thread so the spin loop stays responsive.
        # The thread blocks on triage's verdict first -- see _repair_if_routed.
        Thread(
            target=self._repair_if_routed, args=(event, log_context), daemon=True
        ).start()

    def _on_fault_route(self, msg: String):
        """Triage's verdict on a fault. Wakes whoever is waiting for it."""
        try:
            route = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Could not parse {ROUTE_TOPIC} payload")
            return
        if not isinstance(route, dict):
            return
        name = str(route.get("node") or "")
        if not name:
            return
        with self._route_cv:
            self._routes[name] = route
            self._route_cv.notify_all()

    def _repair_if_routed(self, event: Dict, log_context: List[Dict]) -> None:
        """Open a planning session only if the fault is worth one.

        The budget used to be the only thing standing between a fault and three
        model calls, and a budget cannot tell a plan that can be fixed from a
        camera that is broken. It spent the same three sessions on both, and on
        the broken camera the sessions were worse than useless: each edit
        wrapped the dead leaf in another retry, and each accepted plan replaced
        the one the escalation would have named its failing node in.

        So the verdict comes first. ``escalate`` means the work leaves this
        robot -- the coordinator sheds it and sends back a
        ``/mission/replan_request``, which is where this robot's plan actually
        gets rewritten. Nothing is lost by not planning here; the rest of the
        mission is untouched and still runs.

        The tree's own outcome is not routable and is not waited for. Triage
        rules on leaves, because a verdict is only useful when it names a unit
        of work the fleet could be offered, and ``<tree>`` names the whole
        mission. Waiting on a verdict triage will never publish just costs
        ROUTE_TIMEOUT_SEC of silence before replanning anyway -- and by then
        the leaf underneath has already been ruled on.
        """
        name = str(event.get("node") or "")
        whole_tree = name in ("", "<tree>") or event.get("source") == "tree"
        route = {"route": "repair"} if whole_tree else self._await_route(name)
        if str(route.get("route")) == "escalate":
            self.get_logger().info(
                f"Fault on {name!r} routed to the fleet — no local replan. "
                f"{str(route.get('rationale', ''))[:160]}"
            )
            return
        self._run_planner(
            event, log_context, route_guidance=str(route.get("guidance") or "")
        )

    def _await_route(self, name: str) -> Dict:
        """Block until triage rules on this node, or ROUTE_TIMEOUT_SEC passes.

        A verdict already in hand returns immediately, which is what makes the
        second and third tick of a leaf failing under its own retry decorator
        free -- triage re-publishes the cached verdict, and this never waits.
        """
        deadline = time.monotonic() + ROUTE_TIMEOUT_SEC
        with self._route_cv:
            while name not in self._routes:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.get_logger().warn(
                        f"No routing verdict for {name!r} after "
                        f"{ROUTE_TIMEOUT_SEC:.0f}s — replanning anyway"
                    )
                    return {"route": "repair"}
                self._route_cv.wait(remaining)
            return dict(self._routes[name])

    def _on_leaf_success(self, event: Dict):
        """A leaf finished. Recorded twice, at two different levels.

        The name goes into ``succeeded_nodes`` unconditionally — that is the
        general ledger, and it is general precisely because it interprets
        nothing: bt_runner reports a SUCCESS the same way for a SampleLeaf as
        for anything added to the schema later, so what the model is told about
        this robot's progress does not need a new fact type per action.

        The tree id, when there is one, additionally goes into
        ``completed_trees``, because the executor rebuilds its tree from
        scratch on every redeploy — anything still in the plan gets re-ticked —
        and the pruning that prevents that removes work by tree id.
        """
        name = event.get("node")
        if not name or name == "<tree>":
            return
        with self._lock:
            if name not in self.succeeded_nodes:
                self.succeeded_nodes.append(name)
            xml = self.current_mission_xml
            orchard_map = self.orchard
        if xml is None:
            return
        try:
            doc = etree.fromstring(xml.encode("utf-8"))
        except etree.XMLSyntaxError:
            return
        tree_id = ontology.objective_trees(doc, orchard_map).get(name)
        if tree_id is None:
            return
        with self._lock:
            newly = tree_id not in self.completed_trees
            self.completed_trees.add(tree_id)
        if newly:
            self.get_logger().info(
                f"Tree {tree_id} sampled — excluding it from future replans"
            )

    def _on_replan_request(self, msg: String):
        """This robot's workload changed; the plan that made it is structural.

        The arbiter has already committed an edit — a task absorbed from a peer
        or one handed away — and what it committed is what the deterministic
        half could build: the objective plus the prerequisites the ontology
        knows about. Nothing in it says where the new work belongs in the run,
        what the peer knew about doing it, or which leftover steps now lead
        nowhere. That is the part only a planner can write, and it is why the
        request carries findings and a note rather than a diff.

        Same session machinery as a fault, and deliberately so: what comes out
        is a candidate on ``/mission/candidate_xml`` and the arbiter gates it
        exactly as it gates a fault-driven edit. No second writer of
        ``/mission/xml``, no model call inside a service handler.
        """
        if self._mission_aborted:
            return
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse /mission/replan_request payload")
            return
        if not isinstance(request, dict):
            return

        # A pseudo-event, so the retry counter, the memory and the rejection
        # loop all key off this the way they key off a failing node.
        cause = str(request.get("cause", "workload_changed"))
        self._record_ownership_change(cause, request.get("target"))
        event = {
            "node": f"{cause}:{request.get('task_id')}",
            "reason": cause,
            "timestamp_ms": request.get("timestamp_ms", 0),
        }

        # The repair budget is spent, and this is the moment it stops applying.
        #
        # MAX_RETRIES bounds how long a robot may keep asking a model to fix ONE
        # situation, and a workload change is the end of that situation: the
        # coordinator has committed an edit, so either the work that could not
        # be done is now a peer's, or work that is not ours has just arrived.
        # Either way the plan in front of the planner is not the plan the budget
        # was spent on.
        #
        # Without this reset, a robot that shed a task stayed exhausted for the
        # rest of the mission, and the damage was not the missed refinement --
        # it was everything after. Observed end to end: amiga1 lost its depth
        # camera, burned all three sessions on tree 20, shed it to amiga3, and
        # then drove to tree 64 and hit the identical fault with no budget left
        # to replan and -- because _gave_up had already recorded
        # max_retries_exhausted and dedupes on it -- no way to escalate a second
        # time either. The one broken camera it could have offered the fleet
        # twice was offered once, and tree 64 was abandoned in silence.
        #
        # _gave_up is cleared for the same reason: it exists so ONE exhausted
        # budget is announced once, not so a robot may only ever give up once.
        with self._lock:
            self._last_status["sessions"] = 0
            self._gave_up.clear()
            self._rejection_retries = 0
            self._last_rejection_reason = None
            self._last_failure_event = None
        self.get_logger().info(
            f"Replan requested — {cause} task {request.get('task_id')}, "
            f"{len(request.get('findings') or [])} finding(s)"
        )
        Thread(
            target=self._run_planner,
            args=(event, []),
            kwargs={
                "template": "mission_planner/replan_request_user.j2",
                "extra": {"request": request},
            },
            daemon=True,
        ).start()

    def _record_ownership_change(self, cause: str, target) -> None:
        """Remember which targets stopped being this robot's, and which came back.

        Every kind of target, not just trees -- an aisle or a GPS waypoint a
        peer took is exactly as re-sendable-to as a tree is, so excluding them
        left a hole this ledger exists to close.

        Absorbing clears the mark rather than ignoring it, so a target handed
        away and later won back is plannable again -- the ledger tracks who
        owns the work now, not what has ever left.
        """
        if not isinstance(target, dict):
            return
        try:
            parsed = Target(
                kind=TargetKind[str(target.get("kind", "none")).upper()],
                a=int(target.get("a") or 0),
                b=int(target.get("b") or 0),
            )
        except (KeyError, ValueError, TypeError):
            return
        if not parsed.placed:
            return
        with self._lock:
            if cause == "task_transferred":
                if parsed not in self.transferred_targets:
                    self.transferred_targets.add(parsed)
                    self.get_logger().info(
                        f"{parsed} is a peer's now — excluding it from "
                        "future replans"
                    )
            elif cause == "task_absorbed":
                self.transferred_targets.discard(parsed)

    def _strip_transferred(self, xml: str, transferred: "Set[Target]") -> Optional[str]:
        """``xml`` with any task on a transferred target cut out.

        The mission text still names transferred work, so a model asked not to
        restore it can still do so by reading the <Mission> line rather than
        the instruction. This is the check that catches it regardless: a
        task's target survives an edit even when its element names do not, so
        matching on target (not on task_id, which the edit can freely change)
        is what makes this a backstop and not just another prompt.

        Returns None if nothing is left to run.
        """
        for task in mission_tasks.tasks_in(xml):
            if task.target not in transferred:
                continue
            stripped = mission_tasks.remove_task(xml, task.task_id)
            if stripped is None:
                continue
            self.get_logger().warn(
                f"  Edit re-added {task.target}, which belongs to a peer — "
                "removing it before publishing"
            )
            xml = stripped
        return xml if mission_tasks.tasks_in(xml) else None

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
                gave_up_node = event.get("node")
                gave_up = True
            else:
                self._rejection_retries += 1
                self._last_rejection_reason = reason
                attempt = self._rejection_retries
                template, extra, guidance = self._last_planner_call
                gave_up = False

        if gave_up:
            # Published outside the lock: _report_gave_up takes it, and this is
            # a plain Lock rather than an RLock.
            self._report_gave_up(
                cause="rejection_retries_exhausted",
                node=gave_up_node,
                reason=f"gave up after {MAX_REJECTION_RETRIES} rejections: {reason}",
            )
            return

        self.get_logger().warn(
            f"Candidate rejected ({reason}) — retry {attempt}/{MAX_REJECTION_RETRIES}"
        )
        Thread(
            target=self._run_planner,
            args=(event, []),
            kwargs={
                "template": template,
                "extra": extra,
                # Carried into the retry too. Triage's reading of the logs is
                # no less true because the arbiter refused the first edit made
                # from it, and dropping it here would mean every retry planned
                # with strictly less evidence than the attempt it is fixing.
                "route_guidance": guidance,
            },
            daemon=True,
        ).start()

    def _report_gave_up(self, cause: str, node, reason: str) -> None:
        """Announce that local repair is over, on /mission/planner_status.

        This is the *only* thing that tells triage the planner has finished
        trying, and therefore the only route from a fault this robot cannot fix
        to /coordination/infeasible and the fleet. A give-up that stops at a log
        line — which is what reaching MAX_RETRIES used to do — is a silent stall:
        the robot holds work it will never do and nobody outside this node ever
        finds out.

        The payload keys are the contract triage reads. `outcome` names the
        terminal state, `cause` says which budget ran out, and `last_reason`
        carries the arbiter's own words so the escalation says something more
        useful than "gave up".

        Deduplicated, because _run_planner is re-entered on every later BT
        failure and an exhausted budget stays exhausted -- without this, one
        give-up would be re-announced for the rest of the mission.

        What the key is depends on which budget ran out, and getting that wrong
        escalates twice. Rejection retries are counted per node, so two nodes
        exhausting their retries are two separate give-ups. MAX_RETRIES counts
        planning *sessions* for the whole mission, so it is one give-up no
        matter which node's failure happened to be the last one in: a leaf
        failure is always followed by the tree's own FAILURE event, so keying
        that on the node reliably fires once for the leaf and again for
        "<tree>", and triage escalates both.
        """
        key = (cause,) if cause == "max_retries_exhausted" else (cause, node)
        with self._lock:
            if key in self._gave_up:
                return
            self._gave_up.add(key)
            self._last_status["last_edit_summary"] = reason

        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "outcome": "gave_up",
                        "cause": cause,
                        "node": node,
                        "last_reason": reason,
                    }
                )
            )
        )
        self.status.publish(self.get_status())
        self.get_logger().error(f"Local replanning gave up ({cause}): {reason}")

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

    def _run_planner(
        self,
        event: Dict,
        log_context: List[Dict],
        template: str = "mission_planner/replan_user.j2",
        extra: Optional[Dict] = None,
        route_guidance: str = "",
    ):
        """One planning session: build context → call LLM → publish candidate.

        ``template`` and ``extra`` are what let a workload change reuse this
        verbatim. Everything after the prompt is the same either way — the XSD
        check, the candidate publish, the memory, the rejection loop — and
        forking it would mean the arbiter's retry feedback only worked for one
        of the two reasons a robot replans.

        ``route_guidance`` is triage's one sentence about what it saw in the
        logs, when it saw something. It arrives with the verdict that let this
        session open at all, so it is the only piece of context here derived
        from evidence *outside* this node's own buffers.
        """
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
            # A terminal outcome like any other. This used to return here and
            # say nothing, so a robot that exhausted its planning budget looked
            # from the outside exactly like one that had stopped failing.
            self._report_gave_up(
                cause="max_retries_exhausted",
                node=event.get("node"),
                reason=(
                    f"reached MAX_RETRIES ({MAX_RETRIES}) planning sessions "
                    f"without a plan that runs"
                ),
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
            # Held so a rejection retries the session that was actually run. A
            # retry that fell back to the fault template would ask the model to
            # fix a failing node for a session that had no failure in it.
            self._last_planner_call = (template, dict(extra or {}), route_guidance)

        self.get_logger().info(_box(f"Mission Planner — session {sessions_done + 1}"))

        # 1. Prune what's already done out of the XML the model sees. The
        # executor has no resume point (a redeploy ticks from the root), so a
        # tree left in the plan gets driven through again — this removes that
        # possibility structurally instead of asking the model to notice it.
        with self._lock:
            completed = set(self.completed_trees)
            succeeded = list(self.succeeded_nodes)
            transferred = set(self.transferred_targets)
            orchard_map = self.orchard
        try:
            active_doc = etree.fromstring(xml.encode("utf-8"))
        except etree.XMLSyntaxError:
            active_doc = None
        if active_doc is None:
            pruned_xml = xml
            pruned_tree_ids: List[str] = []
        else:
            pruned_doc = ontology.prune_completed(active_doc, completed, orchard_map)
            if not any(True for _ in ontology.actions_in(pruned_doc)):
                self.get_logger().warn(
                    "Every objective in the active mission is already complete "
                    "— nothing left to replan"
                )
                return
            pruned_xml = etree.tostring(pruned_doc, encoding="unicode")
            pruned_tree_ids = sorted(
                {
                    (el.get("id") or "").strip()
                    for el in ontology.actions_in(pruned_doc)
                    if el.tag == "MoveToTreeID" and el.get("id")
                }
            )

        # 2. Orchard grounding, scoped to the trees still in play — the
        # concrete fact that was missing when a fault reason like "approach
        # tree 60 not via aisle 6" had nothing to contradict aisle 9 with.
        if orchard_map:
            orchard_facts = orchard.sides_for_trees(orchard_map, pruned_tree_ids)
            known_aisles = sorted(orchard_map.aisles())
        else:
            orchard_facts = {}
            known_aisles = []

        # 3. Fault reasons from earlier this mission, so a constraint raised
        # once (e.g. this soft-ground note) stays visible on every later
        # replan, not just the session it arrived in.
        with self._lock:
            reason = str(event.get("reason") or "")
            event_key = (event.get("node"), event.get("timestamp_ms"))
            seen_keys = {(c["node"], c["timestamp_ms"]) for c in self.fault_constraints}
            prior_fault_constraints = list(self.fault_constraints)
            if reason and event_key not in seen_keys:
                self.fault_constraints.append(
                    {
                        "node": event.get("node"),
                        "reason": reason,
                        "timestamp_ms": event.get("timestamp_ms"),
                    }
                )

        # 4. World state — whatever the /world_state subscription has collected
        with self._lock:
            world_state = list(self.world_state_frames)

        # 5. Compact memory relevant to this failure node
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

        # 6. Build prompt
        log_excerpt = json.dumps(log_context, indent=2)
        if len(log_excerpt) > RESULT_HISTORY_CHARS:
            log_excerpt = log_excerpt[:RESULT_HISTORY_CHARS] + "\n… [truncated]"

        with self._lock:
            rejection_note = self._last_rejection_reason

        fields = dict(extra or {})
        if "request" in fields:
            fields["request_json"] = json.dumps(fields["request"], indent=2)

        prompt = prompts.render(
            template,
            rejection_reason=rejection_note,
            event=json.dumps(event, indent=2),
            mission_xml=pruned_xml,
            xsd_text=self.xsd_text,
            world_state=json.dumps(world_state, indent=2),
            world_state_count=len(world_state),
            log_excerpt=log_excerpt,
            failure_node=failure_node,
            memory_summary=json.dumps(memory_summary, indent=2),
            failure_context_sec=FAILURE_CONTEXT_SEC,
            valid_leaves=self.valid_leaves,
            ontology_rules=ontology.RULES,
            max_edit_lines=MAX_EDIT_LINES,
            orchard_facts=orchard_facts,
            known_aisles=known_aisles,
            completed_trees=sorted(completed),
            transferred_targets=sorted(str(t) for t in transferred),
            succeeded_nodes=succeeded,
            prior_fault_constraints=prior_fault_constraints,
            route_guidance=route_guidance,
            **fields,
        )

        self.get_logger().info(
            f"  Calling model ({llm.MODEL}) — "
            f"world_state={len(world_state)} frames, logs={len(log_context)} entries, "
            f"completed_trees={sorted(completed)}, pruned={len(completed)}, "
            f"transferred_targets={sorted(str(t) for t in transferred)}"
        )

        # 7. Call LLM
        try:
            edited_xml = llm.complete(
                self.system_prompt, prompt, max_tokens=PLANNER_MAX_TOKENS
            )
        except Exception as exc:
            self.get_logger().error(f"  LLM call failed: {exc}")
            return

        edited_xml = llm.strip_code_fence(edited_xml)

        # 8. Validate against the real BT.CPP XSD
        is_valid, xsd_error = xsd.validate(self.xsd_schema, edited_xml)
        if not is_valid:
            self.get_logger().error(
                "  LLM edit failed XSD validation — not publishing\n"
                f"  {xsd_error}\n"
                f"  Response preview: {edited_xml[:200]}"
            )
            return

        # 8b. The mission text still names transferred work (see
        # transferred_targets above), so a model reading it can restore what a
        # peer already owns despite being told not to. The prompt is a
        # courtesy; this is the backstop -- the same move as pruning completed
        # work out before the call, done after instead, because the risk here
        # is the model ADDING work back rather than leaving it in.
        if transferred:
            edited_xml = self._strip_transferred(edited_xml, transferred)
            if edited_xml is None:
                self.get_logger().warn(
                    "  Edit had no work left after removing transferred "
                    "targets — not publishing"
                )
                return

        # 9. Summarise what changed, against the pruned XML the model was
        # actually shown — not the raw active mission, which may still
        # contain trees the model never saw and could not have touched.
        edit_summary = _summarize_edit(pruned_xml, edited_xml)
        self.get_logger().info(
            f"  Edit: {textwrap.shorten(edit_summary, 120, placeholder='…')}"
        )

        # 10. Publish candidate to the Arbiter
        out_msg = String()
        out_msg.data = edited_xml
        self.mission_pub.publish(out_msg)
        self.get_logger().info("  Published candidate XML to /mission/candidate_xml")

        # 11. Store in memory and update status
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
