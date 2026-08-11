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
from collections import deque
from threading import Lock, Thread
from typing import Dict, List, Optional, Set

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
MAX_RETRIES = 20  # max planning attempts before giving up (= MAX_STEPS)
MAX_REJECTION_RETRIES = 3  # max arbiter-rejection retries per single BT failure
COMPRESS_AFTER = 3  # compress memory beyond the last N entries
WORLD_STATE_FRAMES = 3  # /world_state frames kept for prompt context

# Same document, same topic, as the arbiter's own orchard subscription
# (arbiter_node.py's ORCHARD_TOPIC) -- both sides answer from the same map.
ORCHARD_TOPIC = "/orchard/tree_info_json"

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
        self._last_planner_call = ("mission_planner/replan_user.j2", {})
        # (cause, node) pairs already reported as a give-up. Every later BT
        # failure re-enters _run_planner and would re-announce an exhausted
        # budget on each one, which downstream reads as a fresh escalation.
        self._gave_up: Set[tuple] = set()

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

        # Run planner in a background thread so the spin loop stays responsive
        Thread(target=self._run_planner, args=(event, log_context), daemon=True).start()

    def _on_leaf_success(self, event: Dict):
        """A leaf finished. If it was a SampleLeaf, remember which tree —
        the executor rebuilds its tree from scratch on every redeploy, so
        anything still in the plan gets re-ticked; this is what lets a later
        replan leave a finished tree out instead of driving through it again.
        """
        name = event.get("node")
        if not name or name == "<tree>":
            return
        with self._lock:
            xml = self.current_mission_xml
            orchard_map = self.orchard
        if xml is None:
            return
        try:
            doc = etree.fromstring(xml.encode("utf-8"))
        except etree.XMLSyntaxError:
            return
        tree_id = ontology.sample_leaf_trees(doc, orchard_map).get(name)
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
        event = {
            "node": f"{cause}:{request.get('task_id')}",
            "reason": cause,
            "timestamp_ms": request.get("timestamp_ms", 0),
        }
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
                template, extra = self._last_planner_call
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
            kwargs={"template": template, "extra": extra},
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
    ):
        """One planning session: build context → call LLM → publish candidate.

        ``template`` and ``extra`` are what let a workload change reuse this
        verbatim. Everything after the prompt is the same either way — the XSD
        check, the candidate publish, the memory, the rejection loop — and
        forking it would mean the arbiter's retry feedback only worked for one
        of the two reasons a robot replans.
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
            self._last_planner_call = (template, dict(extra or {}))

        self.get_logger().info(_box(f"Mission Planner — session {sessions_done + 1}"))

        # 1. Prune what's already done out of the XML the model sees. The
        # executor has no resume point (a redeploy ticks from the root), so a
        # tree left in the plan gets driven through again — this removes that
        # possibility structurally instead of asking the model to notice it.
        with self._lock:
            completed = set(self.completed_trees)
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
            orchard_facts = orchard.facts_for_trees(orchard_map, pruned_tree_ids)
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
            prior_fault_constraints=prior_fault_constraints,
            **fields,
        )

        self.get_logger().info(
            f"  Calling model ({llm.MODEL}) — "
            f"world_state={len(world_state)} frames, logs={len(log_context)} entries, "
            f"completed_trees={sorted(completed)}, pruned={len(completed)}"
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
