"""
arbiter_node.py

Arbiter agent that gates candidate mission edits before they reach the robot.

For each candidate (from /mission/candidate_xml) it checks, in order:
    1. Well-formed XML
    2. Validates against the real BT.CPP XSD
    3. Preconditions (semantic): every action has what mission.ontology says it
       needs by the time the plan reaches it — today, a SampleLeaf must be
       somewhere a robot has actually approached a tree
    4. Objective preservation / mission viability (semantic):
         - each NEW dropped tree must be justified (permanent failure)
         - total dropped trees must stay within a model-determined viability
           budget; exceeding it ABORTS the mission instead of retrying
    5. LTL verification (formal): the mission text is unchanged, the plan
       establishes every proposition the mission's formula refers to, and SPIN
       agrees the plan satisfies it. See ltl_gate.py.
    6. Edit-size limit (candidate must not rewrite the whole plan)
    7. Rate limit (min interval between ACCEPTED plans; skipped mid-retry)

Outcomes:
    - ACCEPTED  -> published to /mission/xml (sole writer)
    - REJECTED  -> reason published to /mission/rejection (planner retries)
    - ABORTED   -> reason published to /mission/abort (planner halts replanning)

The viability budget is obtained once, up front, via a single LLM call when the
pristine mission is first seen.

Parameters:
    objective_gating (bool, default True)
        Check 4. Whether the plan still contains the work the mission asked
        for, and whether enough of it survives to be worth running. This is
        the *only* check that can ABORT, so it is also the only thing that
        ends the local repair loop — with it off, a plan that quietly drops
        every objective is accepted, the planner never gives up, and nothing
        is ever escalated to the fleet. Off is for testing check 5 in
        isolation, never for a run that has to reach a coordinator.

    ltl_verification (bool, default True)
        Checks 5-7. The formal claim (a formula, and SPIN agreeing the plan
        satisfies it) plus the two churn policies that ride with it. Off,
        every accept is reported unverified — `verified` in the service
        response, `ltl_unverified` in the status — so a run with the gate down
        can never be mistaken for one without.

The two are independent on purpose. They used to be one flag, and because
check 4 sat on the LTL side of it, turning SPIN off to bring the coordination
loop up also turned off the abort that the coordination loop is triggered by:
no viability budget, no /mission/abort, no escalation, no auction. Checks 6-7
are policy about plan churn rather than about meaning and do not really belong
under `ltl_verification` either; they are left there for now because moving
them changes when a candidate is rejected, and that is a separate decision from
this one.
"""

import json
import re
import time
from dataclasses import replace
from threading import Lock, Thread
from typing import Dict, List, Optional, Tuple

from amiga_interfaces.srv import VerifyReplan
from amiga_ros2_comms.codec import Capability, Target, TargetKind, capabilities_in
from lxml import etree
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from ..mission import mission_tasks, ontology, orchard, xsd
from ..runtime import llm, prompts, spin
from ..runtime.status import StatusPublisher
from ..verification import ltl_gate

# ---------------------------------------------------------------------------
# Arbiter policy limits
# ---------------------------------------------------------------------------
MAX_CHANGED_LINE_RATIO = 0.5  # reject if > 50% of lines differ from active plan
MIN_ACCEPT_INTERVAL_SEC = 5.0  # reject if last accepted plan was < N sec ago
MAX_DROPPED_TREES = 0  # trees an edit may drop with no justification
PERMANENT_DROP_ALLOWANCE = 1  # extra NEW drops allowed when the failure is permanent
PERMANENT_KEYWORDS = ("permanent", "removed", "unavailable", "does not exist")

DEFAULT_VIABILITY_BUDGET = 2  # fallback total-drop budget if the model call fails

#: Where the orchard map arrives. Absolute, like the coordinator's: there is one
#: orchard and every robot is in it. Only the *soft* half of the ontology needs
#: it -- which aisle a tree is reached from -- so an arbiter that never receives
#: one still gates plans exactly as it did before.
ORCHARD_TOPIC = "/orchard/tree_info_json"


class ArbiterNode(Node):
    """Gates candidate mission edits before they reach the robot."""

    def __init__(self):
        super().__init__("arbiter")
        self._lock = Lock()

        #: With verification off, checks 5-7 do not run: no LTL formula, no SPIN,
        #: no edit-size or rate limit. What survives is the part that decides
        #: whether a plan will *run* -- well-formed XML, the XSD, and the
        #: ontology's required preconditions -- plus check 4, which decides
        #: whether the plan still contains the mission's work.
        #:
        #: For bringing the coordination loop up end to end, where the question
        #: is whether a task crosses robots and comes back as executable XML,
        #: and the formal argument is a separate claim made separately. Never a
        #: default: an unverified accept must not be indistinguishable from a
        #: verified one, which is also why it is logged at startup and counted
        #: as ``ltl_unverified``.
        self.declare_parameter("ltl_verification", True)
        self.ltl_verification = bool(
            self.get_parameter("ltl_verification").get_parameter_value().bool_value
        )

        #: Check 4, and separate from the flag above because it answers a
        #: different question and has a different consequence. LTL verification
        #: is a claim *about* an accepted plan; this is the only check that can
        #: declare the mission dead, and /mission/abort is what tells triage the
        #: local loop is finished. A robot whose arbiter cannot abort has no way
        #: to reach its own coordinator, so this defaults on even in the runs
        #: that switch the formal gate off.
        self.declare_parameter("objective_gating", True)
        self.objective_gating = bool(
            self.get_parameter("objective_gating").get_parameter_value().bool_value
        )

        self.active_mission_xml: Optional[str] = None
        self.original_objectives: Optional[set] = (
            None  # tree set of the pristine mission
        )
        self.justified_drops: set = set()  # trees already accepted as dropped
        self.max_droppable: Optional[int] = None  # model-determined viability budget
        self._last_failure_reason: str = ""
        self._last_accept_time = 0.0
        self._awaiting_retry = False  # True between a reject and next accept
        # Last plan we published. We also subscribe to /mission/xml (to see the
        # pristine mission from upstream), so our own accepted candidate comes
        # back to us; compare payloads to drop it. A "currently publishing" flag
        # cannot work here — publish() returns long before the message completes
        # the DDS round trip, so the flag is always back to False by then.
        self._published_xml: Optional[str] = None
        #: Did a formal check actually run on the candidate _evaluate last saw?
        #: Kept here rather than returned, so _evaluate's signature stays what
        #: the existing tests call.
        self._last_gate_verified: bool = False
        self._last_status: Dict = {
            "accepted": 0,
            "rejected": 0,
            "last_rejection_reason": None,
            "last_decision": None,
            # Counted separately from accept/reject: "accepted" and "verified"
            # are different claims, and a run where the checker never fired has
            # to be visible as such rather than hiding inside the accept count.
            "ltl_checked": 0,
            "ltl_rejected": 0,
            "ltl_unverified": 0,
            "ltl_last_reason": None,
        }

        self.xsd_schema = xsd.load_schema(self.get_logger())
        self._ltl_gate = ltl_gate.LtlGate(xsd.resolve_path(self.get_logger()))

        #: The tree -> aisle map, once the orchard arrives. None until then, and
        #: the ontology reads that as "the aisle is unknown" rather than as a
        #: reason to refuse anything.
        self.orchard: Optional[orchard.Orchard] = None

        self.create_subscription(
            String, "/mission/candidate_xml", self._on_candidate, 10
        )
        self.create_subscription(String, ORCHARD_TOPIC, self._on_orchard, 10)
        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.create_subscription(String, "/bt/status_change", self._on_bt_failure, 10)
        self.mission_pub = self.create_publisher(String, "/mission/xml", 10)
        self.rejection_pub = self.create_publisher(String, "/mission/rejection", 10)
        self.abort_pub = self.create_publisher(String, "/mission/abort", 10)
        self.budget_pub = self.create_publisher(String, "/mission/viability_budget", 10)
        self.replan_request_pub = self.create_publisher(
            String, "/mission/replan_request", 10
        )

        # The coordinator's entry point. Reentrant plus the multithreaded
        # executor in main(): the gate runs a model call and a SPIN process, and
        # blocking the single-threaded executor here would stall the
        # /mission/candidate_xml subscription this same node depends on.
        self.create_service(
            VerifyReplan,
            "/mission/verify_replan",
            self._on_verify_replan,
            callback_group=ReentrantCallbackGroup(),
        )

        self.status = StatusPublisher(self)
        self.status.publish(self.get_status())

        self.get_logger().info("ArbiterNode started — gating /mission/candidate_xml")
        if not self.ltl_verification:
            self.get_logger().warn(
                "ltl_verification=false — plans are checked for whether they RUN "
                "(XSD + preconditions) and for whether they still contain the "
                "mission's work (objective_gating), but no formula is generated "
                "and SPIN never runs. Every accept will be reported unverified."
            )
        if not self.objective_gating:
            self.get_logger().warn(
                "objective_gating=false — a candidate may drop any or all of the "
                "mission's objectives and still be accepted, and this arbiter "
                "can no longer ABORT. Nothing will publish /mission/abort, so "
                "the local repair loop has no terminal state and no fault will "
                "ever be escalated to the coordinator."
            )

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
        """Track the active plan, and — the first time we see a plan — capture the
        ORIGINAL mission's objective tree set and request a viability budget."""
        with self._lock:
            if msg.data == self._published_xml:
                return  # our own echo; _on_candidate already recorded this plan
            self.active_mission_xml = msg.data
            if self.original_objectives is None:
                try:
                    doc = etree.fromstring(msg.data.encode("utf-8"))
                except etree.XMLSyntaxError:
                    return
                self.original_objectives = self._objective_tree_ids(doc)
                self.justified_drops = set()
                mission_text = doc.findtext("Mission", default="")
                trees = sorted(self.original_objectives)
                self.get_logger().info(
                    f"Captured original mission objectives: trees {trees}"
                )
                # One thread per check, started independently: the budget is
                # check 4's input and the formula is check 5's, and the two
                # checks are switched separately.
                if self.objective_gating:
                    Thread(
                        target=self._compute_viability_budget,
                        args=(mission_text, trees),
                        daemon=True,
                    ).start()
                if self.ltl_verification:
                    # Translate the mission to LTL now, while nothing is waiting
                    # on it. The gate needs a formula for every candidate, and
                    # the model call is the slow part of it; done here, a replan
                    # pays only for SPIN. Also surfaces an unusable mission
                    # statement at mission start rather than at the first fault.
                    Thread(
                        target=self._ltl_gate.warm,
                        args=(mission_text,),
                        daemon=True,
                    ).start()

    def _on_candidate(self, msg: String):
        self._decide(msg.data)

    def _on_orchard(self, msg: String):
        """The tree -> aisle map. Latched by keeping the last one that parsed.

        A dump that does not parse is ignored rather than replacing a good map
        with an empty one -- losing the aisles mid-mission would quietly turn
        every prerequisite into an unknown.
        """
        parsed = orchard.parse(msg.data)
        if not parsed:
            return
        with self._lock:
            first = self.orchard is None
            self.orchard = parsed
        if first:
            self.get_logger().info(f"Orchard map: {len(parsed)} trees placed in aisles")

    def _on_verify_replan(self, request, response):
        """The coordinator's way in: this robot's workload changed.

        The edit is applied here rather than by the caller. The coordinator
        deliberately never parses a mission -- it knows tasks as the flat fields
        the radio carries -- and the plan to edit is the one this node already
        holds, so doing it here also removes any window where the coordinator
        would be editing a copy that has since been replaced.

        The result goes through the same ``_decide`` as a planner-authored
        candidate. Winning someone else's auction and recovering from our own
        fault are one event seen from two sides, and gating them differently
        would mean the guarantee only covers whichever path was tested.
        """
        response.accepted = False
        response.verified = False

        with self._lock:
            active = self.active_mission_xml
        if not active:
            # No plan to edit yet. Refusing would strand a task over a mission
            # we have simply not been handed; it gets gated when it arrives.
            response.accepted = True
            response.reason = "no active mission to verify against"
            return response

        try:
            candidate, appendix, dropped = self._apply_task_edit(request, active)
        except ValueError as exc:
            response.reason = str(exc)
            self.get_logger().warn(f"REJECTED coordinator edit — {exc}")
            return response

        # A transferred objective is not an abandoned one. Gate 1 of
        # _check_objective_preserved exists to stop the *planner* quietly
        # dropping work to make a failing plan pass, and it cannot tell the two
        # apart from the XML alone -- so without this, every successful transfer
        # is rejected as an unjustified drop and the auction never commits. The
        # justification is real and stronger than a fault report: a peer
        # acknowledged the GRANT, which is what makes the task theirs at all.
        departing = (
            self._objectives_leaving(active, candidate) if request.removing else set()
        )

        accepted = False
        # Only meaningful while there is a formula for the text to be checked
        # against; with verification off there is nothing to grant permission to.
        if self.ltl_verification:
            self._ltl_gate.allow_mission_text(appendix)
        with self._lock:
            self.justified_drops |= departing
        try:
            accepted, verified, reason = self._decide(candidate)
        finally:
            # One candidate only. Leaving it armed would let the planner's next
            # edit inherit the coordinator's permission to change the mission
            # text, which is the whole thing that rule prevents.
            if self.ltl_verification:
                self._ltl_gate.allow_mission_text("")
            if not accepted:
                # The edit did not go through, so nothing was handed over.
                # Leaving the drop justified would licence the planner to drop
                # that objective later for free.
                with self._lock:
                    self.justified_drops -= departing

        if accepted:
            self._request_replan(request, candidate, dropped)

        response.accepted = accepted
        response.verified = verified
        response.reason = reason
        return response

    def _request_replan(self, request, committed: str, dropped: List[str]) -> None:
        """Ask this robot's planner to re-plan around the workload it now has.

        The edit that just committed is *structurally* correct and no more than
        that. Absorbing a task installs the floor ``synthesize`` could build
        from seven bytes -- the objective and the prerequisites the ontology
        knows about -- with nothing said about where it belongs in the run, what
        it costs, or what the peer knew about doing it. Shedding one can leave a
        drive into an aisle with no work at the end of it. Both are plans that
        run; neither is a plan anybody would have written.

        So the deterministic half stops here and the model is handed the
        question, with the ontology's findings as the reason. What comes back is
        a candidate on ``/mission/candidate_xml`` and is gated like any other --
        this publishes a *request*, never a plan, which is what keeps the LLM
        outside the loop that decides what the robot actually runs.
        """
        try:
            doc = etree.fromstring(committed.encode("utf-8"))
        except etree.XMLSyntaxError:
            return
        with self._lock:
            orchard_map = self.orchard
        findings = ontology.dangling(doc, orchard_map)

        payload = {
            "cause": "task_transferred" if request.removing else "task_absorbed",
            "task_id": int(request.task_id),
            "target": {
                "kind": TargetKind(int(request.target_kind)).name.lower(),
                "a": int(request.target_a),
                "b": int(request.target_b),
            },
            "capabilities": mission_tasks.capability_names(
                int(request.required_capabilities)
            ),
            # Steps the announcement asked for that the rebuild could not
            # supply -- an arm pose it has no parameters for, an aisle it has no
            # map for. The planner is the only thing here that can fill them in.
            "dropped": list(dropped),
            "findings": findings,
            # What the peer knew about doing this work, if anything reached us.
            # The one piece of the request no amount of structure could derive.
            "note": str(getattr(request, "note", "") or ""),
            "timestamp_ms": int(time.time() * 1000),
        }
        message = String()
        message.data = json.dumps(payload)
        self.replan_request_pub.publish(message)
        self.get_logger().info(
            f"Requested replan ({payload['cause']} task {payload['task_id']}): "
            f"{len(findings)} finding(s), dropped {payload['dropped'] or 'nothing'}"
        )

    def _objectives_leaving(self, active: str, candidate: str) -> set:
        """Objective trees in the old plan that the edited one no longer has."""
        try:
            before = self._objective_tree_ids(etree.fromstring(active.encode("utf-8")))
            after = self._objective_tree_ids(
                etree.fromstring(candidate.encode("utf-8"))
            )
        except etree.XMLSyntaxError:
            return set()
        return before - after

    def _apply_task_edit(self, request, active: str) -> Tuple[str, str, List[str]]:
        """The plan with the task added or removed, the sanctioned text, and
        whatever the rebuild could not supply.

        Raises ValueError with a reason the coordinator can log and act on.
        """
        task_id = int(request.task_id)

        if request.removing:
            edited = mission_tasks.remove_task(active, task_id)
            if edited is None:
                raise ValueError(f"task {task_id} is not in the active plan")
            # Losing work never widens what the mission claims to do, so the
            # text stands and the formula with it -- which is exactly how a plan
            # that quietly drops an objective gets caught.
            return edited, "", []

        task = mission_tasks.MissionTask(
            task_id=task_id,
            name=f"task_{task_id}",
            capabilities=int(request.required_capabilities),
            target=Target(
                kind=TargetKind(int(request.target_kind)),
                a=int(request.target_a),
                b=int(request.target_b),
            ),
            priority=int(request.priority),
            xml=request.task_xml or "",
        )
        dropped: List[str] = []
        if not task.xml:
            # Won from a peer: TASK_ANNOUNCE has no room for a subtree, so it is
            # rebuilt from the fields -- against *this* robot's orchard, not the
            # announcer's plan, because the route in is the winner's to choose.
            # Still a candidate like any other: the XSD check and the
            # verification below both run on it.
            with self._lock:
                orchard_map = self.orchard
            rebuilt = mission_tasks.synthesize(
                task, xsd.resolve_path(self.get_logger()), orchard_map
            )
            if rebuilt is None:
                raise ValueError(
                    f"task {task_id} cannot be rebuilt from its announcement "
                    f"(target_kind {int(request.target_kind)}, capabilities "
                    f"{mission_tasks.capability_names(task.capabilities)})"
                )
            dropped = rebuilt.dropped
            # MissionTask is frozen: a new one, not an assignment.
            task = replace(task, xml=rebuilt.xml)

        edited = mission_tasks.insert_task(active, task)
        if edited is None:
            raise ValueError(f"could not graft task {task_id} into the active plan")

        # The mission text has to grow with the work. Without this the formula
        # still describes the old mission, the absorbed task is outside
        # everything it talks about, and the new work runs unverified while the
        # plan reports a clean check.
        clause = _mission_clause(task)
        if clause:
            edited = _extend_mission_text(edited, clause)
        return edited, clause, dropped

    def _decide(self, candidate: str) -> Tuple[bool, bool, str]:
        """Evaluate a candidate, act on the verdict, and report it.

        Returns (accepted, verified, reason). ``verified`` is not implied by
        ``accepted``: a plan can be accepted with no formal check behind it.
        """
        verdict, reason, abort = self._evaluate(candidate)
        with self._lock:
            verified = self._last_gate_verified

        if verdict:
            # Commit state before publishing, so the echo guard is already armed
            # when our own message comes back on /mission/xml.
            with self._lock:
                self._published_xml = candidate
                self.active_mission_xml = candidate
                self._last_accept_time = time.monotonic()
                self._awaiting_retry = False
                if self.original_objectives:
                    try:
                        cand_doc = etree.fromstring(candidate.encode("utf-8"))
                        cand_objs = self._objective_tree_ids(cand_doc)
                        self.justified_drops |= self.original_objectives - cand_objs
                    except etree.XMLSyntaxError:
                        pass
                self._last_status["accepted"] += 1
                self._last_status["last_decision"] = "accepted"
            out = String()
            out.data = candidate
            self.mission_pub.publish(out)
            self.get_logger().info("ACCEPTED candidate — published to /mission/xml")

        elif abort:
            with self._lock:
                self._awaiting_retry = False
                self._last_status["last_decision"] = "aborted"
                self._last_status["last_rejection_reason"] = reason
            self.get_logger().error(f"ABORTING MISSION — {reason}")
            ab = String()
            ab.data = json.dumps(
                {"reason": reason, "timestamp_ms": int(time.time() * 1000)}
            )
            self.abort_pub.publish(ab)

        else:
            with self._lock:
                self._awaiting_retry = True
                self._last_status["rejected"] += 1
                self._last_status["last_rejection_reason"] = reason
                self._last_status["last_decision"] = "rejected"
            self.get_logger().warn(f"REJECTED candidate — {reason}")
            rej = String()
            rej.data = json.dumps(
                {"reason": reason, "timestamp_ms": int(time.time() * 1000)}
            )
            self.rejection_pub.publish(rej)

        self.status.publish(self.get_status())
        return verdict, verified, reason

    def _on_bt_failure(self, msg: String):
        """The arbiter listens to failures only to know WHY the planner is
        editing — used to decide whether a drop is permanently justified."""
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(event, dict):
            with self._lock:
                self._last_failure_reason = str(event.get("reason", ""))

    # ------------------------------------------------------------------
    # Viability budget — one LLM call when the pristine mission is captured
    # ------------------------------------------------------------------

    def _compute_viability_budget(self, mission_text: str, trees: List):
        n_trees = len(trees)
        try:
            text = llm.complete(
                prompts.render("arbiter/viability_system.j2"),
                prompts.render(
                    "arbiter/viability_user.j2",
                    mission_text=mission_text,
                    trees=trees,
                    n_trees=n_trees,
                ),
            )
            matches = re.findall(r"\d+", text)  # every integer in the reply
            n = (
                int(matches[-1]) if matches else DEFAULT_VIABILITY_BUDGET
            )  # last = final answer
        except Exception as exc:
            n = DEFAULT_VIABILITY_BUDGET
            self.get_logger().warn(f"Viability call failed ({exc}); default budget {n}")

        n = max(1, min(n, n_trees))  # clamp to a sane [1, n_trees]

        with self._lock:
            self.max_droppable = n
            self._last_status["viability_budget"] = n
        b = String()
        b.data = json.dumps({"viability_budget": n, "n_trees": n_trees})
        self.budget_pub.publish(b)
        self.status.publish(self.get_status())
        self.get_logger().info(
            f"Model viability budget: up to {n} tree(s) may be skipped"
        )

    # ------------------------------------------------------------------
    # Checks
    # ------------------------------------------------------------------

    def _evaluate(self, candidate: str) -> Tuple[bool, str, bool]:
        """Returns (accepted, reason_if_not, abort). abort=True means the mission
        is no longer viable and replanning should stop, not retry."""

        # Cleared up front: the structural checks below can return before the
        # gate runs, and inheriting the previous candidate's answer would report
        # an unchecked plan as verified.
        with self._lock:
            self._last_gate_verified = False

        # 1. Well-formed XML
        try:
            doc = etree.fromstring(candidate.encode("utf-8"))
        except etree.XMLSyntaxError as exc:
            return False, f"not well-formed XML: {exc}", False

        # 2. XSD validation
        if self.xsd_schema is not None and not self.xsd_schema.validate(doc):
            return False, f"XSD validation failed: {self.xsd_schema.error_log}", False

        # 3. Semantic: every action has what it needs by the time it runs
        ok, reason = self._check_preconditions(doc)
        if not ok:
            return False, reason, False

        # 4. Semantic: objective preservation / viability. Before the
        # verification flag is consulted, not after: this is the check that
        # decides whether the mission is still worth running, and it is the only
        # one that can abort. A run with the formal gate down still needs the
        # local loop to be able to finish.
        if self.objective_gating:
            ok, reason, abort = self._check_objective_preserved(doc)
            if not ok:
                return False, reason, abort

        # Everything below is the formal claim and the two churn policies that
        # ride with it. That is the question this flag turns off; checks 1-4
        # above ask whether the plan will run and whether it is still the
        # mission, and always hold.
        if not self.ltl_verification:
            with self._lock:
                self._last_status["ltl_unverified"] += 1
                self._last_status["ltl_last_reason"] = "verification disabled"
            return True, "", False

        with self._lock:
            active = self.active_mission_xml

        # 5. LTL verification. After the cheap structural checks, because it
        #    costs a model call and a SPIN run; before the edit-size and rate
        #    limits, because those are policy and this is correctness -- a plan
        #    that violates the mission should be reported as violating it, not
        #    as too large.
        gate = self._ltl_gate.evaluate(candidate, active)
        with self._lock:
            self._last_gate_verified = gate.verified
            self._last_status["ltl_last_reason"] = gate.reason
            if gate.verified:
                self._last_status["ltl_checked"] += 1
            else:
                self._last_status["ltl_unverified"] += 1
        if not gate.accepted:
            with self._lock:
                self._last_status["ltl_rejected"] += 1
            return False, gate.reason, False
        if not gate.verified:
            # Loud on purpose. A mission that runs to completion having never
            # been checked must not look, in a log, like one that passed.
            self.get_logger().warn(f"Plan accepted without verification: {gate.reason}")

        # 6. Edit-size limit (only when we know the active plan)
        if active is not None:
            ratio = self._changed_line_ratio(active, candidate)
            if ratio > MAX_CHANGED_LINE_RATIO:
                return (
                    False,
                    (
                        f"edit too large: {ratio:.0%} of lines changed "
                        f"(limit {MAX_CHANGED_LINE_RATIO:.0%})"
                    ),
                    False,
                )

        # 7. Rate limit — skipped while in a reject→retry cycle
        with self._lock:
            since = time.monotonic() - self._last_accept_time
            awaiting_retry = self._awaiting_retry
        if (
            not awaiting_retry
            and self._last_accept_time > 0
            and since < MIN_ACCEPT_INTERVAL_SEC
        ):
            return (
                False,
                (
                    f"rate limited: last plan accepted {since:.1f}s ago "
                    f"(minimum interval {MIN_ACCEPT_INTERVAL_SEC}s)"
                ),
                False,
            )

        return True, "", False

    def _check_preconditions(self, doc) -> Tuple[bool, str]:
        """Every action must have what it needs by the time the plan reaches it.

        One rule today — a ``SampleLeaf`` needs the robot to be at a tree, or it
        samples wherever it happens to be standing — but asked of
        ``mission.ontology`` rather than restated here, because ``promela``
        refuses the same plans for the same reason and two copies of a rule
        disagree eventually. These did: this check used to scan only the
        ``SampleLeaf``'s own siblings, so a plan that approaches the tree in one
        ``RetryUntilSuccessful`` and samples it in the next was rejected even
        though nothing was wrong with it — and that is how the mission planner
        writes a retried sample.

        Soft preconditions (a tree move expects its aisle move first) are not
        checked here. They are advice for a planner, not grounds for refusal;
        every plan in ``examples/`` omits them and runs.
        """
        problems = ontology.violations(doc, self.orchard)
        return (False, problems[0]) if problems else (True, "")

    @staticmethod
    def _changed_line_ratio(original: str, edited: str) -> float:
        """Fraction of the edited plan's lines that don't appear in the original."""
        orig_lines = {line.strip() for line in original.splitlines() if line.strip()}
        edit_lines = [line.strip() for line in edited.splitlines() if line.strip()]
        if not edit_lines:
            return 1.0
        changed = sum(1 for line in edit_lines if line not in orig_lines)
        return changed / len(edit_lines)

    @staticmethod
    def _objective_tree_ids(doc) -> set:
        """The mission's objectives = the set of tree IDs the robot is meant to
        approach (approach_tree="true"). Transit moves (approach_tree="false")
        are not objectives and are ignored."""
        ids = set()
        for mv in doc.iter("MoveToTreeID"):
            if mv.get("approach_tree", "").lower() == "true":
                tid = mv.get("id")
                if tid is not None:
                    ids.add(tid)
        return ids

    def _check_objective_preserved(self, doc) -> Tuple[bool, str, bool]:
        """Two gates:
        - Gate 1 (fixable → reject): each NEWLY dropped tree must be justified
          by a permanent failure. Only new drops count; already-justified drops
          are remembered so removals accumulate across a mission.
        - Gate 2 (unfixable → abort): total dropped trees must stay within the
          model-determined viability budget."""
        with self._lock:
            original = self.original_objectives
            reason = self._last_failure_reason.lower()
            already_justified = set(self.justified_drops)
            budget = self.max_droppable
        if not original:
            return True, "", False
        if budget is None:
            budget = DEFAULT_VIABILITY_BUDGET  # call not back yet

        candidate_objs = self._objective_tree_ids(doc)
        dropped = original - candidate_objs
        newly_dropped = dropped - already_justified

        # Gate 1: each NEW drop must be justified — fixable → normal rejection
        allowed_new = MAX_DROPPED_TREES
        if any(kw in reason for kw in PERMANENT_KEYWORDS):
            allowed_new = max(allowed_new, PERMANENT_DROP_ALLOWANCE)
        if len(newly_dropped) > allowed_new:
            return (
                False,
                (
                    f"unjustified drop: {sorted(newly_dropped)} not permitted "
                    f"(already justified: {sorted(already_justified)}; "
                    f"allows {allowed_new} new this failure)"
                ),
                False,
            )

        # Gate 2: total drops exceed viability budget — unfixable → ABORT
        if len(dropped) > budget:
            return (
                False,
                (
                    f"mission no longer viable: {len(dropped)} trees dropped "
                    f"{sorted(dropped)} exceeds viability budget of {budget}"
                ),
                True,
            )

        return True, "", False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _extend_mission_text(mission_xml: str, clause: str) -> str:
    """Append ``clause`` to the plan's <Mission>, the one sanctioned edit.

    Must agree exactly with ``ltl_gate._extend``, which is what checks it: the
    two differing by so much as a separator would reject every transfer as an
    attempt to rewrite the specification.
    """
    doc = etree.fromstring(mission_xml.encode("utf-8"))
    element = doc.find("Mission")
    if element is None:
        return mission_xml
    current = (element.text or "").strip()
    element.text = f"{current}; {clause}" if current else clause
    return etree.tostring(doc, encoding="unicode")


def _mission_clause(task) -> str:
    """How an absorbed task is described in the plan's <Mission> text.

    Short and mechanical on purpose. This is the text the LTL agent reads, so it
    has to name the objective in the vocabulary the naming scheme keys on --
    "tree 35" is what yields ``sampled_tree_35``. It is generated from the wire
    fields rather than written by a model precisely so that no model can widen
    the specification it is about to be checked against.
    """
    if task.target.kind != TargetKind.TREE:
        return ""
    caps = {Capability(c) for c in capabilities_in(int(task.capabilities))}
    tree = int(task.target.a)
    if Capability.SAMPLE_LEAF in caps:
        return f"sample leaves from tree {tree}"
    return f"visit tree {tree}"


def main():
    # Multithreaded because the verification gate blocks: an LLM call for the
    # formula and a SPIN process for the check. On a single-threaded executor
    # the coordinator's /mission/verify_replan call would stall the
    # /mission/candidate_xml subscription for the duration, so the planner and
    # the coordinator would contend for the gate they both have to pass.
    spin.run(ArbiterNode, multithreaded=True)


if __name__ == "__main__":
    main()
