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
    5. Edit-size limit (candidate must not rewrite the whole plan)
    6. Rate limit (min interval between ACCEPTED plans; skipped mid-retry)

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
        is ever escalated to the fleet. Defaults on: a robot whose arbiter
        cannot abort has no way to reach its own coordinator.

This used to run a formal LTL check here too — a specification generated from
the mission text, verified against the plan with SPIN — gated by a second flag
(`ltl_verification`). That check has been removed; this pipeline does not use
LTL. Checks 1-4 above ask whether the plan will run and whether it is still
the mission, and always hold.
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

# ---------------------------------------------------------------------------
# Arbiter policy limits
# ---------------------------------------------------------------------------
MAX_CHANGED_LINE_RATIO = 0.5  # reject if > 50% of lines differ from active plan
MIN_ACCEPT_INTERVAL_SEC = 5.0  # reject if last accepted plan was < N sec ago
MAX_DROPPED_TREES = 0  # trees an edit may drop with no justification
PERMANENT_DROP_ALLOWANCE = 1  # extra NEW drops allowed when the failure is permanent
PERMANENT_KEYWORDS = ("permanent", "removed", "unavailable", "does not exist")

DEFAULT_VIABILITY_BUDGET = 2  # fallback total-drop budget if the model call fails

#: How long to hold an accepted plan before publishing it unheard-from. Longer
#: than a mission normally takes to reach its next tree, short enough that a
#: lost mission-end message is not a lost plan. Failing open is safe: see
#: _release_stale_hold.
HOLD_TIMEOUT_SEC = 600.0

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

        #: The only check that can declare the mission dead: /mission/abort is
        #: what tells triage the local loop is finished. A robot whose arbiter
        #: cannot abort has no way to reach its own coordinator, so this
        #: defaults on.
        self.declare_parameter("objective_gating", True)
        self.objective_gating = bool(
            self.get_parameter("objective_gating").get_parameter_value().bool_value
        )

        self.active_mission_xml: Optional[str] = None
        self.original_objectives: Optional[set] = (
            None  # tree set of the pristine mission
        )
        self.justified_drops: set = set()  # trees already accepted as dropped
        # Objectives this robot has actually finished, off the SUCCESS half of
        # /bt/status_change. Distinct from justified_drops: a justified drop is
        # work nobody is doing, a completed objective is work already done, and
        # conflating them is what made an accepted plan look like an abandoned
        # one. See _check_objective_preserved.
        self.completed_objectives: set = set()
        # Objectives a peer took at auction. A third case again: not done, not
        # abandoned -- being done by somebody else. Only the viability budget
        # needs the distinction, and it needs it badly. The budget answers "how
        # much of this mission may go UNDONE", and work that left for a robot
        # that can do it is not undone by anyone's reading. Counting it as such
        # was live: amiga1's camera was dead, both its trees went to peers who
        # sampled them, and its own arbiter aborted the mission on the second
        # transfer for "2 trees dropped ['20', '64'] exceeds viability budget
        # of 1" -- while robot 2 was standing at tree 20.
        self.transferred_objectives: set = set()

        # Delivery of an accepted plan, deferred until the robot can take it.
        #
        # bt_runner cannot adopt a plan mid-mission -- its tick loop only exits
        # when the tree returns SUCCESS or FAILURE -- so publishing early never
        # made work start sooner. What it did do was leave the plan sitting in
        # bt_runner's single pending slot going stale: a plan written before a
        # tree was sampled, adopted ninety seconds after it was, still contains
        # that tree, and the executor re-ticks from the root. Observed: amiga3
        # sampled tree 26, then drove back and sampled it again.
        #
        # Holding costs nothing for the same reason it fixes something. The
        # plan runs at exactly the moment it would have anyway; it is simply
        # pruned against what is finished *then* rather than when it was
        # written.
        self._held_plan: Optional[str] = None
        self._held_since = 0.0
        self._executor_busy = False
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
        self._last_status: Dict = {
            "accepted": 0,
            "rejected": 0,
            "last_rejection_reason": None,
            "last_decision": None,
        }

        self.xsd_schema = xsd.load_schema(self.get_logger())

        #: The tree -> aisle map, once the orchard arrives. None until then, and
        #: the ontology reads that as "the aisle is unknown" rather than as a
        #: reason to refuse anything.
        self.orchard: Optional[orchard.Orchard] = None

        self.create_subscription(
            String, "/mission/candidate_xml", self._on_candidate, 10
        )
        self.create_subscription(String, ORCHARD_TOPIC, self._on_orchard, 10)
        self.create_subscription(String, "/mission/xml", self._on_mission, 10)
        self.create_subscription(String, "/bt/status_change", self._on_bt_status, 10)
        self.mission_pub = self.create_publisher(String, "/mission/xml", 10)
        self.rejection_pub = self.create_publisher(String, "/mission/rejection", 10)
        self.abort_pub = self.create_publisher(String, "/mission/abort", 10)
        self.budget_pub = self.create_publisher(String, "/mission/viability_budget", 10)
        self.replan_request_pub = self.create_publisher(
            String, "/mission/replan_request", 10
        )

        # The coordinator's entry point. Reentrant plus the multithreaded
        # executor in main(): blocking the single-threaded executor here would
        # stall the /mission/candidate_xml subscription this same node depends
        # on.
        self.create_service(
            VerifyReplan,
            "/mission/verify_replan",
            self._on_verify_replan,
            callback_group=ReentrantCallbackGroup(),
        )

        self.status = StatusPublisher(self)
        # Only ever fires when bt_runner has gone quiet on us; see
        # _release_stale_hold.
        self.create_timer(5.0, self._release_stale_hold)
        self.status.publish(self.get_status())

        self.get_logger().info("ArbiterNode started — gating /mission/candidate_xml")
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
            # Somebody else put a plan on the topic -- the first mission arrives
            # this way -- so the robot is running one and an accepted candidate
            # has to wait its turn like any other.
            self._executor_busy = True
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
                if self.objective_gating:
                    Thread(
                        target=self._compute_viability_budget,
                        args=(mission_text, trees),
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

        with self._lock:
            active = self.active_mission_xml
        if not active:
            # No plan to edit yet. Refusing would strand a task over a mission
            # we have simply not been handed; it gets gated when it arrives.
            response.accepted = True
            response.reason = "no active mission to verify against"
            return response

        try:
            candidate, dropped = self._apply_task_edit(request, active)
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
        reason = ""

        # A removal that empties the plan has no candidate the normal gate
        # can pass: a control node needs a child, so there is no valid XML
        # for "this robot now has nothing to do", and _decide would XSD-
        # reject it. That is what left a shed task looking rejected while the
        # auction had already handed it to a peer -- the removal never
        # actually landed, so this robot kept driving to a tree that was
        # someone else's, failed on it again, and the fleet auctioned the
        # same task a second time.
        #
        # The removal itself is not in question -- request.removing said so,
        # and departing is computed either way -- only whether there is a
        # document left to check. There is nothing to publish here, so
        # nothing downstream ever sees the empty candidate: no XSD check
        # needed, no document written.
        if request.removing and not mission_tasks.tasks_in(candidate):
            with self._lock:
                self.active_mission_xml = candidate
                self._awaiting_retry = False
                self.justified_drops |= departing
                self.transferred_objectives |= departing
                self._last_status["accepted"] += 1
                self._last_status["last_decision"] = "accepted"
            accepted = True
            reason = "removal leaves no work — accepted with nothing to publish"
            self.get_logger().info(f"ACCEPTED coordinator edit — {reason}")
            self.status.publish(self.get_status())
        else:
            with self._lock:
                self.justified_drops |= departing
                self.transferred_objectives |= departing
            try:
                accepted, reason = self._decide(candidate)
            finally:
                if not accepted:
                    # The edit did not go through, so nothing was handed
                    # over. Leaving the drop justified would licence the
                    # planner to drop that objective later for free.
                    with self._lock:
                        self.justified_drops -= departing
                        self.transferred_objectives -= departing

        if accepted:
            self._request_replan(request, candidate, dropped)

        response.accepted = accepted
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
            # The plan the edit above actually produced -- not a reference to
            # it, the text itself. /mission/xml only carries a plan once the
            # robot is free to adopt it, so a robot already mid-mission has no
            # other way to see this: its own copy of "the current plan" is
            # whatever was last delivered, which can predate this commit by
            # as long as the mission runs. Without this, the planner's next
            # session edits that stale copy from scratch, rebuilding a unit
            # this commit already wrote correctly and losing whatever the
            # rebuild leaves out.
            "committed_mission_xml": committed,
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

    def _apply_task_edit(self, request, active: str) -> Tuple[str, List[str]]:
        """The plan with the task added or removed, and whatever the rebuild
        could not supply.

        Raises ValueError with a reason the coordinator can log and act on.
        """
        task_id = int(request.task_id)

        if request.removing:
            edited = mission_tasks.remove_task(active, task_id)
            if edited is None:
                raise ValueError(f"task {task_id} is not in the active plan")
            # Losing work never widens what the mission claims to do, so the
            # text stands unchanged.
            return edited, []

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

        edited = mission_tasks.insert_task(self._without_completed(active), task)
        if edited is None:
            raise ValueError(f"could not graft task {task_id} into the active plan")

        # The mission text has to grow with the work. The replanning LLM sees
        # the whole plan XML, <Mission> element included, as its context for
        # what the mission still needs -- without extending it here, a later
        # replan has no way to know the absorbed objective is part of the
        # mission rather than something safe to drop.
        clause = _mission_clause(task)
        if clause:
            edited = _extend_mission_text(edited, clause)
        return edited, dropped

    def _deliver(self, plan: str) -> None:
        """Publish an accepted plan, or hold it until the robot can take it."""
        with self._lock:
            busy = self._executor_busy
            if busy:
                self._held_plan = plan
                self._held_since = time.monotonic()
        if busy:
            self.get_logger().info(
                "ACCEPTED candidate — held until the running mission ends"
            )
            return
        self._publish_plan(plan)

    def _publish_plan(self, plan: str) -> None:
        """Put a plan on /mission/xml and note that the robot is now running it."""
        with self._lock:
            # Armed here rather than at accept time: the echo guard exists to
            # recognise our own message coming back, and a held plan has not
            # been sent yet.
            self._published_xml = plan
            self._held_plan = None
            self._executor_busy = True
        out = String()
        out.data = plan
        self.mission_pub.publish(out)
        self.get_logger().info("ACCEPTED candidate — published to /mission/xml")

    def _on_mission_ended(self) -> None:
        """bt_runner finished a tree. Release whatever was waiting for it.

        Pruned *now*, against the objectives finished during the mission that
        just ended -- which is the whole point of having waited. The same plan
        published when it was written would still name trees this robot
        completed while it was held, and the executor rebuilds from the root.

        Pruning can remove everything the held plan had -- it was written for
        a robot with some tree left to visit, and every one of those finished
        while it waited. There is no valid document for "nothing left to do"
        (a control node needs a child; see ``mission_tasks._prune_empty``), so
        this does not try to write one. ``_executor_busy`` is already False
        above, which is the only thing a plan with no work would have been
        for -- publishing an empty one bought nothing and cost bt_runner a
        schema error it could not recover from.
        """
        with self._lock:
            self._executor_busy = False
            plan = self._held_plan
            self._held_plan = None
        if plan is None:
            return
        pruned = self._without_completed(plan)
        if not mission_tasks.tasks_in(pruned):
            self.get_logger().info(
                "held plan is empty after pruning finished work -- nothing "
                "left to publish"
            )
            return
        self._publish_plan(pruned)

    def _release_stale_hold(self) -> None:
        """Send a plan that has been held too long, and give up on waiting.

        The signal this waits for is one message from one node; nothing here
        can prove it will arrive. Failing open costs only the freshness the
        holding was for -- a plan published while the robot is busy sits in
        bt_runner's pending slot exactly as every plan did before any of this,
        so the worst case is the behaviour this replaced, never a robot idle
        with work it was never handed.

        Same pruning as ``_on_mission_ended``, and the same reason it can come
        back with nothing: whatever the held plan was for may have finished by
        other means while this waited. There is still no valid document for
        that, so this still does not publish one -- discarding a stale hold
        that turned out to be empty is strictly better than handing bt_runner
        XML it cannot run.
        """
        with self._lock:
            plan = self._held_plan
            waited = time.monotonic() - self._held_since
        if plan is None or waited < HOLD_TIMEOUT_SEC:
            return
        pruned = self._without_completed(plan)
        if not mission_tasks.tasks_in(pruned):
            self.get_logger().warn(
                f"held plan for {waited:.0f}s with no mission-end from "
                "bt_runner, and it is empty after pruning -- discarding it "
                "rather than publishing nothing runnable"
            )
            with self._lock:
                self._held_plan = None
            return
        self.get_logger().warn(
            f"held a plan for {waited:.0f}s with no mission-end from bt_runner "
            "— publishing it anyway"
        )
        self._publish_plan(pruned)

    def _without_completed(self, xml: str) -> str:
        """``xml`` with work this robot no longer owns taken out -- sampled by
        it, or handed to a peer.

        bt_runner has no resume point -- it builds a fresh tree from every plan
        published to /mission/xml and ticks it from the root -- so a task
        grafted onto the plan as written sends the robot back through its whole
        finished aisle before it reaches the work it just won. That is not a
        hypothetical: a robot that had sampled trees 58 and 64, then won tree 20
        at auction, drove back to 58 and 64 and re-sampled both before setting
        off for 20.

        Transferred trees are the same shape of problem from the other
        direction: a plan held while a robot was mid-mission can be published
        after that robot has since given the tree away, and pruning only what
        it *sampled* would send it right back to a tree a peer is already
        working. ``transferred_objectives`` is that ledger -- the same one
        ``_check_objective_preserved`` reads to keep a justified drop from
        being mistaken for an abandoned one -- so a tree gone for either reason
        comes out here the same way.

        The planner already prunes for exactly this reason before it shows the
        model a plan. The arbiter is the *other* writer of /mission/xml and was
        the one path that did not, which is why absorbing work re-ran it.

        Unlike the planner's copy of this step there is no bail-out when
        everything prunes away: an empty plan is the right answer for a robot
        whose own aisle is finished, because a task is about to be appended to
        it. Both ledgers this subtracts are the same ones
        ``_check_objective_preserved`` subtracts, so a plan short of these
        trees is not read as dropping them.
        """
        with self._lock:
            stale = set(self.completed_objectives) | set(self.transferred_objectives)
            orchard_map = self.orchard
        # No orchard means no way to tell which aisle heads the *remaining*
        # trees still need, and prune_completed drops every one it cannot
        # justify -- which would leave the surviving objectives with no route
        # into their aisle. Re-driving a stale tree is waste; publishing a
        # plan that cannot reach the trees it kept is worse.
        if not stale or orchard_map is None:
            return xml
        try:
            doc = etree.fromstring(xml.encode("utf-8"))
        except etree.XMLSyntaxError:
            return xml
        pruned = ontology.prune_completed(doc, stale, orchard_map)
        self.get_logger().info(
            f"pruned {sorted(stale)} from the plan before grafting — already "
            "sampled or handed to a peer this session"
        )
        return etree.tostring(pruned, encoding="unicode")

    def _decide(self, candidate: str) -> Tuple[bool, str]:
        """Evaluate a candidate, act on the verdict, and report it.

        Returns (accepted, reason).
        """
        verdict, reason, abort = self._evaluate(candidate)

        if verdict:
            # Commit state before publishing, so the echo guard is already armed
            # when our own message comes back on /mission/xml.
            with self._lock:
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
            self._deliver(candidate)

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
        return verdict, reason

    def _on_bt_status(self, msg: String):
        """Both halves of /bt/status_change, for two different questions.

        FAILURE tells the arbiter WHY the planner is editing, which is what
        decides whether a drop is permanently justified.

        SUCCESS tells it what this robot has already DONE, which it previously
        had no way of knowing at all — and that gap is a bug the fleet ran into
        live. The planner deliberately prunes finished work out of the XML it
        shows the model (the executor has no resume point), so the candidate
        that comes back is missing those objectives on purpose. With no record
        of completion, every one of them read as an unjustified drop: amiga3
        finished trees 24 and 68, won tree 20 at auction, and had its correct
        absorb plan rejected for "dropping" the two trees it had just sampled.

        Read off the same binding the planner uses, so the two agree by
        construction rather than by both being right separately.
        """
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(event, dict):
            return

        # Either outcome ends the mission, and the FAILURE branch below returns
        # early, so this has to come first.
        if str(event.get("node") or "") == "<tree>":
            self._on_mission_ended()

        status = str(event.get("status") or "FAILURE").upper()
        if status == "FAILURE":
            with self._lock:
                self._last_failure_reason = str(event.get("reason", ""))
            return
        if status != "SUCCESS":
            return

        name = event.get("node")
        if not name or name == "<tree>":
            return
        with self._lock:
            xml = self.active_mission_xml
            orchard_map = self.orchard
        if not xml:
            return
        try:
            doc = etree.fromstring(xml.encode("utf-8"))
        except etree.XMLSyntaxError:
            return
        tree_id = ontology.objective_trees(doc, orchard_map).get(name)
        if tree_id is None:
            return
        with self._lock:
            self.completed_objectives.add(tree_id)

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

        # 4. Semantic: objective preservation / viability. This is the only
        # check that can abort, so it is also the only thing that ends the
        # local repair loop.
        if self.objective_gating:
            ok, reason, abort = self._check_objective_preserved(doc)
            if not ok:
                return False, reason, abort

        with self._lock:
            active = self.active_mission_xml

        # 5. Edit-size limit (only when we know the active plan)
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

        # 6. Rate limit — skipped while in a reject→retry cycle
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
          model-determined viability budget.

        Both gates are about work that will NOT happen, so both are computed
        against the objectives still outstanding — the mission minus what this
        robot has already finished, and minus what a peer took at auction. An
        objective absent from a candidate because it is done is not a drop by
        any reading: the work happened. Counting it as one made a finished
        robot unable to produce an acceptable plan at all, since every edit it
        made "dropped" its whole completed history. An objective absent because
        a peer won it is not a drop either — the work is scheduled, just not
        here — and counting it aborted a mission over trees that were being
        sampled at that moment."""
        with self._lock:
            original = self.original_objectives
            reason = self._last_failure_reason.lower()
            already_justified = set(self.justified_drops)
            completed = set(self.completed_objectives)
            transferred = set(self.transferred_objectives)
            budget = self.max_droppable
        if not original:
            return True, "", False
        if budget is None:
            budget = DEFAULT_VIABILITY_BUDGET  # call not back yet

        candidate_objs = self._objective_tree_ids(doc)
        outstanding = original - completed - transferred
        dropped = outstanding - candidate_objs
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
    """Append ``clause`` to the plan's <Mission> text."""
    doc = etree.fromstring(mission_xml.encode("utf-8"))
    element = doc.find("Mission")
    if element is None:
        return mission_xml
    current = (element.text or "").strip()
    element.text = f"{current}; {clause}" if current else clause
    return etree.tostring(doc, encoding="unicode")


def _mission_clause(task) -> str:
    """How an absorbed task is described in the plan's <Mission> text.

    Short and mechanical on purpose, and generated from the wire fields rather
    than written by a model -- this is the line the replanning LLM reads to
    know the objective is part of the mission.
    """
    if task.target.kind != TargetKind.TREE:
        return ""
    caps = {Capability(c) for c in capabilities_in(int(task.capabilities))}
    tree = int(task.target.a)
    if Capability.SAMPLE_LEAF in caps:
        return f"sample leaves from tree {tree}"
    return f"visit tree {tree}"


def main():
    # Multithreaded so a slow /mission/verify_replan call from the coordinator
    # cannot stall the /mission/candidate_xml subscription this same node
    # depends on.
    spin.run(ArbiterNode, multithreaded=True)


if __name__ == "__main__":
    main()
