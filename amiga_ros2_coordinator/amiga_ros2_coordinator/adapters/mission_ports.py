#!/usr/bin/env python3
"""The MissionInterface, answered from the behaviour tree's own plan.

The read half of the coordinator's mission port. ``mission_bridge_node`` in
``amiga_ros2_agents`` watches ``/mission/xml`` and publishes a latched summary
of it; this subscribes to that summary and answers the three questions the
bidder needs. Neither side teaches the coordinator lxml, which is
``mission_tasks``' rule -- *the coordinator never parses XML* -- kept intact.

    mission/coordination_state ──▶ [this] ──▶ can_absorb / current_task_id /
                                              battery_percent

**Why a cached snapshot and not a service.** All three questions are asked from
``_assess``, under the coordinator's lock -- the lock ``tick`` and
``on_message`` need. A service call there would stop auctions and heartbeats for
the round trip, and a robot that stops answering is one the fleet writes off as
dead. So the answers come out of a variable, refreshed by a subscription.

**Why the mutations do not touch the plan.** ``absorb``, ``release`` and
``mark_transferred`` only record what happened here. The plan edit that
implements them is the *replanner's*: ``VerifyingReplanner`` sends the same
ownership change to the arbiter over ``VerifyReplan``, and the arbiter -- the
sole writer of ``/mission/xml`` -- applies the edit to the plan it already
holds. Editing here as well would graft the same subtree twice, and would do it
from a copy of the mission the arbiter may since have replaced. One event, one
writer.

That leaves these three as bookkeeping, which is all their signatures ever
promised: every one of them returns None.
"""

import json
from threading import Lock
from typing import Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from amiga_ros2_comms.codec import TASK_NONE

from ..vocabulary.model import Task

#: Must match ``mission_bridge_node.LATCHED``, or the snapshot never arrives:
#: a TRANSIENT_LOCAL publisher and a VOLATILE subscriber do not match, and the
#: symptom is a coordinator that silently never bids.
LATCHED = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class BehaviorTreeMission:
    """The mission port, backed by the mission bridge's snapshot."""

    def __init__(
        self,
        node: Node,
        state_topic: str = "mission/coordination_state",
    ):
        self._logger = node.get_logger()
        self._lock = Lock()
        self._state: dict = {}
        self._warned_silent = False

        #: (event, task_id) in order, for tests and for asking a running
        #: system what it thinks it is holding.
        self.history: "list[tuple[str, int]]" = []
        self._state_sub = node.create_subscription(
            String, state_topic, self._on_state, LATCHED
        )

    # ------------------------------------------------------------------
    # MissionInterface -- the questions
    # ------------------------------------------------------------------

    def can_absorb(self, task: Task) -> bool:
        """Whether the mission could take this on at all.

        False until the bridge has said anything, which is the safe direction:
        a robot that bids because its mission node never answered is a robot
        that takes work it cannot do -- the same reasoning ``UnavailableMission``
        is built on.
        """
        state = self._snapshot()
        if state is None:
            self._warn_once_silent()
            return False
        if int(task.task_id) in set(state.get("task_ids") or []):
            # Already in our plan. Bidding for it would be bidding against
            # ourselves, and absorbing it twice would graft the subtree twice.
            return False
        return bool(state.get("can_absorb", False))

    def current_task_id(self) -> int:
        state = self._snapshot()
        return int((state or {}).get("current_task_id", TASK_NONE))

    def battery_percent(self) -> int:
        state = self._snapshot()
        return max(0, min(100, int((state or {}).get("battery_percent", 0))))

    # ------------------------------------------------------------------
    # MissionInterface -- the mutations
    # ------------------------------------------------------------------

    def absorb(self, task: Task) -> None:
        """Won at auction. Recorded here; the arbiter makes the plan edit."""
        self._note("absorbed", task)

    def release(self, task: Task) -> None:
        """Handed back -- absorbed then rejected, or given up on locally."""
        self._note("released", task)

    def mark_transferred(self, task: Task) -> None:
        """Another robot acknowledged owning it. Never on a mere GRANT."""
        self._note("transferred", task)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _note(self, what: str, task: Task) -> None:
        """Log the ownership change and remember it.

        Deliberately not a publish. The plan edit belongs to the replanner, and
        two writers for one event is how a subtree gets grafted twice.
        """
        self.history.append((what, int(task.task_id)))
        self._logger.info(f"mission: task {task.task_id} {what}")

    def _on_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            self._logger.error("unusable mission state snapshot; ignoring it")
            return
        if not isinstance(state, dict):
            self._logger.error(
                f"mission state must be a JSON object, got {type(state).__name__}"
            )
            return
        with self._lock:
            self._state = state
            self._warned_silent = False

    def _snapshot(self) -> Optional[dict]:
        with self._lock:
            return dict(self._state) if self._state else None

    def _warn_once_silent(self) -> None:
        with self._lock:
            if self._warned_silent:
                return
            self._warned_silent = True
        self._logger.warn(
            "no mission state has arrived yet, so this robot will not bid. "
            "Is mission_bridge running, and is its state_topic the one this "
            "coordinator subscribes to?"
        )
