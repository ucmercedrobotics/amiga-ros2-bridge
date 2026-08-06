#!/usr/bin/env python3
"""
mission_bridge_node.py

The coordinator's view of the mission, and the mission's view of the coordinator.

``mission_tasks``' module docstring is explicit that **the coordinator never
parses XML** -- it learns about tasks over the radio and over
``/coordination/infeasible``, in the flat vocabulary the codec defines. That is
a good rule and this node is what makes it affordable: the coordinator has a
``MissionInterface`` port with five questions on it, every one of which is
really a question about the behaviour tree's plan, and something has to answer
them without teaching the coordinator lxml.

    /mission/xml ──▶ [this] ──▶ mission/coordination_state  (latched JSON)

**Read-only, on purpose.** This does not edit the plan. When a coordinator wins
or sheds a task it tells the arbiter directly over ``VerifyReplan``, and the
arbiter -- the sole writer of ``/mission/xml`` -- applies the edit to the plan it
already holds. A second writer here would graft the same subtree twice, from a
copy of the mission the arbiter may since have replaced. What this node does is
answer questions, and the answers refresh when the arbiter's edit comes back
around on ``/mission/xml``.

The plan is the only input. ``/bt/status_change`` is deliberately not one:
FaultReporter publishes it on failures and on the tree's final outcome, never on
an ordinary node transition, so it cannot say which task is running now -- see
``_current_task_id`` for what is used instead and why that is enough.

**Why a topic each way rather than a service.** ``can_absorb``,
``current_task_id`` and ``battery_percent`` are called from ``_assess``, under
the coordinator's lock -- the lock ``tick`` and ``on_message`` need. A service
call there would block auctions and heartbeats for the round trip. So the state
goes out as a latched snapshot the port reads out of a variable, and the three
mutations do not come here at all -- see below.

Topic names are parameters defaulting to *relative*, so a namespaced instance is
genuinely per-robot. Note that the rest of this package hardcodes absolute names
(``/mission/xml``, ``/bt/status_change``, ``/world_state``), so a fleet running
one agent stack per robot needs the same treatment there before N of them can
coexist. In the single-robot case a relative name resolves to the identical
absolute one, so nothing changes today.
"""

import json
from threading import Lock

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from amiga_ros2_comms.codec import TASK_NONE

from ..mission import mission_tasks
from ..runtime import spin
from ..runtime.status import StatusPublisher

#: Latched, so a coordinator that starts after this node immediately learns the
#: mission state instead of bidding on the assumption of an empty plan. Same
#: reasoning as the coordinator's own preemption flag.
LATCHED = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MissionBridgeNode(Node):
    """Answers the coordinator's mission questions from the behaviour tree's plan."""

    def __init__(self, **node_kwargs):
        super().__init__("mission_bridge", **node_kwargs)

        self.declare_parameter("mission_topic", "mission/xml")
        self.declare_parameter("state_topic", "mission/coordination_state")
        self.declare_parameter("max_tasks", 8)
        # No battery on a simulated robot and no farm-ng battery topic wired
        # here yet. A parameter rather than a fabricated reading, so it is
        # obvious in `ros2 param get` that this number is set and not measured
        # -- and so a scenario can hand one robot a flat battery and watch the
        # fitness ordering change.
        self.declare_parameter("battery_percent", 100)

        self._lock = Lock()
        self._mission: str = ""
        self._max_tasks = int(self.get_parameter("max_tasks").value)

        self._state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), LATCHED
        )
        self.create_subscription(
            String, str(self.get_parameter("mission_topic").value), self._on_mission, 10
        )
        self._status = StatusPublisher(self)
        self._publish_state()
        self.get_logger().info(
            f"mission_bridge ready: max_tasks={self._max_tasks}, "
            f"battery={self._battery()}%"
        )

    # ------------------------------------------------------------------
    # Reading the mission
    # ------------------------------------------------------------------

    def _on_mission(self, msg: String) -> None:
        with self._lock:
            self._mission = msg.data or ""
        self._publish_state()

    def _publish_state(self) -> None:
        """Republish the snapshot the coordinator's mission port reads."""
        state = self._state()
        self._state_pub.publish(String(data=json.dumps(state)))
        self._status.publish(state)

    def _state(self) -> dict:
        with self._lock:
            mission = self._mission
        tasks = mission_tasks.tasks_in(mission)
        return {
            "current_task_id": self._current_task_id(tasks),
            "battery_percent": self._battery(),
            "task_count": len(tasks),
            "task_ids": [int(task.task_id) for task in tasks],
            # Capacity only. Whether a *particular* task fits is asked of the
            # arbiter, by offering it the candidate -- which is the same
            # question the arbiter answers for every other edit, so asking it
            # twice with two different answers is what this avoids.
            "can_absorb": len(tasks) < self._max_tasks,
        }

    def _current_task_id(self, tasks) -> int:
        """The task the robot is working on, as closely as this can be known.

        The *next* task in plan order, which for the Sequence-shaped missions
        this fleet writes is the one running. It is not read off the tree:
        ``/bt/status_change`` fires on failures and on the tree's final outcome
        (see FaultReporter), not on every node transition, so there is no topic
        that says which node is ticking right now.

        Being approximate is affordable because of what the value is *for*: it
        rides in a HEARTBEAT and it sets ``busy`` in ``default_fitness``, which
        is a 0.35 cost penalty rather than a veto. "Does this robot have work"
        is the question being asked, and that this answers exactly.
        """
        return int(tasks[0].task_id) if tasks else TASK_NONE

    def _battery(self) -> int:
        return max(0, min(100, int(self.get_parameter("battery_percent").value)))


def main(args=None):
    spin.run(MissionBridgeNode)


if __name__ == "__main__":
    main()
