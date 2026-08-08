#!/usr/bin/env python3
"""The coordinator with its ports actually connected to a robot.

``node.py``'s ``main()`` starts a coordinator whose nav and mission ports
decline every question, on purpose: a robot that bids because nothing answered
takes work it cannot do, so refusing is the right *default*. This is the other
entry point, the one that answers -- three objects passed to a constructor and
nothing else changed, which is the property the port design exists for.

    CoordinatorNode(
        nav       = GpsNav(...),                # gps/pvt + the orchard model
        mission   = BehaviorTreeMission(...),   # the mission bridge's snapshot
        replanner = VerifyingReplanner(...),    # the arbiter, over VerifyReplan
    )

**Why there is a third node.** The ports own subscriptions, so they need a Node
to create them on; the coordinator needs the ports at construction time. Rather
than resolve that by letting ports be installed after the fact -- which would
make "the ports are fixed once the state machine is running" stop being true --
the subscriptions live on a small node of their own. The coordinator is then
constructed exactly as any test constructs it, with three finished objects, and
not one line of ``coordinator.py`` or ``node.py`` knows this file exists.

Named ``sim`` for where it is exercised rather than for what it fakes: all three
ports read real state off real topics. What makes it unsuitable as the
production default is narrower and worth being precise about -- the battery
number is a parameter rather than a measurement, and travel is costed as a
straight line because the fleet has no prior map to plan against. Both are
documented where they live.

Four nodes, one multi-threaded executor -- multi-threaded for the same reason
``node.py``'s is: the reliability layer resolves a GRANT's future from its own
retransmit tick, and the coordinator's outcome handler runs there.
"""

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from amiga_ros2_comms.reliability.node import ReliabilityNode

from ..adapters.mission_ports import BehaviorTreeMission
from ..adapters.nav_ports import (
    DEFAULT_FIX_TTL_SEC,
    DEFAULT_ORCHARD_TOPIC,
    DEFAULT_SPEED_MPS,
    GpsNav,
)
from .coordinator_node import CoordinatorNode
from ..adapters.replanner_client import VerifyingReplanner


class DeferredCallback:
    """A callable slot filled in after construction.

    There is a genuine cycle here: the replanner needs somewhere to report a
    rejection, that somewhere is ``report_infeasible`` on the coordinator, and
    the coordinator cannot be built until the replanner exists. This breaks it
    without anyone reaching into anyone else's attributes, and without making
    the ports mutable after the state machine is running.

    Calls before the slot is filled are dropped rather than raising: the window
    is the few milliseconds between constructing the replanner and constructing
    the coordinator, and nothing can have been verified yet in it.
    """

    def __init__(self, name: str = ""):
        self.name = name
        self.target = None

    def __call__(self, *args, **kwargs):
        if self.target is None:
            return None
        return self.target(*args, **kwargs)


class RobotPorts(Node):
    """Holds the subscriptions the nav and mission ports read from.

    Its own node rather than the coordinator's so that the coordinator can be
    constructed with the ports already finished, which is how every acceptance
    test constructs it too.
    """

    def __init__(self, **node_kwargs):
        super().__init__("coordinator_ports", **node_kwargs)

        self.declare_parameter("nominal_speed_mps", DEFAULT_SPEED_MPS)
        self.declare_parameter("gps_fix_ttl_sec", DEFAULT_FIX_TTL_SEC)
        self.declare_parameter("gps_topic", "gps/pvt")
        self.declare_parameter("orchard_topic", DEFAULT_ORCHARD_TOPIC)
        self.declare_parameter("mission_state_topic", "mission/coordination_state")
        self.declare_parameter("verify_replan_service", "/mission/verify_replan")
        self.declare_parameter("require_verifier", False)

        self.nav = GpsNav(
            self,
            speed_mps=float(self.get_parameter("nominal_speed_mps").value),
            fix_ttl_sec=float(self.get_parameter("gps_fix_ttl_sec").value),
            gps_topic=str(self.get_parameter("gps_topic").value),
            orchard_topic=str(self.get_parameter("orchard_topic").value),
        )
        self.mission = BehaviorTreeMission(
            self,
            state_topic=str(self.get_parameter("mission_state_topic").value),
        )
        #: Filled in by main() once the coordinator exists. See DeferredCallback.
        self.on_replan_rejected = DeferredCallback("on_rejected")
        self.replanner = VerifyingReplanner(
            self,
            on_rejected=self.on_replan_rejected,
            service_name=str(self.get_parameter("verify_replan_service").value),
            require_verifier=bool(self.get_parameter("require_verifier").value),
        )

        self.get_logger().info(
            f"ports wired: gps={self.get_parameter('gps_topic').value}, "
            f"orchard={self.get_parameter('orchard_topic').value}, "
            f"speed={self.get_parameter('nominal_speed_mps').value} m/s"
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    reliability = ReliabilityNode()
    ports = RobotPorts()
    coordinator = CoordinatorNode(
        reliability=reliability,
        nav=ports.nav,
        mission=ports.mission,
        replanner=ports.replanner,
    )
    # Close the loop the replanner leaves open. It reports a committed ownership
    # change without waiting for the verdict, so a rejection arrives afterwards
    # -- on the verifier's own thread, off every coordinator lock, which is what
    # report_infeasible needs. Work this robot holds and cannot legitimately do
    # is what the anomaly path is for, and it is the one route that can offer
    # the task back to the fleet.
    ports.on_replan_rejected.target = lambda task, reason: (
        coordinator.session.report_infeasible(
            task, detail=f"the arbiter rejected the mission that took it on: {reason}"
        )
    )

    executor = MultiThreadedExecutor()
    for node in (reliability, ports, coordinator):
        executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or SIGTERM from launch. Ordinary ways to stop, not errors.
        pass
    finally:
        for node in (coordinator, ports, reliability):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
