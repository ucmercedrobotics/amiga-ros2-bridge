#!/usr/bin/env python3
"""Fire one escalation at a robot, by hand.

    ros2 run amiga_ros2_coordinator escalate --robot amiga2 --task 42 --tree 60

This is the owner role's entry point, poked with a stick. In a running system
``/coordination/infeasible`` is published by the triage agent when the planner
and the arbiter have both given up; this publishes the same message, so a fleet
can be made to do something without waiting for a behaviour tree to genuinely
fail. That makes it the manual injection point for any bench or simulation run,
not only the scripted demo.

What happens next is the whole coordination stack: the robot interprets the
anomaly, announces the task, the fleet bids, the best bid wins, the winner
absorbs it and both robots replan.

The ``--note`` text becomes the payload's ``detail`` field, which is what
``anomaly_context`` carries to ``interpret_anomaly``. With the stub interpreter
that text *is* the note the fleet hears; with the triage agent it is the context
the model authors one from. Either way the sentence an operator writes here is
the sentence that reaches the decision.

Note the default target is a **tree**. ``mission_tasks.synthesize`` rebuilds a
subtree from an announcement for tree-targeted sampling work and refuses GPS, so
a GPS task can be auctioned and won and then not executed. ``--gps`` is offered
because the auction itself handles it and it is worth being able to exercise
that, but it warns.
"""

import argparse
import json
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from amiga_ros2_comms.codec import (
    CAPABILITY_BY_ELEMENT,
    PRIORITY_MAX,
    TASK_ID_MAX,
    Target,
    cap_mask,
)

#: What a task needs to be worth trading: get to the tree, then work on it.
#: The pairing examples/sample_leafs.xml uses and the one synthesize rebuilds.
DEFAULT_CAPABILITIES = "MoveToTreeID,SampleLeaf"

#: The Gazebo orchard's navsat origin, from worlds/orchard_nbv.sdf. Only used
#: for --gps, and only so the flag has a default that is somewhere in the world
#: rather than in the Gulf of Guinea.
WORLD_ORIGIN = (37.3611, -120.4322)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="escalate",
        description="Tell one robot it cannot finish a task, and watch the "
        "fleet decide what to do about it.",
    )
    parser.add_argument(
        "--robot",
        default="",
        help="Namespace of the robot to escalate at, e.g. amiga2. Empty -- the "
        "default -- targets the unnamespaced robot, which is robot 1 in "
        "sim_bringup.launch.py.",
    )
    parser.add_argument(
        "--task", type=int, default=42, help=f"Task id, 1..{TASK_ID_MAX}."
    )

    where = parser.add_mutually_exclusive_group()
    where.add_argument(
        "--tree", type=int, help="Tree index the work is at (default 60)."
    )
    where.add_argument("--aisle", type=int, help="Aisle index the work is at.")
    where.add_argument(
        "--gps",
        metavar="LAT,LON",
        help="Coordinates the work is at. The auction handles these; the "
        "behaviour-tree rebuild currently does not -- see the module docstring.",
    )

    parser.add_argument(
        "--capabilities",
        default=DEFAULT_CAPABILITIES,
        help=f"Comma-separated behaviour-tree action names. Default: "
        f"{DEFAULT_CAPABILITIES}. Known: {', '.join(sorted(CAPABILITY_BY_ELEMENT))}",
    )
    parser.add_argument("--priority", type=int, default=100, help=f"0..{PRIORITY_MAX}.")
    parser.add_argument(
        "--note",
        default="",
        help="What a robot taking this on should know. Becomes the escalation's "
        "`detail`, and from there the note broadcast alongside the announcement.",
    )
    parser.add_argument(
        "--topic",
        default="",
        help="Override the escalation topic outright. Default is "
        "/<robot>/coordination/infeasible.",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=1.0,
        help="Seconds to wait for the subscription to match before publishing. "
        "A publish nobody is subscribed to yet goes nowhere.",
    )
    return parser


def target_from(args) -> Target:
    if args.aisle is not None:
        return Target.aisle(args.aisle)
    if args.gps:
        try:
            lat, lon = (float(part) for part in args.gps.split(","))
        except ValueError:
            raise SystemExit(f"--gps wants LAT,LON; got {args.gps!r}")
        return Target.gps(lat, lon)
    return Target.tree(60 if args.tree is None else args.tree)


def capabilities_from(text: str) -> int:
    names = [name.strip() for name in text.split(",") if name.strip()]
    if not names:
        raise SystemExit(
            "--capabilities cannot be empty; a task with no actions "
            "describes no work"
        )
    unknown = [name for name in names if name not in CAPABILITY_BY_ELEMENT]
    if unknown:
        raise SystemExit(
            f"unknown capability {unknown}; known: "
            f"{', '.join(sorted(CAPABILITY_BY_ELEMENT))}"
        )
    return cap_mask(*(CAPABILITY_BY_ELEMENT[name] for name in names))


def payload_for(args) -> dict:
    """The escalation, in the flat shape ``_on_infeasible`` parses.

    Every field the coordinator's ``_task_from_payload`` needs, and no more: a
    task assembled half from here and half from a default would be an
    announcement that means something nobody decided.
    """
    target = target_from(args)
    return {
        "task_id": int(args.task),
        "capabilities": capabilities_from(args.capabilities),
        "target_kind": int(target.kind),
        "target_a": int(target.a),
        "target_b": int(target.b),
        "priority": int(args.priority),
        "detail": args.note or "local recovery exhausted",
    }


def topic_for(args) -> str:
    if args.topic:
        return args.topic
    robot = args.robot.strip("/")
    return f"/{robot}/coordination/infeasible" if robot else "/coordination/infeasible"


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    payload = payload_for(args)
    topic = topic_for(args)

    if args.gps:
        print(
            "warning: a GPS-targeted task can be auctioned and won, but "
            "mission_tasks.synthesize cannot rebuild its subtree, so the winner "
            "will refuse to absorb it. Use --tree to see a transfer complete.",
            file=sys.stderr,
        )

    rclpy.init()
    node = Node("escalate")
    publisher = node.create_publisher(String, topic, 10)

    deadline = time.time() + max(0.0, args.settle)
    while time.time() < deadline and publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if publisher.get_subscription_count() == 0:
        print(
            f"warning: nothing is subscribed to {topic}. Is the coordinator up, "
            f"and is --robot right?",
            file=sys.stderr,
        )

    publisher.publish(String(data=json.dumps(payload)))
    # Spin briefly after publishing: rclpy hands the message to the middleware
    # asynchronously, and destroying the node immediately can drop it.
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.02)

    print(f"escalated task {payload['task_id']} on {topic}")
    print(f"  {json.dumps(payload, indent=2)}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
