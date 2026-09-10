#!/usr/bin/env python3
"""Print just the agent story off /rosout, tagged by robot.

The agents run under `sim_bringup.launch.py` with `output="screen"`, so their
stdout lands in the launch window interleaved with Gazebo, Nav2 and three
robots' worth of controller chatter. On a fleet run the lines that matter --
what the camera saw, what triage decided, what the planner tried, what a peer
ended up doing with a transferred task -- scroll out of a pane in seconds.
`/rosout` carries the same lines with the node name attached, so this is that
stream with everything else dropped, plus one line Nav2 logs: which tree or
aisle a robot is actually driving to, which is what makes two robots
converging on the same target visible at all.

    python3 scripts/watch_agents.py                 # the decision story
    python3 scripts/watch_agents.py --all           # every agent line
    python3 scripts/watch_agents.py --pattern 'camera|routed'

Nothing here is demo-specific; it is useful whenever a fleet run is too noisy
to read.
"""

import argparse
import re
import signal
import sys

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile

#: Nodes worth listening to at all. `/rosout` carries the whole machine.
AGENTS = re.compile(r"triage|mission_planner|arbiter|world_state|coordinator|note")

#: The default filter: the line the camera produced, the verdict that
#: followed, the auction that verdict opened, and where the work ended up.
#: Everything else an agent logs is startup noise or per-tick detail.
#:
#: Started as decisions only (routed/interpretation/candidate/accepted/
#: rejected) and stopped there -- a fault could be watched from triage's
#: verdict through the arbiter's accept, but not to a tree actually getting
#: sampled, or to a transferred tree staying out of a later replan. Added:
#: `announced`/`granted`/`absorbed` (the auction itself, not just its
#: winner), `excluding it from` (both the "sampled/harvested" outcome line
#: and the transferred-target ownership line share this wording), `Mission
#: Planner —` (session count, i.e. cost), and `re-added` (the backstop
#: stripping a target a model restored against instruction).
STORY = re.compile(
    r"camera:|camera did not answer|routed |escalating|interpretation:|"
    r"candidate|accepted|rejected|abort|won |granted|bid |announced |"
    r"absorbed|excluding it from|Mission Planner —|re-added",
    re.IGNORECASE,
)

#: Not an agent's own line -- Nav2's waypoint_follower reports which tree or
#: aisle a robot is actually heading for, which is the only way to see two
#: robots converging on the same target. Matched separately from AGENTS/STORY
#: and shown unconditionally (even under --all's raw dump) because the node
#: also logs a "distance to goal" line on every control tick: folding it into
#: AGENTS wholesale turns a several-thousand-line run into tens of thousands.
NAV_TARGET_NODE = re.compile(r"waypoint_follower")
NAV_TARGET = re.compile(r"Navigating to (row waypoint near tree|aisle head) ")

LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--all", action="store_true", help="every agent line, not just the story"
    )
    parser.add_argument("--pattern", help="regex to keep, overrides the default")
    args = parser.parse_args()

    keep = re.compile(args.pattern, re.IGNORECASE) if args.pattern else STORY

    # This lives in a tmux pane and under `timeout`, so it is killed by SIGTERM
    # far more often than by Ctrl-C. Without this, every ordinary teardown ends
    # in a traceback across the pane that matters most.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    rclpy.init()
    node = Node("watch_agents")

    # /rosout is published TRANSIENT_LOCAL with a deep history, so a late
    # subscriber still receives what was logged before it started -- which is
    # the point when this is opened after a run has already gone wrong.
    qos = QoSProfile(
        depth=1000,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )

    def on_log(msg: Log) -> None:
        text = ANSI.sub("", msg.msg)
        is_nav_target = bool(
            NAV_TARGET_NODE.search(msg.name) and NAV_TARGET.search(text)
        )
        if not is_nav_target:
            if not AGENTS.search(msg.name):
                return
            if not args.all and not keep.search(text):
                return
        stamp = msg.stamp.sec % 86400
        level = LEVELS.get(msg.level, str(msg.level))
        # Flushed every line: this is watched live in a tmux pane and piped to
        # tee, and block buffering would hold the interesting line back until
        # the next one arrived.
        print(
            f"[{stamp // 3600:02d}:{stamp % 3600 // 60:02d}:{stamp % 60:02d}] "
            f"{level:5s} {msg.name:28s} {text}",
            flush=True,
        )

    node.create_subscription(Log, "/rosout", on_log, qos)
    print(
        f"watching /rosout for {'every agent line' if args.all else 'the decision story'}",
        file=sys.stderr,
        flush=True,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
