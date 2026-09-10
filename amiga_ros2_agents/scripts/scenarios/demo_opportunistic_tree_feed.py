"""Feed + detection-event script for demo_opportunistic_tree.sh.

Feeds the mission itself (sample_aisle2.bin -- trees 20, 26, same as
demo_missing_tree.sh) over the real TCP mission port, exactly like every
other demo -- NOT published directly onto /mission/xml. That matters:
tcp_demux_node decodes the TCP payload into BOTH the mission XML
(-> /mission/xml) AND the orchard layout (-> /orchard/tree_info_json, which
is what makes GetTreeInfo answer at all). Publishing the XML by hand skips
that second half and leaves the orchard service with nothing cached --
confirmed live, it just returns empty results.

The TCP send happens from THIS script, after its own /mission/xml
subscription is already up -- not via a separate `nc` call from the shell.
/mission/xml is plain (volatile) QoS, not latched, so a subscriber that
starts even slightly after the publish just never sees it; a script that
waited for "the first /mission/xml message" to mean "the mission I fed" was
tried and confirmed to race -- it sometimes caught a LATER, unrelated
message instead (an arbiter repair candidate) and mistimed everything after
it. Doing the send itself removes the race instead of tuning around it.

Once the real mission is confirmed live, this reports a tree spotted in
aisle 4 -- a real tree (58, from sample_aisle4.xml), never claimed as part
of this mission -- straight onto /bt/status_change with status DETECTION.
mission_planner_node.py's real _on_detection path takes it from there: a
real LLM call decides whether it is worth adding, and if the arbiter accepts
the edit, the accepted plan comes back on /mission/xml for real.
"""

import argparse
import json
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

# Matches fault_reporter.cpp's makeFaultPublisher: TRANSIENT_LOCAL, so a node
# that subscribes with the same durability (triage_node.py) can actually
# receive this. A plain-QoS (volatile) publisher on this topic is silently
# incompatible with that subscription -- confirmed live, the message is
# dropped with no error, only a QoS warning in triage's own log.
STATUS_CHANGE_QOS = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# A real tree in a different aisle (58, aisle 4 -- see sample_aisle4.xml),
# never part of the aisle-2 mission being fed.
DETECTED_TREE_ID = 58
DETECTED_AISLE = 4

DETECTION_EVENT = {
    "node": "ApproachTree20",
    "status": "DETECTION",
    "timestamp_ms": None,  # filled in at publish time
    "reason": (
        f"a tree in aisle {DETECTED_AISLE} (id {DETECTED_TREE_ID}) shows heavy "
        "leaf necrosis and possible blight; it is not in the current mission. "
        "If you act on this, ADD a new task for it -- do not remove, replace, "
        "or modify any existing tree already in the plan. Leaving the plan "
        "unchanged is also a valid choice if it is not worth the detour."
    ),
}

# After the mission is confirmed live, how long to let the robot actually
# get underway before reporting the sighting.
SETTLE_SEC = 15.0


class Feeder(Node):
    def __init__(self, port: int, mission_bin: bytes):
        super().__init__("opportunistic_tree_demo_feed")
        self.port = port
        self.mission_bin = mission_bin
        self.bt_pub = self.create_publisher(
            String, "/bt/status_change", STATUS_CHANGE_QOS
        )
        self.initial_mission = None
        self.received_edit = None
        self.create_subscription(String, "/mission/xml", self._on_xml, 10)

    def _on_xml(self, msg):
        if self.initial_mission is None:
            self.initial_mission = msg.data
            self.get_logger().info("Mission is live on /mission/xml")
            return
        if msg.data == self.initial_mission:
            return
        self.received_edit = msg.data
        self.get_logger().info("Received an accepted plan back on /mission/xml")

    def _feed_mission(self):
        # Give the subscription above time to actually match tcp_demux_node's
        # publisher over DDS before triggering the publish -- the same
        # discovery gap that made /bt/status_change need TRANSIENT_LOCAL.
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Feeding mission on port {self.port}...")
        with socket.create_connection(("127.0.0.1", self.port), timeout=10) as s:
            s.sendall(self.mission_bin)

    def run(self):
        self._feed_mission()

        self.get_logger().info("Waiting for the mission to appear on /mission/xml...")
        deadline = time.time() + 60
        while time.time() < deadline and self.initial_mission is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        if self.initial_mission is None:
            print(
                "Mission never appeared on /mission/xml -- not feeding a detection.",
                file=sys.stderr,
            )
            return

        self.get_logger().info(
            f"Waiting {SETTLE_SEC:.0f}s for the robot to get underway..."
        )
        deadline = time.time() + SETTLE_SEC
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info(
            f"Publishing detection event (sick tree {DETECTED_TREE_ID}, aisle {DETECTED_AISLE})..."
        )
        event = dict(DETECTION_EVENT)
        event["timestamp_ms"] = int(time.time() * 1000)
        f = String()
        f.data = json.dumps(event)
        self.bt_pub.publish(f)

        self.get_logger().info("Waiting for an accepted edit (up to 180s)...")
        deadline = time.time() + 180
        while time.time() < deadline and self.received_edit is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        if self.received_edit:
            print(
                "\n=== Accepted mission XML (after detection) ===\n"
                + self.received_edit
                + "\n===============================================\n"
            )
        else:
            print(
                "No accepted edit in 180s -- the planner may have decided not "
                "to add it, or the candidate was rejected. Check the "
                "'watch' and 'agents' windows.",
                file=sys.stderr,
            )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--mission-bin", required=True)
    args = ap.parse_args()

    with open(args.mission_bin, "rb") as fh:
        mission_bin = fh.read()

    rclpy.init()
    node = Feeder(args.port, mission_bin)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
