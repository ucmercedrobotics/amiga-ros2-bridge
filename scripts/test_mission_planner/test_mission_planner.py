"""
Usage (run inside the container with ROS2 sourced):
  # Terminal 1: ros2 run amiga_ros2_mission_planner mission_planner
  # Terminal 2: python3 scripts/test_mission_planner.py

Steps:
  1. Publishes a sample mission XML to /mission/xml
  2. Waits 2 s for the node to register it
  3. Publishes a mock BT failure event to /bt/status_change
  4. Listens on /mission/xml for the edited plan (300 s timeout)
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SAMPLE_XML = """\
<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">
  <Mission>sample leaves from trees 10 and 60</Mission>
  <BehaviorTree ID="Sample_Leaves_Trees_10_60">
    <Sequence>
      <MoveToTreeID name="Visit_Tree_10" action_name="follow_tree_id_waypoint" id="10" approach_tree="true"/>
      <SampleLeaf name="Sample_Leaves_Tree_10" action_name="segment_leaves"/>
      <MoveToTreeID name="Exit_To_Top_Headland" action_name="follow_tree_id_waypoint" id="2" approach_tree="false"/>
      <MoveToTreeID name="Visit_Tree_60" action_name="follow_tree_id_waypoint" id="60" approach_tree="true"/>
      <SampleLeaf name="Sample_Leaves_Tree_60" action_name="segment_leaves"/>
    </Sequence>
  </BehaviorTree>
</root>"""


MOCK_FAILURE = {
    "node": "Visit_Tree_60",
    "status": "FAILURE",
    "timestamp_ms": int(time.time() * 1000),
    "reason": "action server returned ABORTED — obstacle detected near tree 60",
}

MOCK_FAILURE_2 = {
    "node": "Visit_Tree_60",
    "status": "FAILURE",
    "timestamp_ms": 0,  # set at publish time
    "reason": (
        "action server returned ABORTED — still cannot reach tree 60 after "
        "3 retry attempts; obstacle appears permanent (fallen tree across row)"
    ),
}
class Tester(Node):
    def __init__(self):
        super().__init__("mission_planner_tester")
        self.xml_pub = self.create_publisher(String, "/mission/xml", 10)
        self.bt_pub = self.create_publisher(String, "/bt/status_change", 10)
        self.received_edits = []                        # CHANGED: list instead of single value

        self.create_subscription(String, "/mission/xml", self._on_xml, 10)

    def _on_xml(self, msg: String):                     # CHANGED: collect every distinct edit
        if msg.data == SAMPLE_XML:
            return
        if msg.data in self.received_edits:
            return
        self.received_edits.append(msg.data)
        self.get_logger().info(f"Received edited XML #{len(self.received_edits)}!")

    def _wait_for_edit(self, count: int, timeout_sec: float = 300.0) -> bool:   # NEW helper
        deadline = time.time() + timeout_sec
        while time.time() < deadline and len(self.received_edits) < count:
            rclpy.spin_once(self, timeout_sec=1.0)
        return len(self.received_edits) >= count

    def run(self):
        self.get_logger().info("Publishing sample mission XML…")
        m = String()
        m.data = SAMPLE_XML
        self.xml_pub.publish(m)

        time.sleep(2.0)

        # --- Failure 1: obstacle at tree 60 ---
        self.get_logger().info("Publishing mock BT failure 1…")
        MOCK_FAILURE["timestamp_ms"] = int(time.time() * 1000)
        f = String()
        f.data = json.dumps(MOCK_FAILURE)
        self.bt_pub.publish(f)

        self.get_logger().info("Waiting for edited plan 1 (up to 300 s)…")
        if not self._wait_for_edit(1):
            print("TIMEOUT — no edited XML for failure 1", file=sys.stderr)
            sys.exit(1)

        time.sleep(2.0)   # let the planner ingest its own edit #1

        # --- Failure 2: retries exhausted at tree 60 ---
        self.get_logger().info("Publishing mock BT failure 2 (retries exhausted)…")
        MOCK_FAILURE_2["timestamp_ms"] = int(time.time() * 1000)
        f2 = String()
        f2.data = json.dumps(MOCK_FAILURE_2)
        self.bt_pub.publish(f2)

        self.get_logger().info("Waiting for edited plan 2 (up to 300 s)…")
        if not self._wait_for_edit(2):
            print("TIMEOUT — no edited XML for failure 2", file=sys.stderr)
            sys.exit(1)
        # ========================================

        for i, xml in enumerate(self.received_edits, 1):
            print(f"\n=== Edited mission XML #{i} ===")
            print(xml)
            print("=" * 30)

def main():
    rclpy.init()
    tester = Tester()
    try:
        tester.run()
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()