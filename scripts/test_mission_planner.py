"""
End-to-end smoke test for the Mission Planner node.

Usage (run inside the container with ROS2 sourced):
  # Terminal 1: ros2 run amiga_ros2_mission_planner mission_planner
  # Terminal 2: python3 scripts/test_mission_planner.py

Steps:
  1. Publishes a sample mission XML to /mission/xml
  2. Waits 2 s for the node to register it
  3. Publishes a mock BT failure event to /bt/status_change
  4. Listens on /mission/xml for the edited plan (90 s timeout)
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


class Tester(Node):
    def __init__(self):
        super().__init__("mission_planner_tester")
        self.xml_pub = self.create_publisher(String, "/mission/xml", 10)
        self.bt_pub = self.create_publisher(String, "/bt/status_change", 10)
        self.received_edit = None

        self.create_subscription(String, "/mission/xml", self._on_xml, 10)

    def _on_xml(self, msg: String):
        if msg.data == SAMPLE_XML:
            return
        self.received_edit = msg.data
        self.get_logger().info("Received edited XML!")

    def run(self):
        self.get_logger().info("Publishing sample mission XML…")
        m = String()
        m.data = SAMPLE_XML
        self.xml_pub.publish(m)

        time.sleep(2.0)

        self.get_logger().info("Publishing mock BT failure…")
        f = String()
        f.data = json.dumps(MOCK_FAILURE)
        self.bt_pub.publish(f)

        self.get_logger().info("Waiting for edited plan (up to 90 s)…")
        deadline = time.time() + 90
        while time.time() < deadline and self.received_edit is None:
            rclpy.spin_once(self, timeout_sec=1.0)

        if self.received_edit:
            print("\n=== Edited mission XML ===")
            print(self.received_edit)
            print("=========================\n")
        else:
            print("TIMEOUT — no edited XML received within 90 s", file=sys.stderr)
            sys.exit(1)


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