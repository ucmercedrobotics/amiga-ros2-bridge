"""

Scenario: robot samples every tree in a row (1-10). At tree 3 the
approach fails (e.g. obstacle / blocked approach path). The planner
should minimally edit the plan to recover — e.g. retry tree 3, skip it,
or approach_tree="false" — and republish.
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _tree_steps():
    steps = []
    for i in range(1, 11):
        steps.append(
            f'      <MoveToTreeID name="Visit_Tree_{i}" '
            f'action_name="follow_tree_id_waypoint" id="{i}" approach_tree="true"/>'
        )
        steps.append(
            f'      <SampleLeaf name="Sample_Leaves_Tree_{i}" '
            f'action_name="segment_leaves"/>'
        )
    return "\n".join(steps)


SAMPLE_XML = f"""\
<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">
  <Mission>sample leaves from ALL trees in row one, trees 1 through 10</Mission>
  <BehaviorTree ID="Sample_Row_1_Trees_1_10">
    <Sequence>
{_tree_steps()}
    </Sequence>
  </BehaviorTree>
</root>"""

MOCK_FAILURE = {
    "node": "Visit_Tree_6",
    "status": "FAILURE",
    "timestamp_ms": 0,  # set at publish time in run()
    "reason": (
        "action server returned ABORTED — no tree found at waypoint for tree ID 6: "
        "reached the row waypoint successfully but no tree is present at this "
        "position (tree appears to have been removed from the orchard); "
        "tree 6 is permanently unavailable"
    ),
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
        self.get_logger().info("Publishing 10-tree row mission XML…")
        m = String()
        m.data = SAMPLE_XML
        self.xml_pub.publish(m)

        time.sleep(2.0)

        self.get_logger().info("Publishing mock BT failure ")
        MOCK_FAILURE["timestamp_ms"] = int(time.time() * 1000)
        f = String()
        f.data = json.dumps(MOCK_FAILURE)
        self.bt_pub.publish(f)

        self.get_logger().info("Waiting for edited plan (up to 300 s)…")
        deadline = time.time() + 300
        while time.time() < deadline and self.received_edit is None:
            rclpy.spin_once(self, timeout_sec=1.0)

        if self.received_edit:
            print("\n=== Edited mission XML ===")
            print(self.received_edit)
            print("=========================\n")
        else:
            print("TIMEOUT — no edited XML received within 300 s", file=sys.stderr)
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