"""Scenario 2: robot immobilized (nav-stack failure) — replanning cannot help."""

import json, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _tree_steps(ids):
    rows = []
    for i in ids:
        rows.append(
            f'      <MoveToTreeID name="Visit_Tree_{i}" action_name="follow_tree_id_waypoint" id="{i}" approach_tree="true"/>'
        )
        rows.append(
            f'      <SampleLeaf name="Sample_Leaves_Tree_{i}" action_name="segment_leaves"/>'
        )
    return "\n".join(rows)


SAMPLE_XML = f"""\
<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">
  <Mission>sample leaves from trees 1 through 10</Mission>
  <BehaviorTree ID="Sample_Leaves_Row_1_10">
    <Sequence>
{_tree_steps(range(1, 11))}
    </Sequence>
  </BehaviorTree>
</root>"""

MOCK_FAILURE = {
    "node": "Visit_Tree_3",
    "status": "FAILURE",
    "timestamp_ms": int(time.time() * 1000),
    "reason": "the base is not moving; wheels are sunk in a dirt pit and no forward or backward motion is happening.",
}


class Tester(Node):
    def __init__(self):
        super().__init__("stuck_robot_tester")
        self.xml_pub = self.create_publisher(String, "/mission/xml", 10)
        self.bt_pub = self.create_publisher(String, "/bt/status_change", 10)
        self.abort_msg = None
        self.received_edit = None
        self.create_subscription(String, "/mission/xml", self._on_xml, 10)
        self.create_subscription(String, "/mission/abort", self._on_abort, 10)

    def _on_xml(self, msg):
        if msg.data == SAMPLE_XML:
            return
        self.received_edit = msg.data
        self.get_logger().info("Received edited XML!")

    def _on_abort(self, msg):
        self.abort_msg = msg.data
        self.get_logger().warn("ABORT signal received!")

    def run(self):
        self.get_logger().info("Publishing row 1-10 mission…")
        m = String()
        m.data = SAMPLE_XML
        self.xml_pub.publish(m)
        time.sleep(2.0)
        self.get_logger().info("Publishing mock failure (robot stuck in pit)…")
        f = String()
        f.data = json.dumps(MOCK_FAILURE)
        self.bt_pub.publish(f)
        self.get_logger().info("Waiting for abort or edit (up to 240 s)…")
        deadline = time.time() + 240
        while (
            time.time() < deadline
            and self.abort_msg is None
            and self.received_edit is None
        ):
            rclpy.spin_once(self, timeout_sec=1.0)
        if self.abort_msg:
            print(
                "\n=== MISSION ABORTED ===\n"
                + str(self.abort_msg)
                + "\n=======================\n"
            )
        elif self.received_edit:
            print(
                "\n=== Edited mission XML (model did NOT abort) ===\n"
                + self.received_edit
                + "\n===\n"
            )
        else:
            print("TIMEOUT — no abort or edit in 240 s", file=sys.stderr)
            sys.exit(1)


def main():
    rclpy.init()
    t = Tester()
    try:
        t.run()
    finally:
        t.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
