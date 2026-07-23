"""Scenario 1: a tree in the row is physically missing when the robot arrives."""
import json, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _tree_steps(ids):
    rows = []
    for i in ids:
        rows.append(f'      <MoveToTreeID name="Visit_Tree_{i}" action_name="follow_tree_id_waypoint" id="{i}" approach_tree="true"/>')
        rows.append(f'      <SampleLeaf name="Sample_Leaves_Tree_{i}" action_name="segment_leaves"/>')
    return "\n".join(rows)


SAMPLE_XML = f"""\
<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">
  <Mission>sample leaves from every tree in the row, trees 1 through 10</Mission>
  <BehaviorTree ID="Sample_Leaves_Row_1_10">
    <Sequence>
{_tree_steps(range(1, 11))}
    </Sequence>
  </BehaviorTree>
</root>"""

MOCK_FAILURE = {
    "node": "Visit_Tree_5",
    "status": "FAILURE",
    "timestamp_ms": int(time.time() * 1000),
    "reason": "no tree at the mapped location for tree 5; the spot is bare soil.",
}

class Tester(Node):
    def __init__(self):
        super().__init__("missing_tree_tester")
        self.xml_pub = self.create_publisher(String, "/mission/xml", 10)
        self.bt_pub = self.create_publisher(String, "/bt/status_change", 10)
        self.received_edit = None
        self.create_subscription(String, "/mission/xml", self._on_xml, 10)

    def _on_xml(self, msg):
        if msg.data == SAMPLE_XML:
            return
        self.received_edit = msg.data
        self.get_logger().info("Received edited XML!")

    def run(self):
        self.get_logger().info("Publishing row 1-10 mission…")
        m = String(); m.data = SAMPLE_XML; self.xml_pub.publish(m)
        time.sleep(2.0)
        self.get_logger().info("Publishing mock failure (tree 5 missing)…")
        f = String(); f.data = json.dumps(MOCK_FAILURE); self.bt_pub.publish(f)
        self.get_logger().info("Waiting for edited plan (up to 240 s)…")
        deadline = time.time() + 240
        while time.time() < deadline and self.received_edit is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        if self.received_edit:
            print("\n=== Edited mission XML ===\n" + self.received_edit + "\n=========================\n")
        else:
            print("TIMEOUT — no edited XML in 240 s", file=sys.stderr); sys.exit(1)


def main():
    rclpy.init(); t = Tester()
    try: t.run()
    finally: t.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()