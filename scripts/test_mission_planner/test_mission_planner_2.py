"""
Scenario: robot samples every tree in a row (1-10). Trees 3, 4, 5, 6 are each
found missing at their waypoints (permanently removed). The planner should
drop each missing tree; the arbiter should allow each drop (permanent).
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


def _missing_tree_failure(tree_id: int) -> dict:
    return {
        "node": f"Visit_Tree_{tree_id}",
        "status": "FAILURE",
        "timestamp_ms": 0,  # set at publish time
        "reason": (
            f"action server returned ABORTED — no tree detected at waypoint for tree ID {tree_id}. "
            f"Cause unknown: could be a sensor miss, temporary gap, or a spot being replanted. "
            f"Tree may be present on a later pass."
        ),
    }



MOCK_FAILURES = [_missing_tree_failure(t) for t in (3, 4, 5, 6, 7, 8)]


class Tester(Node):
    def __init__(self):
        super().__init__("mission_planner_tester")
        self.xml_pub = self.create_publisher(String, "/mission/xml", 10)
        self.bt_pub = self.create_publisher(String, "/bt/status_change", 10)
        self.received_edits = []

        self.create_subscription(String, "/mission/xml", self._on_xml, 10)

    def _on_xml(self, msg: String):
        if msg.data == SAMPLE_XML:
            return
        if msg.data in self.received_edits:
            return
        self.received_edits.append(msg.data)
        self.get_logger().info(f"Received edited XML #{len(self.received_edits)}!")

    def _wait_for_edit(self, count: int, timeout_sec: float = 300.0) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline and len(self.received_edits) < count:
            rclpy.spin_once(self, timeout_sec=1.0)
        return len(self.received_edits) >= count

    def run(self):
        self.get_logger().info("Publishing 10-tree row mission XML…")
        m = String()
        m.data = SAMPLE_XML
        self.xml_pub.publish(m)

        time.sleep(2.0)

        for i, failure in enumerate(MOCK_FAILURES, 1):
            self.get_logger().info(
                f"Publishing mock BT failure {i}/{len(MOCK_FAILURES)} "
                f"({failure['node']})…"
            )
            failure["timestamp_ms"] = int(time.time() * 1000)
            f = String()
            f.data = json.dumps(failure)
            self.bt_pub.publish(f)

            self.get_logger().info(f"Waiting for edited plan {i} (up to 300 s)…")
            if not self._wait_for_edit(i):
                print(f"TIMEOUT — no edited XML for failure {i}", file=sys.stderr)
                sys.exit(1)

            time.sleep(5.5)  # ≥ arbiter MIN_ACCEPT_INTERVAL_SEC so the next
                             # accepted plan isn't bounced by the rate limit

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