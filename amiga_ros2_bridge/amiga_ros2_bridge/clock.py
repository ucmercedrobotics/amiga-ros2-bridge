#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock


class PublishClock(Node):
    def __init__(self):
        super().__init__("clock")

        self.sub_canbus_twist = self.create_subscription(
            TwistStamped, "/canbus/twist", self.canbus_twist_callback, 10
        )

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)

    def canbus_twist_callback(self, msg):
        self.publish_clock(msg.header.stamp)

    def publish_clock(self, stamp):
        clock_msg = Clock()
        clock_msg.clock = stamp
        self.clock_pub.publish(clock_msg)
        self.get_logger().info(f"Published timestamp to /clock: {stamp}")


def main(args=None):
    rclpy.init(args=args)
    node = PublishClock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
