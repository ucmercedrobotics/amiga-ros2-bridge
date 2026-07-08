#!/usr/bin/env python3
"""Simulation stand-in for the farm-ng canbus twist feedback.

On hardware, amiga_streams_node publishes /canbus/twist (TwistStamped, frame
"robot") with the measured vehicle twist; wheel_odometry_node integrates it.
In simulation the closed-loop body twist reported by diff_drive_controller's
odometry is republished in the same shape, rate-limited to roughly the canbus
cadence.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class SimCanbusTwist(Node):
    def __init__(self):
        super().__init__("sim_canbus_twist")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("frame_id", "robot")
        rate = self.get_parameter("rate_hz").value
        self.min_period_ns = int(1e9 / rate) if rate > 0.0 else 0
        self.frame_id = self.get_parameter("frame_id").value
        self.last_pub_ns = 0

        self.pub = self.create_publisher(TwistStamped, "/canbus/twist", 10)
        self.sub = self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self.cb, 10
        )
        self.get_logger().info(
            "sim canbus twist: /diff_drive_controller/odom -> /canbus/twist"
        )

    def cb(self, msg: Odometry):
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self.min_period_ns and stamp_ns - self.last_pub_ns < self.min_period_ns:
            return
        self.last_pub_ns = stamp_ns

        out = TwistStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.twist = msg.twist.twist
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(SimCanbusTwist())


if __name__ == "__main__":
    main()
