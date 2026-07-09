#!/usr/bin/env python3
"""Simulation stand-in for amiga_ros2_bridge twist_control_node.

On hardware, twist_control_node forwards /cmd_vel to the farm-ng canbus
service. In simulation the same /cmd_vel is forwarded to the ros2_control
diff_drive_controller running inside Gazebo.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist


class SimTwistControl(Node):
    def __init__(self):
        super().__init__("sim_twist_control")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(
            Twist, "/diff_drive_controller/cmd_vel_unstamped", qos
        )
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, qos)
        self.get_logger().info(
            "sim twist control: /cmd_vel -> /diff_drive_controller/cmd_vel_unstamped"
        )

    def cb(self, msg: Twist):
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SimTwistControl())


if __name__ == "__main__":
    main()
