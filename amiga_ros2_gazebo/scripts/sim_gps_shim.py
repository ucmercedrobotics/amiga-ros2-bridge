#!/usr/bin/env python3
"""Republish the Gazebo navsat fix as the farm-ng GPS topic.

Rewrites frame_id to the bridge-native antenna frame and substitutes a sane
position covariance when Gazebo reports zeros (robot_localization treats a
zero covariance as perfect measurement).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class SimGpsShim(Node):
    def __init__(self):
        super().__init__("sim_gps_shim")
        self.declare_parameter("input_topic", "/navsat")
        self.declare_parameter("output_topic", "/gps/pvt")
        self.declare_parameter("frame_id", "gps_antenna")
        self.declare_parameter("default_covariance", 0.01)  # m^2 diagonal

        self.frame_id = self.get_parameter("frame_id").value
        self.default_cov = self.get_parameter("default_covariance").value

        self.pub = self.create_publisher(
            NavSatFix, self.get_parameter("output_topic").value, 10
        )
        self.sub = self.create_subscription(
            NavSatFix, self.get_parameter("input_topic").value, self.cb, 10
        )

    def cb(self, msg: NavSatFix):
        msg.header.frame_id = self.frame_id
        if all(c == 0.0 for c in msg.position_covariance):
            cov = [0.0] * 9
            cov[0] = cov[4] = cov[8] = self.default_cov
            msg.position_covariance = cov
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SimGpsShim())


if __name__ == "__main__":
    main()
