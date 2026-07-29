#!/usr/bin/env python3
"""Republish the Gazebo chassis IMU as the bridge-native IMU topic.

Defaults to the BNO085 topic/frame (base_ekf.yaml). The bno085 URDF frame is
axis-aligned with base_link, matching the chassis-mounted Gazebo IMU, so a
frame_id rewrite is sufficient. Substitutes small covariances when Gazebo
reports zeros so robot_localization does not treat samples as perfect.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _fill_if_zero(cov, value):
    if all(c == 0.0 for c in cov):
        out = [0.0] * 9
        out[0] = out[4] = out[8] = value
        return out
    return cov


class SimImuShim(Node):
    def __init__(self):
        super().__init__("sim_imu_shim")
        self.declare_parameter("input_topic", "/chassis/imu")
        self.declare_parameter("output_topic", "/bno085/imu")
        self.declare_parameter("frame_id", "bno085")
        self.declare_parameter("orientation_covariance", 0.001)
        self.declare_parameter("angular_velocity_covariance", 0.0001)
        self.declare_parameter("linear_acceleration_covariance", 0.01)

        self.frame_id = self.get_parameter("frame_id").value
        self.ori_cov = self.get_parameter("orientation_covariance").value
        self.gyr_cov = self.get_parameter("angular_velocity_covariance").value
        self.acc_cov = self.get_parameter("linear_acceleration_covariance").value

        self.pub = self.create_publisher(
            Imu, self.get_parameter("output_topic").value, 10
        )
        self.sub = self.create_subscription(
            Imu, self.get_parameter("input_topic").value, self.cb, 10
        )

    def cb(self, msg: Imu):
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance = _fill_if_zero(
            msg.orientation_covariance, self.ori_cov
        )
        msg.angular_velocity_covariance = _fill_if_zero(
            msg.angular_velocity_covariance, self.gyr_cov
        )
        msg.linear_acceleration_covariance = _fill_if_zero(
            msg.linear_acceleration_covariance, self.acc_cov
        )
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SimImuShim())


if __name__ == "__main__":
    main()
