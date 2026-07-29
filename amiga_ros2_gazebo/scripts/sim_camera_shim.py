#!/usr/bin/env python3
"""Republish one Gazebo rgbd camera under the bridge-native topics/frames.

One instance runs per camera (oak0, oak1, wrist). It rewrites frame_ids to
the frames the real drivers stamp (depthai / kinova_vision) and optionally
converts Gazebo's 32FC1 metre depth images to the 16UC1 millimetre encoding
both real drivers publish.

Empty output-topic parameters disable the corresponding stream.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class SimCameraShim(Node):
    def __init__(self):
        super().__init__("sim_camera_shim")
        p = self.declare_parameter
        p("rgb_in", "")
        p("rgb_out", "")
        p("depth_in", "")
        p("depth_out", "")
        p("info_in", "")
        p("rgb_info_out", "")
        p("depth_info_out", "")
        p("points_in", "")
        p("points_out", "")
        p("rgb_frame_id", "")
        p("depth_frame_id", "")
        p("depth_to_mm", True)

        g = lambda name: self.get_parameter(name).value
        self.rgb_frame = g("rgb_frame_id")
        self.depth_frame = g("depth_frame_id") or self.rgb_frame
        self.depth_to_mm = g("depth_to_mm")

        # Sensor data QoS on inputs (matches ros_gz bridge publishers).
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        out_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        if g("rgb_in") and g("rgb_out"):
            self.rgb_pub = self.create_publisher(Image, g("rgb_out"), out_qos)
            self.create_subscription(Image, g("rgb_in"), self.rgb_cb, sensor_qos)

        if g("depth_in") and g("depth_out"):
            self.depth_pub = self.create_publisher(Image, g("depth_out"), out_qos)
            self.create_subscription(Image, g("depth_in"), self.depth_cb, sensor_qos)

        self.rgb_info_pub = None
        self.depth_info_pub = None
        if g("info_in"):
            if g("rgb_info_out"):
                self.rgb_info_pub = self.create_publisher(
                    CameraInfo, g("rgb_info_out"), out_qos
                )
            if g("depth_info_out"):
                self.depth_info_pub = self.create_publisher(
                    CameraInfo, g("depth_info_out"), out_qos
                )
            self.create_subscription(CameraInfo, g("info_in"), self.info_cb, sensor_qos)

        if g("points_in") and g("points_out"):
            self.points_pub = self.create_publisher(
                PointCloud2, g("points_out"), out_qos
            )
            self.create_subscription(
                PointCloud2, g("points_in"), self.points_cb, sensor_qos
            )

    def rgb_cb(self, msg: Image):
        msg.header.frame_id = self.rgb_frame
        self.rgb_pub.publish(msg)

    def depth_cb(self, msg: Image):
        msg.header.frame_id = self.depth_frame
        if self.depth_to_mm and msg.encoding == "32FC1":
            depth_m = np.frombuffer(msg.data, dtype=np.float32)
            depth_mm = np.nan_to_num(depth_m * 1000.0, nan=0.0, posinf=0.0)
            depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)
            msg.encoding = "16UC1"
            msg.data = depth_mm.tobytes()
            msg.step = msg.width * 2
            msg.is_bigendian = 0
        self.depth_pub.publish(msg)

    def info_cb(self, msg: CameraInfo):
        if self.rgb_info_pub is not None:
            msg.header.frame_id = self.rgb_frame
            self.rgb_info_pub.publish(msg)
        if self.depth_info_pub is not None:
            msg.header.frame_id = self.depth_frame
            self.depth_info_pub.publish(msg)

    def points_cb(self, msg: PointCloud2):
        msg.header.frame_id = self.rgb_frame
        self.points_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SimCameraShim())


if __name__ == "__main__":
    main()
