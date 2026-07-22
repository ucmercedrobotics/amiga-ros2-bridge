#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import NavSatFix, Joy
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

from tf2_ros import Buffer, TransformListener

import math
import yaml
import os

from amiga_ros2_teleop.controller_utils import (
    load_controller_config,
    ControllerMap,
    ButtonTrigger,
)
from amiga_navigation.utils.gps_utils import (
    euler_from_quaternion,
    apply_yaw_offset,
    get_yaw_from_tf,
)


class JoyGpsWaypointLogger(Node):
    def __init__(self, output_path: str, controller_config: dict):
        """
        ROS2 node to log GPS waypoints to a file.
        Is triggered from a controller button.
        """
        super().__init__("joy_gps_waypoint_logger")

        self.output_path = output_path
        self.controller_config = controller_config

        qos_profile = QoSProfile(depth=10)

        # -- Subscribe to Joy topic
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_callback, qos_profile
        )
        self.button = ButtonTrigger(key="a", max_hold_frames=10)

        # TODO: Make configurable
        # -- Subscribe to NavSatFix for lat and lon
        self.gps_sub = self.create_subscription(
            NavSatFix, "/ublox_gps_node/fix", self.gps_callback, qos_profile
        )
        self.last_gps_position = NavSatFix()

        # -- Subscribe to Global EKF for heading
        self.odom_sub = self.create_subscription(
            Odometry, "odometry/filtered/global", self.odom_callback, qos_profile
        )
        self.last_heading = 0.0
        # Need to transform to ENU heading using map->utm transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.yaw_offset = None
        self.create_timer(1.0, self.check_for_yaw_offset)

    def gps_callback(self, msg: NavSatFix):
        """
        Callback to store the last GPS pose
        """
        self.last_gps_position = msg

    def odom_callback(self, msg: Odometry):
        """
        Callback to store the last heading.
        Takes yaw from global odometry and transforms heading into
        ENU format using the Map->Utm transform.
        """
        _, _, ekf_yaw = euler_from_quaternion(msg.pose.pose.orientation)

        if self.yaw_offset is not None:
            corrected_yaw = apply_yaw_offset(ekf_yaw, self.yaw_offset)
            self.last_heading = corrected_yaw
        else:
            self.get_logger().warn("Yaw offset not initialized... using raw EKF yaw.")
            self.last_heading = ekf_yaw

    def joy_callback(self, msg: Joy):
        cmap = ControllerMap(msg, self.controller_config)

        if self.button.query(cmap):
            data = self.log_waypoint()
            self.get_logger().info(f"Logged waypoint to file: {data}")

    def log_waypoint(self):
        """
        Function to save a new waypoint to a file.
        """
        # -- Ensure gps is stored
        if not self.last_gps_position.header.stamp.sec:
            self.get_logger().warn("GPS data not yet received. Cannot log waypoint.")
            return

        # -- read existing waypoints
        try:
            with open(self.output_path, "r") as yaml_file:
                existing_data = yaml.safe_load(yaml_file)
        # in case the file does not exist, create with the new wps
        except FileNotFoundError:
            existing_data = {"waypoints": []}

        # -- build new waypoint object
        data = {
            "latitude": self.last_gps_position.latitude,
            "longitude": self.last_gps_position.longitude,
            "yaw": self.last_heading,
        }
        existing_data["waypoints"].append(data)

        # -- write updated waypoints
        with open(self.output_path, "w") as yaml_file:
            yaml.dump(existing_data, yaml_file, default_flow_style=False)

        return data

    async def check_for_yaw_offset(self):
        """Get the yaw offset based om the map->utm transform"""
        from_frame = "utm"
        to_frame = "map"
        when = rclpy.time.Time()

        self.get_logger().debug(f"Checking for yaw offset: {from_frame} -> {to_frame}")
        try:
            transform = await self.tf_buffer.lookup_transform_async(
                to_frame, from_frame, when
            )
            self.yaw_offset = get_yaw_from_tf(transform)
            self.get_logger().debug(
                f"Yaw offset set to {math.degrees(self.yaw_offset):.2f}°"
            )
        except LookupException as e:
            self.get_logger().warn_throttle(5.0, f"Yaw offset not yet available: {e}")


def main(args=None):
    rclpy.init(args=args)

    yaml_file_path = "gps_waypoints.yaml"
    controller_config_path = os.path.join(
        get_package_share_directory("amiga_ros2_teleop"), "config", "ps4.yaml"
    )
    controller_config = load_controller_config(controller_config_path)

    node = JoyGpsWaypointLogger(yaml_file_path, controller_config)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
