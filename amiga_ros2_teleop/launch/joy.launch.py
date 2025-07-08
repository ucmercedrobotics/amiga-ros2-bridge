import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[{"dev": "/dev/input/js0", "deadzone": 0.1}],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[
                    os.path.join(
                        get_package_share_directory("amiga_ros2_teleop"),
                        "config",
                        "teleop.config.yaml",
                    )
                ],
            ),
        ]
    )
