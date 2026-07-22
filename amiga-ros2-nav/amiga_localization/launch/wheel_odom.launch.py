from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def qualify_ros(ns, topic):
    """Absolute topic name, namespaced under `ns` (ns="" leaves it unchanged)."""
    topic = topic.lstrip("/")
    return f"/{ns}/{topic}" if ns else f"/{topic}"


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)

    return [
        Node(
            package='amiga_localization',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            namespace=ns,
            parameters=[os.path.join(get_package_share_directory("amiga_localization"), "config", "wheel_odom.yaml")],
            # wheel_odom.py hardcodes these absolute in source (not
            # parameterized), so they bypass namespace= without a remap.
            remappings=[
                ("/canbus/twist", qualify_ros(ns, "canbus/twist")),
                ("/wheel/odometry", qualify_ros(ns, "wheel/odometry")),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for wheel_odometry_node (per-robot, e.g. 'amiga2')",
        ),
        OpaqueFunction(function=launch_setup),
    ])
