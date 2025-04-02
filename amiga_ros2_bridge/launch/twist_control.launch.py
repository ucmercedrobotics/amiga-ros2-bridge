from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetLaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            SetLaunchConfiguration("log_level", "DEBUG"),
            Node(
                package="amiga_ros2_bridge",
                executable="twist_control",
                name="twist_control",
            ),
        ]
    )
