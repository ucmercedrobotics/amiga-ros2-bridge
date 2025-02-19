from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="amiga_ros2_bridge",
                executable="twist_wasd",
                name="twist_wasd",
            )
        ]
    )
