from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="amiga_ros2_agents",
                executable="ltl_gen",
                name="ltl_gen",
                output="screen",
            ),
            Node(
                package="amiga_ros2_agents",
                executable="dummy_agent",
                name="dummy_agent",
                output="screen",
            ),
        ]
    )
