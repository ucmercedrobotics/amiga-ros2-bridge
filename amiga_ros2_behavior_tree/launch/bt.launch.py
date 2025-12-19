from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="amiga_ros2_behavior_tree",
                executable="bt_runner",
                name="bt_runner",
                output="screen",
            ),
            Node(
                package="amiga_ros2_behavior_tree",
                executable="tcp_demux_node",
                name="tcp_demux",
                output="screen",
            ),
            Node(
                package="amiga_ros2_behavior_tree",
                executable="orchard_management_node",
                name="orchard_management",
                output="screen",
            ),
        ]
    )
