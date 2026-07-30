from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="amiga_ros2_mission_planner",
            executable="mission_planner",
            name="mission_planner",
            output="screen",
        )
    ])