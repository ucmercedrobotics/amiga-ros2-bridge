import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [get_package_share_directory("amiga_ros2_bridge"), "urdf/", "amiga_base.urdf"]
    )
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name="urdf", default_value=urdf_path, description="Path to robot URDF"
        # ),
        DeclareLaunchArgument(
            name="publish_joints", default_value="true", description="Publish joint states"
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output='screen',
            condition=IfCondition(LaunchConfiguration("publish_joints")),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    # TODO: Convert urdf to xacro for readability
                    # 'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                    'robot_description': robot_description_content,
                }
            ]
        ),
    ])