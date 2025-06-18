import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory("amiga_ros2_description")
    urdf_path = os.path.join(package_dir, "urdf", "amiga_descr.urdf.xacro")

    return LaunchDescription([
        DeclareLaunchArgument(
            name="urdf", default_value=urdf_path, description="Path to robot URDF"
        ),
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
                   'robot_description': ParameterValue(
                        Command(['xacro ', LaunchConfiguration('urdf')]),
                        value_type=str
                    )
                }
            ]
        ),
    ])