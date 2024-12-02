from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Declare essential launch arguments
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
    ]

    # Paths
    gazebo_ros_package = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    amiga_gazebo_package = FindPackageShare(package='amiga_gazebo').find('amiga_gazebo')
    world_file_path = os.path.join(amiga_gazebo_package, 'worlds', 'farmWith1CropRow.world')

    # Include the Gazebo launch file
    gazebo_launch_file = os.path.join(gazebo_ros_package, 'launch', 'gazebo.launch.py')

    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'world': world_file_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Return the minimal launch description
    return LaunchDescription(declared_arguments + [include_gazebo])

