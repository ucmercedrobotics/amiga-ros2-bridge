import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

# Oak0: front camera, Oak1: back camera
CAMS = ["oak0", "oak1"]


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory("amiga_ros2_oakd")
    params_file = os.path.join(package_dir, "config", "amiga_cameras.yaml")

    # Launch nodes for each camera
    nodes = []
    for cam_name in CAMS:
        node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, "launch", "oakd_camera.launch.py")
            ),
            launch_arguments={
                "name": cam_name,
                "params_file": params_file,
                "rectify_rgb": "false",
                "pointcloud_enable": "true",
            }.items(),
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
