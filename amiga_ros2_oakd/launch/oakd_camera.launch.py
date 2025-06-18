import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration("name").perform(context)

    nodes = [
        ComposableNodeContainer(
            name=name + "_container",
            namespace=name,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=name,
                    parameters=[params_file],
                )
            ],
            arguments=["--ros-args", "--log-level", log_level],
            output="both",
        ),
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name=name + "_rectify_color_node",
                    remappings=[
                        ("image", "rgb/image_raw"),
                        ("camera_info", "rgb/camera_info"),
                        ("image_rect", "rgb/image_rect"),
                        ("image_rect/compressed", "rgb/image_rect/compressed"),
                        ("image_rect/compressedDepth", "rgb/image_rect/compressedDepth"),
                        ("image_rect/theora", "rgb/image_rect/theora"),
                    ],
                )
            ],
        ),
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name=name + "_point_cloud_xyzrgb_node",
                    remappings=[
                        ("depth_registered/image_rect", "stereo/image_raw"),
                        ("rgb/image_rect_color", "rgb/image_rect"),
                        ("rgb/camera_info", "rgb/camera_info"),
                        ("points", "points"),
                    ],
                ),
            ],
        ),
    ]

    return nodes


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "rgbd.yaml"),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )