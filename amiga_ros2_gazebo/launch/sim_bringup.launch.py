"""Single entry point for the Amiga + Kinova simulation.

    ros2 launch amiga_ros2_gazebo sim_bringup.launch.py

Swaps ONLY the hardware layer (farm-ng bridge, depthai, kortex hardware) for
Gazebo + shims; everything above it — URDF/robot_state_publisher, wheel
odometry, EKFs, Nav2, behavior trees, kortex arm control — is included from
the SAME launch files the real robot uses (see scripts/bringup_amiga_tmux.sh),
with use_sim_time forced true for the whole tree.

The real-robot bringup path is untouched.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _include(package, launch_file, **launch_args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments={k: v for k, v in launch_args.items()}.items(),
    )


def generate_launch_description():
    use_lidar = LaunchConfiguration("use_lidar")
    use_gps = LaunchConfiguration("use_gps")
    launch_nav = LaunchConfiguration("launch_nav")
    launch_arm = LaunchConfiguration("launch_arm")
    launch_helpers = LaunchConfiguration("launch_helpers")
    launch_bt = LaunchConfiguration("launch_bt")
    yaw_offset = LaunchConfiguration("yaw_offset")

    sim_stack = GroupAction(
        actions=[
            # Force sim clock on every node started below (includes too).
            SetParameter(name="use_sim_time", value=True),
            # ── Hardware layer replacement ──────────────────────────────
            _include(
                "amiga_ros2_gazebo",
                "gazebo.launch.py",
                world=LaunchConfiguration("world"),
                headless=LaunchConfiguration("headless"),
            ),
            _include("amiga_ros2_gazebo", "sim_hardware_shims.launch.py"),
            Node(
                package="amiga_ros2_gazebo",
                executable="sim_joint_state_filter.py",
                name="amiga_base_joint_state_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/joint_states",
                        "mode": "amiga",
                    }
                ],
            ),
            # ── Identical-to-hardware stack ─────────────────────────────
            # Robot description / TF (base). publish_joints:=false because
            # the joint_state_broadcaster is the source, filtered to base joints.
            _include(
                "amiga_ros2_description",
                "urdf.launch.py",
                publish_joints="false",
                joint_states_topic="/amiga/joint_states",
                use_lidar=use_lidar,
                gps_link_name="gps_antenna",
                use_vectornav="false",
            ),
            # Localization: wheel odometry + dual EKF + navsat (unchanged).
            _include(
                "amiga_localization",
                "bringup.launch.py",
                use_vectornav="false",
                use_gps=use_gps,
                gps_topic="/gps/pvt",
            ),
        ]
    )

    nav_stack = GroupAction(
        condition=IfCondition(launch_nav),
        actions=[
            SetParameter(name="use_sim_time", value=True),
            _include(
                "amiga_navigation",
                "navigation.launch.py",
                use_sim_time="True",
            ),
        ],
    )

    arm_stack = GroupAction(
        condition=IfCondition(launch_arm),
        actions=[
            SetParameter(name="use_sim_time", value=True),
            _include(
                "amiga_ros2_gazebo",
                "sim_arm.launch.py",
                launch_rviz=LaunchConfiguration("launch_rviz"),
            ),
        ],
    )

    # Optional helper nodes mirroring the production tmux session.
    helpers = GroupAction(
        condition=IfCondition(launch_helpers),
        actions=[
            SetParameter(name="use_sim_time", value=True),
            Node(
                package="amiga_navigation",
                executable="waypoint_follower.py",
                name="waypoint_follower",
                output="screen",
                parameters=[{"yaw_offset": yaw_offset}],
            ),
            Node(
                package="amiga_navigation",
                executable="linear_velo",
                name="linear_velo",
                output="screen",
                parameters=[
                    {
                        "min_linear_velocity": 0.3,
                        "max_linear_velocity": 1.0,
                        "yaw_slowdown": 0.8,
                        "max_angular_velocity": 0.7,
                        "yaw_offset": yaw_offset,
                    }
                ],
            ),
            Node(
                package="amiga_navigation",
                executable="lidar_object_navigator",
                name="lidar_object_navigator",
                output="screen",
                parameters=[
                    {
                        "safety_distance": 0.8,
                        "lidar_topic": "/ouster/points",
                        "azimuth_tolerance": 0.5,
                        "min_object_height": 0.1,
                        "max_object_height": 1.5,
                        "min_object_distance": 1.0,
                        "max_object_distance": 5.0,
                    }
                ],
            ),
        ],
    )

    bt_stack = GroupAction(
        condition=IfCondition(launch_bt),
        actions=[
            SetParameter(name="use_sim_time", value=True),
            _include("amiga_ros2_behavior_tree", "bt.launch.py"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    get_package_share_directory("amiga_ros2_gazebo"),
                    "worlds",
                    "orchard_nbv.sdf",
                ),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("use_gps", default_value="true"),
            DeclareLaunchArgument("launch_nav", default_value="true"),
            DeclareLaunchArgument("launch_arm", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument(
                "launch_helpers",
                default_value="true",
                description="Start waypoint_follower + linear_velo (as in tmux bringup)",
            ),
            DeclareLaunchArgument("launch_bt", default_value="true"),
            DeclareLaunchArgument(
                "yaw_offset",
                default_value="0.0",
                description="Heading offset for waypoint helpers; sim GPS/IMU "
                "are ENU-consistent so 0.0 (production uses 1.57 for the real "
                "compass mounting)",
            ),
            sim_stack,
            nav_stack,
            arm_stack,
            helpers,
            bt_stack,
        ]
    )
