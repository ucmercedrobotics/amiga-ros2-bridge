"""Single entry point for the Amiga + Kinova simulation.

    ros2 launch amiga_ros2_gazebo sim_bringup.launch.py

Swaps ONLY the hardware layer (farm-ng bridge, depthai, kortex hardware) for
Gazebo + shims; everything above it — URDF/robot_state_publisher, wheel
odometry, EKFs, Nav2, behavior trees, kortex arm control — is included from
the SAME launch files the real robot uses (see scripts/bringup_amiga_tmux.sh),
with use_sim_time forced true for the whole tree.

The real-robot bringup path is untouched.

Two robots are spawned in Gazebo (see gazebo.launch.py): robot1 is the
original, completely unnamespaced instance; robot2 (namespaced under the
`robot2_name` arg, default "amiga2") is a second, fully independent robot —
base, arm, localization (wheel odom + dual EKF + navsat), and Nav2 (own
`/<robot2_name>/navigate_to_pose` action server), all under its own
namespace (see amiga_localization/bringup.launch.py and
amiga_navigation/navigation.launch.py for how those two — living in the
amiga-ros2-nav submodule — were made namespace-safe). The custom
mission/behavior-tree layer (amiga_ros2_behavior_tree: bt_runner,
waypoint_follower/linear_velo/lidar_object_navigator helpers,
orchard_management) stays single-instance, wired to robot1 only — those
still hardcode several absolute topic/action/service names (and one raw TCP
port) in their own submodules, out of scope here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
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


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    use_lidar = LaunchConfiguration("use_lidar")
    use_gps = LaunchConfiguration("use_gps")
    launch_rviz = LaunchConfiguration("launch_rviz")
    yaw_offset = LaunchConfiguration("yaw_offset")
    robot2_name = LaunchConfiguration("robot2_name").perform(context)

    launch_nav = LaunchConfiguration("launch_nav").perform(context).lower() == "true"
    launch_arm = LaunchConfiguration("launch_arm").perform(context).lower() == "true"
    launch_helpers = (
        LaunchConfiguration("launch_helpers").perform(context).lower() == "true"
    )
    launch_bt = LaunchConfiguration("launch_bt").perform(context).lower() == "true"

    actions = [
        # Force sim clock on every node started below (includes too).
        SetParameter(name="use_sim_time", value=True),
        # ── Hardware layer replacement: spawns BOTH robot1 (unnamespaced)
        # and robot2 (namespaced under robot2_name) — see gazebo.launch.py.
        _include(
            "amiga_ros2_gazebo",
            "gazebo.launch.py",
            world=world,
            headless=headless,
            robot2_name=robot2_name,
        ),
        _include(
            "amiga_ros2_gazebo",
            "sim_hardware_shims.launch.py",
            robot2_name=robot2_name,
        ),
        # ── robot1: identical-to-hardware stack (unchanged) ─────────────
        # publish_joints:=false because the joint_state_broadcaster is the
        # source, filtered to base joints.
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
        _include(
            "amiga_ros2_description",
            "urdf.launch.py",
            publish_joints="false",
            namespace="",
            joint_states_topic="/amiga/joint_states",
            use_lidar=use_lidar,
            gps_link_name="gps_antenna",
            use_vectornav="false",
        ),
        # Localization: wheel odometry + dual EKF + navsat (unchanged,
        # robot1 only — see module docstring).
        _include(
            "amiga_localization",
            "bringup.launch.py",
            use_vectornav="false",
            use_gps=use_gps,
            gps_topic="/gps/pvt",
            namespace="",
        ),
        # ── robot2: base description/TF (own namespace) ──────────────────
        # Localization/Nav2 for robot2 are included further below, gated by
        # launch_nav (same flag as robot1's), not here — this block only
        # covers the always-on base description/TF layer.
        Node(
            package="amiga_ros2_gazebo",
            executable="sim_joint_state_filter.py",
            name="amiga_base_joint_state_filter",
            namespace=robot2_name,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "input_topic": f"/{robot2_name}/joint_states",
                    "mode": "amiga",
                }
            ],
            remappings=[
                ("/amiga/joint_states", f"/{robot2_name}/amiga/joint_states"),
            ],
        ),
        _include(
            "amiga_ros2_description",
            "urdf.launch.py",
            publish_joints="false",
            namespace=robot2_name,
            joint_states_topic=f"/{robot2_name}/amiga/joint_states",
            use_lidar=use_lidar,
            gps_link_name="gps_antenna",
            use_vectornav="false",
        ),
    ]

    if launch_nav:
        actions.append(
            _include(
                "amiga_navigation",
                "navigation.launch.py",
                use_sim_time="True",
                # Explicit, not omitted: LaunchConfiguration("namespace") is
                # global across the whole launch tree, not scoped per
                # include — omitting this would let robot2's includes (which
                # set it explicitly) leak into robot1's stack depending on
                # action ordering. Always pass it explicitly at every call
                # site that reaches a file declaring a "namespace" arg.
                namespace="",
            )
        )
        # robot2's own localization (wheel odom + dual EKF + navsat) and
        # Nav2 stack, fully namespaced — see amiga_localization/bringup.launch.py
        # and amiga_navigation/navigation.launch.py for the namespacing details.
        actions.append(
            _include(
                "amiga_localization",
                "bringup.launch.py",
                use_vectornav="false",
                use_gps=use_gps,
                gps_topic=f"/{robot2_name}/gps/pvt",
                namespace=robot2_name,
            )
        )
        actions.append(
            _include(
                "amiga_navigation",
                "navigation.launch.py",
                use_sim_time="True",
                namespace=robot2_name,
            )
        )

    if launch_arm:
        # robot1's arm (also the only instance carrying kortex_move's
        # `moveto` node, see sim_arm.launch.py). robot_name explicit for the
        # same reason as the "namespace" args above.
        actions.append(
            _include(
                "amiga_ros2_gazebo",
                "sim_arm.launch.py",
                launch_rviz=launch_rviz,
                robot_name="",
            )
        )
        # robot2's arm, fully namespaced, controllable via its own
        # move_group (no `moveto` wrapper — see sim_arm.launch.py).
        actions.append(
            _include(
                "amiga_ros2_gazebo",
                "sim_arm.launch.py",
                launch_rviz=launch_rviz,
                robot_name=robot2_name,
            )
        )

    # Optional helper nodes mirroring the production tmux session (robot1 only).
    if launch_helpers:
        actions += [
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
        ]

    if launch_bt:
        actions.append(_include("amiga_ros2_behavior_tree", "bt.launch.py"))

    return actions


def generate_launch_description():
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
                "robot2_name",
                default_value="amiga2",
                description="Spawn name / ROS namespace for the second robot",
            ),
            DeclareLaunchArgument(
                "yaw_offset",
                default_value="0.0",
                description="Heading offset for waypoint helpers; sim GPS/IMU "
                "are ENU-consistent so 0.0 (production uses 1.57 for the real "
                "compass mounting)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
