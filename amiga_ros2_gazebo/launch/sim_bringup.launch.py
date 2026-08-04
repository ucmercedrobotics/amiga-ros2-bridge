"""Single entry point for the Amiga + Kinova simulation.

    ros2 launch amiga_ros2_gazebo sim_bringup.launch.py

Swaps ONLY the hardware layer (farm-ng bridge, depthai, kortex hardware) for
Gazebo + shims; everything above it — URDF/robot_state_publisher, wheel
odometry, EKFs, Nav2, behavior trees, kortex arm control — is included from
the SAME launch files the real robot uses (see scripts/bringup_amiga_tmux.sh),
with use_sim_time forced true for the whole tree.

The real-robot bringup path is untouched.

`robot_count` (default 1) identical robots are brought up, each with its own
base, arm, localization (wheel odom + dual EKF + navsat), Nav2 (own
`/<name>/navigate_to_pose` action server), and mission/behavior-tree layer
(amiga_ros2_behavior_tree: bt_runner, waypoint_follower/linear_velo/
lidar_object_navigator helpers, orchard_management) — all under its own
namespace except robot1, which stays completely unnamespaced (identical to
the original single-robot topic/node layout, see gazebo.launch.py's module
docstring for why). See amiga_localization/bringup.launch.py and
amiga_navigation/navigation.launch.py (amiga-ros2-nav submodule) and
amiga_ros2_behavior_tree/launch/bt.launch.py for how each of those was made
namespace-safe — `amiga_navigation`'s three helper nodes are constructed
directly below since they have no launch file of their own upstream of this
one. `make sim-dual` sets `robot_count:=2`; `make sim-multi
ROBOT_COUNT=<n>` sets `robot_count:=<n>`.

The one process-level (not just namespace) per-robot resource is
amiga_ros2_behavior_tree's tcp_demux_node TCP port: each robot's `bt.launch.py`
gets `port=mission_port_base + (i-1)`, since two instances sharing a port
wouldn't fail to start (SO_REUSEPORT) — they'd silently load-balance/misroute
mission connections between robots instead.
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


def qualify_ros(ns, topic):
    """Absolute topic name, namespaced under `ns` (ns="" leaves it unchanged)."""
    topic = topic.lstrip("/")
    return f"/{ns}/{topic}" if ns else f"/{topic}"


# tf2_ros hardcodes /tf, /tf_static as absolute regardless of node namespace
# — every node here that might touch TF gets this remap, same as everywhere
# else in this repo's sim launch files.
def tf_remaps(ns):
    return [
        ("/tf", qualify_ros(ns, "tf")),
        ("/tf_static", qualify_ros(ns, "tf_static")),
    ]


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    robot_count = int(LaunchConfiguration("robot_count").perform(context))
    name_prefix = LaunchConfiguration("robot_name_prefix").perform(context)
    use_lidar = LaunchConfiguration("use_lidar")
    use_gps = LaunchConfiguration("use_gps")
    launch_rviz = LaunchConfiguration("launch_rviz")
    yaw_offset = LaunchConfiguration("yaw_offset")
    mission_port_base = int(LaunchConfiguration("mission_port_base").perform(context))

    launch_nav = LaunchConfiguration("launch_nav").perform(context).lower() == "true"
    launch_arm = LaunchConfiguration("launch_arm").perform(context).lower() == "true"
    launch_helpers = (
        LaunchConfiguration("launch_helpers").perform(context).lower() == "true"
    )
    launch_bt = LaunchConfiguration("launch_bt").perform(context).lower() == "true"

    actions = [
        # Force sim clock on every node started below (includes too).
        SetParameter(name="use_sim_time", value=True),
        # ── Hardware layer replacement for all robot_count robots — see
        # gazebo.launch.py.
        _include(
            "amiga_ros2_gazebo",
            "gazebo.launch.py",
            world=world,
            headless=headless,
            robot_count=str(robot_count),
            robot_name_prefix=name_prefix,
        ),
        _include(
            "amiga_ros2_gazebo",
            "sim_hardware_shims.launch.py",
            robot_count=str(robot_count),
            robot_name_prefix=name_prefix,
        ),
    ]

    for i in range(1, robot_count + 1):
        # robot1 is always unnamespaced — see module docstring. Namespace
        # only, not spawn name (that's gazebo.launch.py's own robot1_name
        # arg, not threaded through here, same as before this file
        # generalized to N robots).
        ns = "" if i == 1 else f"{name_prefix}{i}"

        # ── Base description/TF (identical-to-hardware stack) ─────────────
        # publish_joints:=false because the joint_state_broadcaster is the
        # source, filtered to base joints.
        actions += [
            Node(
                package="amiga_ros2_gazebo",
                executable="sim_joint_state_filter.py",
                name="amiga_base_joint_state_filter",
                namespace=ns,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": qualify_ros(ns, "joint_states"),
                        "mode": "amiga",
                    }
                ],
                # /amiga/joint_states is hardcoded absolute in
                # sim_joint_state_filter.py's own output (not a param);
                # no-op remap when ns="" (qualify_ros returns the same string).
                remappings=[
                    ("/amiga/joint_states", qualify_ros(ns, "amiga/joint_states")),
                ],
            ),
            _include(
                "amiga_ros2_description",
                "urdf.launch.py",
                publish_joints="false",
                namespace=ns,
                joint_states_topic=qualify_ros(ns, "amiga/joint_states"),
                use_lidar=use_lidar,
                gps_link_name="gps_antenna",
                use_vectornav="false",
            ),
        ]

        # ── Localization + Nav2 ────────────────────────────────────────────
        if launch_nav:
            actions.append(
                _include(
                    "amiga_localization",
                    "bringup.launch.py",
                    use_vectornav="false",
                    use_gps=use_gps,
                    gps_topic=qualify_ros(ns, "gps/pvt"),
                    # Explicit, not omitted: LaunchConfiguration("namespace")
                    # is global across the whole launch tree, not scoped per
                    # include — an omitted arg picks up whatever an earlier
                    # include last set it to, rather than this file's own
                    # default. Always pass it explicitly at every call site
                    # that reaches a file declaring a "namespace" arg.
                    namespace=ns,
                )
            )
            actions.append(
                _include(
                    "amiga_navigation",
                    "navigation.launch.py",
                    use_sim_time="True",
                    namespace=ns,
                )
            )

        # ── Arm (MoveIt) — robot1 also carries kortex_move's `moveto`,
        # everyone else just their own move_group (see sim_arm.launch.py) ──
        if launch_arm:
            actions.append(
                _include(
                    "amiga_ros2_gazebo",
                    "sim_arm.launch.py",
                    launch_rviz=launch_rviz,
                    robot_name=ns,
                )
            )

        # ── Helper nodes mirroring the production tmux session ────────────
        if launch_helpers:
            actions += [
                Node(
                    package="amiga_navigation",
                    executable="waypoint_follower.py",
                    name="waypoint_follower",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        {
                            "yaw_offset": yaw_offset,
                            "orchard_tree_service": qualify_ros(
                                ns, "orchard/get_tree_info"
                            ),
                        }
                    ],
                    # /gps/filtered, /odometry/filtered/local: hardcoded
                    # absolute subscriptions, no param. /navigate_via_lidar:
                    # hardcoded absolute ActionClient (not a param, unlike
                    # its own action servers) — without this remap it would
                    # always dial the unnamespaced top-level action instead
                    # of this robot's own lidar_object_navigator below.
                    remappings=tf_remaps(ns)
                    + [
                        ("/gps/filtered", qualify_ros(ns, "gps/filtered")),
                        (
                            "/odometry/filtered/local",
                            qualify_ros(ns, "odometry/filtered/local"),
                        ),
                        ("/navigate_via_lidar", qualify_ros(ns, "navigate_via_lidar")),
                    ],
                ),
                Node(
                    package="amiga_navigation",
                    executable="linear_velo",
                    name="linear_velo",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        {
                            "min_linear_velocity": 0.3,
                            "max_linear_velocity": 1.0,
                            "yaw_slowdown": 0.8,
                            "max_angular_velocity": 0.7,
                            "yaw_offset": yaw_offset,
                            "odom_topic": qualify_ros(ns, "odometry/filtered/local"),
                        }
                    ],
                    # /cmd_vel_raw: hardcoded absolute publisher, no param.
                    remappings=tf_remaps(ns)
                    + [("/cmd_vel_raw", qualify_ros(ns, "cmd_vel_raw"))],
                ),
                Node(
                    package="amiga_navigation",
                    executable="lidar_object_navigator",
                    name="lidar_object_navigator",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        {
                            "safety_distance": 0.8,
                            "lidar_topic": qualify_ros(ns, "ouster/points"),
                            "azimuth_tolerance": 0.5,
                            "min_object_height": 0.1,
                            "max_object_height": 1.5,
                            "min_object_distance": 1.0,
                            "max_object_distance": 5.0,
                        }
                    ],
                    # base_frame/lidar_link stay bare ("base_link"/
                    # "lidar_link") — each robot's TF lives on its own
                    # namespaced tf topic (see tf_remaps below), not on a
                    # prefixed frame_id string, matching every other
                    # TF-consuming node in this repo.
                    remappings=tf_remaps(ns),
                ),
            ]

        # ── Mission/behavior-tree layer ────────────────────────────────────
        if launch_bt:
            actions.append(
                _include(
                    "amiga_ros2_behavior_tree",
                    "bt.launch.py",
                    namespace=ns,
                    port=str(mission_port_base + i - 1),
                )
            )

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
            DeclareLaunchArgument(
                "robot_count",
                default_value="1",
                description="How many identical robots to bring up (see "
                "module docstring). make sim-dual sets this to 2, "
                "make sim-multi to $(ROBOT_COUNT).",
            ),
            DeclareLaunchArgument(
                "robot_name_prefix",
                default_value="amiga",
                description="Name/namespace prefix for robots 2..N",
            ),
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
                "mission_port_base",
                default_value="12346",
                description="TCP mission port for robot1; robot i gets "
                "mission_port_base + (i-1) (see module docstring).",
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
