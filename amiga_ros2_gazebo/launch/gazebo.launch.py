"""Gazebo (Ignition Fortress) bringup for the Amiga + Kinova simulator.

Starts:
  * ign gazebo with the orchard world (levels enabled)
  * robot1 ("amiga_kinova"): the ORIGINAL single-robot instance, completely
    unnamespaced — every topic/node name is byte-for-byte what it always was.
    This is intentional: amiga_localization/amiga_navigation/amiga_ros2_behavior_tree
    (separate git submodules, out of scope here) hardcode absolute topics
    like /bno085/imu and /oak0/points, and in this phase the single-instance
    Nav2/EKF/BT stack (see sim_bringup.launch.py) still targets robot1. Do not
    add a namespace to robot1 without also updating those configs.
  * robot2 ("amiga2" by default): a second, fully independent robot spawned
    under its own ROS namespace — own controller_manager, own sensor/bridge
    topics, own hardware shims, own MoveIt arm stack. Driveable in parallel
    with robot1, but not (yet) wired into Nav2/EKF/BT.

Each namespaced robot's ign_ros2_control plugin instance gets a
<ros><namespace> tag so its controller_manager and every controller's
topics/actions live under /<ns>/... . Ignition-transport sensor topics
(which are NOT auto-scoped by model name once a sensor declares an explicit
<topic>) are likewise prefixed with <ns>/ before spawning, so the two robots
never share a Gazebo-side topic.

Controller names match the real ros2-kortex-control stack so kortex_move /
MoveIt run unchanged (see config/ros2_controllers_sim.yaml).
"""

import os
import re
import subprocess
import tempfile

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Ignition-transport sensor topics declared with an explicit <topic> in
# model.sdf (not auto-scoped by model name) that must be made unique per
# namespaced robot instance.
SENSOR_TOPICS = [
    "chassis/imu",
    "navsat",
    "oak_camera_front/imu",
    "oak_camera_front",
    "oak_camera_back",
    "realsense",
    "ouster",
]

IGN_ROS2_CONTROL_PLUGIN_OPEN = (
    '<plugin filename="libign_ros2_control-system.so" '
    'name="ign_ros2_control::IgnitionROS2ControlPlugin">'
)


def resolve_package_uris(content: str) -> str:
    """Rewrite package:// URIs to file:// — ign gazebo cannot resolve them."""

    def replacer(match):
        pkg, rest = match.group(1), match.group(2)
        try:
            return f"file://{get_package_share_directory(pkg)}/{rest}"
        except PackageNotFoundError:
            print(f"[amiga_ros2_gazebo] WARNING: package not found: {pkg}")
            return match.group(0)

    return re.sub(r"package://([^/]+)/(.+?)(?=[<\"\s])", replacer, content)


def qualify_ros(ns: str, path: str) -> str:
    """Absolute ROS name, namespaced under `ns` (ns="" leaves it unchanged)."""
    path = path.lstrip("/")
    return f"/{ns}/{path}" if ns else f"/{path}"


def qualify_gz(ns: str, topic: str) -> str:
    """Ignition-transport topic name, namespaced under `ns` (ns="" = as-is)."""
    return f"{ns}/{topic}" if ns else topic


def namespace_model_sdf(content: str, ns: str) -> str:
    """Prefix per-instance sensor topics and namespace the ign_ros2_control
    plugin so a namespaced robot never collides with robot1 or another
    namespaced robot. No-op when ns is empty (robot1)."""
    if not ns:
        return content
    for topic in SENSOR_TOPICS:
        content = content.replace(
            f"<topic>{topic}</topic>", f"<topic>{qualify_gz(ns, topic)}</topic>"
        )
    content = content.replace(
        IGN_ROS2_CONTROL_PLUGIN_OPEN,
        IGN_ROS2_CONTROL_PLUGIN_OPEN + f"\n      <ros><namespace>{ns}</namespace></ros>",
        1,
    )
    return content


def robot_bridge_args(ns: str) -> list:
    return [
        f"{qualify_ros(ns, 'navsat')}@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
        f"{qualify_ros(ns, 'chassis/imu')}@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        # oak0 (front)
        f"{qualify_ros(ns, 'oak_camera_front/image')}@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"{qualify_ros(ns, 'oak_camera_front/depth_image')}@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"{qualify_ros(ns, 'oak_camera_front/camera_info')}@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        f"{qualify_ros(ns, 'oak_camera_front/points')}@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        # oak1 (back)
        f"{qualify_ros(ns, 'oak_camera_back/image')}@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"{qualify_ros(ns, 'oak_camera_back/depth_image')}@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"{qualify_ros(ns, 'oak_camera_back/camera_info')}@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        f"{qualify_ros(ns, 'oak_camera_back/points')}@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        # Kinova wrist camera
        f"{qualify_ros(ns, 'realsense/image')}@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"{qualify_ros(ns, 'realsense/depth_image')}@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"{qualify_ros(ns, 'realsense/camera_info')}@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        # 3D lidar
        f"{qualify_ros(ns, 'ouster/points')}@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
    ]


def launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory("amiga_ros2_gazebo")
    pkg_descr = get_package_share_directory("amiga_ros2_description")

    world_path = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    use_sim_time = {"use_sim_time": True}

    robots = [
        {
            # robot1 is always unnamespaced — see module docstring.
            "name": LaunchConfiguration("robot1_name").perform(context),
            "ns": "",
            "x": LaunchConfiguration("robot1_x").perform(context),
            "y": LaunchConfiguration("robot1_y").perform(context),
            "z": LaunchConfiguration("robot1_z").perform(context),
            "yaw": LaunchConfiguration("robot1_yaw").perform(context),
            "spawn_delay": 0.0,
        },
        {
            "name": LaunchConfiguration("robot2_name").perform(context),
            "ns": LaunchConfiguration("robot2_name").perform(context),
            "x": LaunchConfiguration("robot2_x").perform(context),
            "y": LaunchConfiguration("robot2_y").perform(context),
            "z": LaunchConfiguration("robot2_z").perform(context),
            "yaw": LaunchConfiguration("robot2_yaw").perform(context),
            # Staggered relative to robot1: both robots' ign_ros2_control
            # plugin instances (and their controller_manager objects) live
            # inside this single `ign gazebo` process. If their controller
            # spawner chains load plugins (joint_state_broadcaster,
            # diff_drive_controller, ...) concurrently, pluginlib's class
            # loader has known thread-safety gaps that intermittently throw
            # "no factory exists for it" for one robot's controller. This
            # delay doesn't eliminate that race, just shrinks the overlap
            # window so it's far less likely to hit in practice.
            "spawn_delay": float(
                LaunchConfiguration("robot2_spawn_delay").perform(context)
            ),
        },
    ]

    # ── Resolve world SDF (package:// -> file://) ────────────────────────
    with open(world_path) as f:
        world_content = f.read()
    world_name_match = re.search(r'<world name="([^"]+)"', world_content)
    world_name = world_name_match.group(1) if world_name_match else "orchard_nbv"
    tmp_world = tempfile.NamedTemporaryFile(
        mode="w", suffix=".sdf", prefix="amiga_world_resolved_", delete=False
    )
    tmp_world.write(resolve_package_uris(world_content))
    tmp_world.flush()

    # ── Resolve robot model SDF (shared template, namespaced per instance) ─
    model_path = os.path.join(pkg_gazebo, "models", "amiga_kinova", "model.sdf")
    controllers_yaml = os.path.join(pkg_gazebo, "config", "ros2_controllers_sim.yaml")
    with open(model_path) as f:
        base_model_content = f.read()
    base_model_content = base_model_content.replace(
        "$(find-pkg-share amiga_ros2_gazebo)/config/ros2_controllers_sim.yaml",
        controllers_yaml,
    )
    base_model_content = resolve_package_uris(base_model_content)

    # ── Combined sim URDF for ign_ros2_control (TF/description muted) ────
    # Shared across both robots: this xacro output is never used for TF (see
    # header of amiga_kinova_sim.urdf.xacro), only for ign_ros2_control's
    # joint lookup, so identical joint names in both robots' isolated
    # controller_manager namespaces do not collide.
    xacro_path = os.path.join(pkg_descr, "urdf", "sim", "amiga_kinova_sim.urdf.xacro")
    result = subprocess.run(
        ["xacro", xacro_path, "sim_ignition:=true"], capture_output=True, text=True
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed for {xacro_path}:\n{result.stderr}")
    sim_robot_description = result.stdout

    # ── Gazebo (single process hosts both robots) ─────────────────────────
    gz_cmd = ["ign", "gazebo", "-r", "--levels"]
    if headless:
        gz_cmd += ["-s", "--headless-rendering"]
    gz_cmd.append(tmp_world.name)
    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env={"IGN_GAZEBO_SYSTEM_PLUGIN_PATH": "/opt/ros/humble/lib"},
    )

    def spawner(controller, ns):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller,
                "-c",
                qualify_ros(ns, "controller_manager"),
                "--controller-manager-timeout",
                "120",
            ],
            output="screen",
            parameters=[use_sim_time],
        )

    actions = [gazebo]
    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"]
    performer_procs = []

    for robot in robots:
        name = robot["name"]
        ns = robot["ns"]

        model_content = namespace_model_sdf(base_model_content, ns)
        tmp_model = tempfile.NamedTemporaryFile(
            mode="w", suffix=".sdf", prefix=f"{name}_resolved_", delete=False
        )
        tmp_model.write(model_content)
        tmp_model.flush()

        gz_description_server = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="gz_description_server",
            namespace=ns,
            output="screen",
            parameters=[use_sim_time, {"robot_description": sim_robot_description}],
            # "/tf"/"/tf_static" are correctly absolute here: tf2_ros
            # hardcodes those as absolute topics regardless of node
            # namespace. "robot_description"/"joint_states" must stay
            # relative (no leading slash) — RSP requests them as relative
            # topics, which only resolve to the absolute form below when
            # ns="" (root); an absolute "from" pattern would silently fail
            # to match once this node is namespaced.
            remappings=[
                ("/tf", qualify_ros(ns, "gz_sim/tf")),
                ("/tf_static", qualify_ros(ns, "gz_sim/tf_static")),
                ("robot_description", qualify_ros(ns, "gz_sim/robot_description")),
                ("joint_states", qualify_ros(ns, "gz_sim/joint_states")),
            ],
        )

        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-file",
                tmp_model.name,
                "-name",
                name,
                "-x",
                robot["x"],
                "-y",
                robot["y"],
                "-z",
                robot["z"],
                "-Y",
                robot["yaw"],
            ],
            output="screen",
            parameters=[use_sim_time],
        )

        bridge_args += robot_bridge_args(ns)

        performer_procs.append(
            ExecuteProcess(
                cmd=[
                    "ign",
                    "service",
                    "-s",
                    f"/world/{world_name}/level/set_performer",
                    "--reqtype",
                    "ignition.msgs.StringMsg",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "2000",
                    "--req",
                    f'data: "{name}"',
                ],
                output="screen",
            )
        )

        spawn_jsb = spawner("joint_state_broadcaster", ns)
        spawn_diff = spawner("diff_drive_controller", ns)
        spawn_jtc = spawner("joint_trajectory_controller", ns)
        spawn_gripper = spawner("robotiq_gripper_controller", ns)

        # Delaying `spawn` delays its whole downstream OnProcessExit chain
        # (jsb -> diff -> jtc -> gripper) with it — see spawn_delay comment
        # above for why this exists.
        spawn_action = (
            TimerAction(period=robot["spawn_delay"], actions=[spawn])
            if robot["spawn_delay"] > 0
            else spawn
        )

        actions += [
            gz_description_server,
            spawn_action,
            RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[spawn_jsb])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_diff])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_diff, on_exit=[spawn_jtc])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_jtc, on_exit=[spawn_gripper])
            ),
        ]

    # ── gz <-> ROS bridge (raw sim topics; shims re-publish bridge-native) ─
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        output="screen",
        parameters=[use_sim_time],
        arguments=bridge_args,
    )
    actions.append(bridge)

    # ── Register both robots as levels performers ─────────────────────────
    # Must fire after every robot has actually spawned, including any
    # staggered/delayed one.
    performer_period = max(5.0, max(r["spawn_delay"] for r in robots) + 2.0)
    actions.append(TimerAction(period=performer_period, actions=performer_procs))

    return actions


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("amiga_ros2_gazebo")
    default_world = os.path.join(pkg_gazebo, "worlds", "orchard_nbv.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("robot1_name", default_value="amiga_kinova"),
            DeclareLaunchArgument("robot1_x", default_value="-5.0"),
            DeclareLaunchArgument("robot1_y", default_value="-3.0"),
            DeclareLaunchArgument("robot1_z", default_value="0.05"),
            DeclareLaunchArgument("robot1_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot2_name", default_value="amiga2"),
            DeclareLaunchArgument("robot2_x", default_value="-5.0"),
            DeclareLaunchArgument("robot2_y", default_value="3.0"),
            DeclareLaunchArgument("robot2_z", default_value="0.05"),
            DeclareLaunchArgument("robot2_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "robot2_spawn_delay",
                default_value="3.0",
                description="Seconds to delay robot2's spawn+controller-loading "
                "chain behind robot1's, to reduce (not eliminate) a pluginlib "
                "class-loader race when both robots' controller_manager "
                "instances load plugins concurrently in the same ign gazebo "
                "process.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
