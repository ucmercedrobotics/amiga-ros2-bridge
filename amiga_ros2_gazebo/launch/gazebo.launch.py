"""Gazebo (Ignition Fortress) bringup for the Amiga + Kinova simulator.

Starts:
  * ign gazebo with the orchard world (all trees always-loaded; see
    generate_orchard_world.py for why there's no levels/performer gating)
  * `robot_count` (default 1) identical Amiga+Kinova robots. With
    robot_count==1, that single robot is unnamespaced ("amiga_kinova" by
    default) — every topic/node name for it is byte-for-byte what it always
    was. This is intentional: amiga_localization/amiga_navigation/
    amiga_ros2_behavior_tree (separate git submodules) hardcode absolute
    topics like /bno085/imu and /oak0/points, historically targeting a
    single unnamespaced robot; single-robot sim keeps that exact shape so
    those configs need no changes. Once robot_count>1, EVERY robot
    (including robot1) is named/namespaced "<robot_name_prefix><i>", e.g.
    "amiga1", "amiga2", ... — each a fully independent robot under its own
    ROS namespace: own controller_manager, own sensor/bridge topics, own
    hardware shims, own MoveIt arm stack, own localization/Nav2/BT (see
    sim_bringup.launch.py).

Each namespaced robot's ign_ros2_control plugin instance gets a
<ros><namespace> tag so its controller_manager and every controller's
topics/actions live under /<ns>/... . Ignition-transport sensor topics
(which are NOT auto-scoped by model name once a sensor declares an explicit
<topic>) are likewise prefixed with <ns>/ before spawning, so robots never
share a Gazebo-side topic.

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
    """Prefix per-instance sensor topics, TF frame ids and namespace the
    ign_ros2_control plugin so a namespaced robot never collides with robot1
    or another namespaced robot. No-op when ns is empty (robot1).

    `<gz_frame_id>` (chassis_imu, the lidar) is the frame_id Gazebo stamps
    directly on the bridged message, with nothing downstream to rewrite it —
    unlike the camera/GPS/IMU shims, which take their own frame_id parameter.
    Left bare, every robot's lidar claims frame "lidar_link", and a viewer
    that watches more than one robot's /tf at once (Foxglove's 3D panel does,
    merging every tf2_msgs/TFMessage topic into one tree keyed by frame_id)
    resolves that name to whichever robot last published it — every robot's
    point cloud renders through one robot's transform. robot_state_publisher
    gets the matching `frame_prefix` in urdf.launch.py, so both sides agree
    on "<ns>/lidar_link".
    """
    if not ns:
        return content
    for topic in SENSOR_TOPICS:
        content = content.replace(
            f"<topic>{topic}</topic>", f"<topic>{qualify_gz(ns, topic)}</topic>"
        )
    content = re.sub(
        r"<gz_frame_id>([^<]+)</gz_frame_id>",
        rf"<gz_frame_id>{ns}/\1</gz_frame_id>",
        content,
    )
    content = content.replace(
        IGN_ROS2_CONTROL_PLUGIN_OPEN,
        IGN_ROS2_CONTROL_PLUGIN_OPEN
        + f"\n      <ros><namespace>{ns}</namespace></ros>",
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
    robot_count = int(LaunchConfiguration("robot_count").perform(context))
    name_prefix = LaunchConfiguration("robot_name_prefix").perform(context)
    robot1_x = float(LaunchConfiguration("robot1_x").perform(context))
    robot1_y = float(LaunchConfiguration("robot1_y").perform(context))
    robot1_z = float(LaunchConfiguration("robot1_z").perform(context))
    robot1_yaw = float(LaunchConfiguration("robot1_yaw").perform(context))
    spacing_x = float(LaunchConfiguration("robot_spacing_x").perform(context))
    spacing_y = float(LaunchConfiguration("robot_spacing_y").perform(context))
    spawn_stagger = float(LaunchConfiguration("robot_spawn_stagger").perform(context))
    use_sim_time = {"use_sim_time": True}

    robots = []
    for i in range(1, robot_count + 1):
        if i == 1 and robot_count == 1:
            # Single-robot sim stays unnamespaced — see module docstring.
            name = LaunchConfiguration("robot1_name").perform(context)
            ns = ""
        else:
            name = f"{name_prefix}{i}"
            ns = name
        robots.append(
            {
                "name": name,
                "ns": ns,
                "x": str(robot1_x + (i - 1) * spacing_x),
                "y": str(robot1_y + (i - 1) * spacing_y),
                "z": str(robot1_z),
                "yaw": str(robot1_yaw),
                # Every namespaced robot's ign_ros2_control plugin instance
                # (and controller_manager) lives inside this single
                # `ign gazebo` process; loading controller plugins for
                # several at once hits a pluginlib class-loader
                # thread-safety gap that can intermittently fail one robot's
                # controller. Staggering doesn't eliminate the race, just
                # shrinks the overlap window between any two robots.
                "spawn_delay": (i - 1) * spawn_stagger,
            }
        )

    # ── Resolve world SDF (package:// -> file://) ────────────────────────
    with open(world_path) as f:
        world_content = f.read()
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
    gz_cmd = ["ign", "gazebo", "-r"]
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
            # relative (no leading slash): RSP requests them as relative
            # topics, so an absolute "from" pattern would stop matching
            # once this node is namespaced.
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

    return actions


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("amiga_ros2_gazebo")
    default_world = os.path.join(pkg_gazebo, "worlds", "orchard_nbv.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "robot_count",
                default_value="1",
                description="How many identical Amiga+Kinova robots to spawn. "
                "With robot_count==1 that robot is unnamespaced (see module "
                "docstring); with robot_count>1 every robot, including "
                "robot1, is named/namespaced '<robot_name_prefix><i>'. make "
                "sim-dual sets this to 2, make sim-multi to $(ROBOT_COUNT).",
            ),
            DeclareLaunchArgument(
                "robot_name_prefix",
                default_value="amiga",
                description="Name/namespace prefix for robots 2..N, and for "
                "robot1 too once robot_count>1 (e.g. 'amiga' -> amiga1, "
                "amiga2, ...)",
            ),
            DeclareLaunchArgument(
                "robot1_name",
                default_value="amiga_kinova",
                description="Spawn name for robot1 when robot_count==1 only "
                "(unnamespaced single-robot sim). Ignored for robot_count>1, "
                "where robot1 is named '<robot_name_prefix>1' like every "
                "other robot.",
            ),
            DeclareLaunchArgument("robot1_x", default_value="-5.0"),
            DeclareLaunchArgument("robot1_y", default_value="-3.0"),
            DeclareLaunchArgument("robot1_z", default_value="0.05"),
            DeclareLaunchArgument("robot1_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "robot_spacing_x",
                default_value="0.0",
                description="X offset (meters) added per robot index beyond "
                "robot1 (robot i spawns at robot1_x + (i-1)*robot_spacing_x). "
                "May need tuning for large robot_count depending on world layout.",
            ),
            DeclareLaunchArgument(
                "robot_spacing_y",
                default_value="10.0",
                description="Y offset (meters) added per robot index ",
            ),
            DeclareLaunchArgument(
                "robot_spawn_stagger",
                default_value="3.0",
                description="Seconds to delay each robot's spawn+controller-"
                "loading chain behind the previous one's (robot i delays "
                "(i-1)*robot_spawn_stagger), to reduce (not eliminate) a "
                "pluginlib class-loader race when multiple robots'  "
                "controller_manager instances load plugins concurrently in "
                "the same ign gazebo process.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
