"""Gazebo (Ignition Fortress) bringup for the Amiga + Kinova simulator.

Starts:
  * ign gazebo with the orchard world (levels enabled)
  * a TF-muted robot_state_publisher ("gz_description_server") that only
    serves the combined sim URDF to the ign_ros2_control plugin
  * robot spawn + ros_gz parameter bridge
  * levels performer registration
  * controller spawners chained on process exit:
    joint_state_broadcaster -> diff_drive_controller ->
    joint_trajectory_controller -> robotiq_gripper_controller

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


def launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory("amiga_ros2_gazebo")
    pkg_descr = get_package_share_directory("amiga_ros2_description")

    world_path = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    use_sim_time = {"use_sim_time": True}

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

    # ── Resolve robot model SDF ──────────────────────────────────────────
    model_path = os.path.join(pkg_gazebo, "models", "amiga_kinova", "model.sdf")
    controllers_yaml = os.path.join(pkg_gazebo, "config", "ros2_controllers_sim.yaml")
    with open(model_path) as f:
        model_content = f.read()
    model_content = model_content.replace(
        "$(find-pkg-share amiga_ros2_gazebo)/config/ros2_controllers_sim.yaml",
        controllers_yaml,
    )
    tmp_model = tempfile.NamedTemporaryFile(
        mode="w", suffix=".sdf", prefix="amiga_kinova_resolved_", delete=False
    )
    tmp_model.write(resolve_package_uris(model_content))
    tmp_model.flush()

    # ── Combined sim URDF for ign_ros2_control (TF/description muted) ────
    xacro_path = os.path.join(pkg_descr, "urdf", "sim", "amiga_kinova_sim.urdf.xacro")
    result = subprocess.run(
        ["xacro", xacro_path, "sim_ignition:=true"], capture_output=True, text=True
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed for {xacro_path}:\n{result.stderr}")
    sim_robot_description = result.stdout

    gz_description_server = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="gz_description_server",
        output="screen",
        parameters=[use_sim_time, {"robot_description": sim_robot_description}],
        remappings=[
            ("/tf", "/gz_sim/tf"),
            ("/tf_static", "/gz_sim/tf_static"),
            ("/robot_description", "/gz_sim/robot_description"),
            ("joint_states", "/gz_sim/joint_states"),
        ],
    )

    # ── Gazebo ───────────────────────────────────────────────────────────
    gz_cmd = ["ign", "gazebo", "-r", "--levels"]
    if headless:
        gz_cmd += ["-s", "--headless-rendering"]
    gz_cmd.append(tmp_world.name)
    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env={"IGN_GAZEBO_SYSTEM_PLUGIN_PATH": "/opt/ros/humble/lib"},
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file", tmp_model.name,
            "-name", "amiga_kinova",
            "-x", spawn_x, "-y", spawn_y, "-z", spawn_z, "-Y", spawn_yaw,
        ],
        output="screen",
        parameters=[use_sim_time],
    )

    # ── gz <-> ROS bridge (raw sim topics; shims re-publish bridge-native) ─
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        output="screen",
        parameters=[use_sim_time],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
            "/chassis/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # oak0 (front)
            "/oak_camera_front/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/oak_camera_front/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/oak_camera_front/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/oak_camera_front/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            # oak1 (back)
            "/oak_camera_back/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/oak_camera_back/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/oak_camera_back/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/oak_camera_back/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            # Kinova wrist camera
            "/realsense/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/realsense/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/realsense/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            # 2D lidar (frame lidar_link matches the amiga URDF when use_lidar:=true)
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        ],
    )

    # ── Register robot as levels performer ───────────────────────────────
    set_performer = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ign", "service",
                    "-s", f"/world/{world_name}/level/set_performer",
                    "--reqtype", "ignition.msgs.StringMsg",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "2000",
                    "--req", 'data: "amiga_kinova"',
                ],
                output="screen",
            )
        ],
    )

    # ── Controller spawners, chained after the robot spawn exits ─────────
    def spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller,
                "-c", "/controller_manager",
                "--controller-manager-timeout", "120",
            ],
            output="screen",
            parameters=[use_sim_time],
        )

    spawn_jsb = spawner("joint_state_broadcaster")
    spawn_diff = spawner("diff_drive_controller")
    spawn_jtc = spawner("joint_trajectory_controller")
    spawn_gripper = spawner("robotiq_gripper_controller")

    chain = [
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_diff])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_diff, on_exit=[spawn_jtc])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jtc, on_exit=[spawn_gripper])),
    ]

    return [gazebo, gz_description_server, spawn, bridge, set_performer] + chain


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("amiga_ros2_gazebo")
    default_world = os.path.join(pkg_gazebo, "worlds", "orchard_nbv.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("spawn_x", default_value="-5.0"),
            DeclareLaunchArgument("spawn_y", default_value="-3.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.05"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
