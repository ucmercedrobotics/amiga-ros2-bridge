"""Adapters that make the simulator impersonate the real hardware layer.

Each shim republishes a raw Gazebo-bridged topic under the exact topic name,
frame_id and encoding the corresponding hardware driver uses:

  sim_twist_control   /cmd_vel -> diff_drive_controller   (farm-ng canbus cmd)
  sim_canbus_twist    diff_drive odom -> /canbus/twist    (farm-ng canbus fb)
  sim_gps_shim        /navsat -> /gps/pvt                 (farm-ng gps)
  sim_imu_shim        /chassis/imu -> /bno085/imu          (BNO085 driver)
  camera shims        gz rgbd -> depthai / kinova_vision topics

robot1's shims are completely unnamespaced (byte-for-byte the original
single-robot topic set — amiga_localization/amiga_navigation/
amiga_ros2_behavior_tree hardcode absolute paths like /bno085/imu and
/oak0/points and historically targeted a single unnamespaced robot, see
gazebo.launch.py's docstring). Every additional robot's shims (built for
i=2..robot_count) are fully namespaced so they never collide with robot1's
or each other's.

Downstream consumers (wheel odometry, EKFs, Nav2, BTs, kortex_vision) run
with configs identical to the real robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def qualify_ros(ns, topic):
    """Absolute topic name, namespaced under `ns` (ns="" leaves it unchanged)."""
    topic = topic.lstrip("/")
    return f"/{ns}/{topic}" if ns else f"/{topic}"


def build_shims(ns, imu_topic, imu_frame):
    use_sim_time = {"use_sim_time": True}

    return [
        Node(
            package="amiga_ros2_gazebo",
            executable="sim_twist_control.py",
            name="sim_twist_control",
            namespace=ns,
            output="screen",
            parameters=[use_sim_time],
            remappings=[
                ("/cmd_vel", qualify_ros(ns, "/cmd_vel")),
                (
                    "/diff_drive_controller/cmd_vel_unstamped",
                    qualify_ros(ns, "/diff_drive_controller/cmd_vel_unstamped"),
                ),
            ],
        ),
        Node(
            package="amiga_ros2_gazebo",
            executable="sim_canbus_twist.py",
            name="sim_canbus_twist",
            namespace=ns,
            output="screen",
            parameters=[use_sim_time],
            remappings=[
                (
                    "/diff_drive_controller/odom",
                    qualify_ros(ns, "/diff_drive_controller/odom"),
                ),
                ("/canbus/twist", qualify_ros(ns, "/canbus/twist")),
            ],
        ),
        Node(
            package="amiga_ros2_gazebo",
            executable="sim_gps_shim.py",
            name="sim_gps_shim",
            namespace=ns,
            output="screen",
            parameters=[
                use_sim_time,
                {
                    "input_topic": qualify_ros(ns, "/navsat"),
                    "output_topic": qualify_ros(ns, "/gps/pvt"),
                },
            ],
        ),
        Node(
            package="amiga_ros2_gazebo",
            executable="sim_imu_shim.py",
            name="sim_imu_shim",
            namespace=ns,
            output="screen",
            parameters=[
                use_sim_time,
                {
                    "input_topic": qualify_ros(ns, "/chassis/imu"),
                    "output_topic": qualify_ros(ns, imu_topic),
                    "frame_id": imu_frame,
                },
            ],
        ),
        # oak0 (front camera) -> depthai topics
        Node(
            package="amiga_ros2_gazebo",
            executable="sim_camera_shim.py",
            name="sim_oak0_shim",
            namespace=ns,
            output="screen",
            parameters=[
                use_sim_time,
                {
                    "rgb_in": qualify_ros(ns, "/oak_camera_front/image"),
                    "rgb_out": qualify_ros(ns, "/oak0/rgb/image_raw"),
                    "depth_in": qualify_ros(ns, "/oak_camera_front/depth_image"),
                    "depth_out": qualify_ros(ns, "/oak0/stereo/image_raw"),
                    "info_in": qualify_ros(ns, "/oak_camera_front/camera_info"),
                    "rgb_info_out": qualify_ros(ns, "/oak0/rgb/camera_info"),
                    "depth_info_out": qualify_ros(ns, "/oak0/stereo/camera_info"),
                    "points_in": qualify_ros(ns, "/oak_camera_front/points"),
                    "points_out": qualify_ros(ns, "/oak0/points"),
                    "rgb_frame_id": "oak0_rgb_camera_frame",
                    "depth_to_mm": True,
                },
            ],
        ),
        # oak1 (back camera) -> depthai topics
        Node(
            package="amiga_ros2_gazebo",
            executable="sim_camera_shim.py",
            name="sim_oak1_shim",
            namespace=ns,
            output="screen",
            parameters=[
                use_sim_time,
                {
                    "rgb_in": qualify_ros(ns, "/oak_camera_back/image"),
                    "rgb_out": qualify_ros(ns, "/oak1/rgb/image_raw"),
                    "depth_in": qualify_ros(ns, "/oak_camera_back/depth_image"),
                    "depth_out": qualify_ros(ns, "/oak1/stereo/image_raw"),
                    "info_in": qualify_ros(ns, "/oak_camera_back/camera_info"),
                    "rgb_info_out": qualify_ros(ns, "/oak1/rgb/camera_info"),
                    "depth_info_out": qualify_ros(ns, "/oak1/stereo/camera_info"),
                    "points_in": qualify_ros(ns, "/oak_camera_back/points"),
                    "points_out": qualify_ros(ns, "/oak1/points"),
                    "rgb_frame_id": "oak1_rgb_camera_frame",
                    "depth_to_mm": True,
                },
            ],
        ),
        # Kinova wrist camera -> kinova_vision topics
        Node(
            package="amiga_ros2_gazebo",
            executable="sim_camera_shim.py",
            name="sim_wrist_camera_shim",
            namespace=ns,
            output="screen",
            parameters=[
                use_sim_time,
                {
                    "rgb_in": qualify_ros(ns, "/realsense/image"),
                    "rgb_out": qualify_ros(ns, "/camera/color/image_raw"),
                    "depth_in": qualify_ros(ns, "/realsense/depth_image"),
                    "depth_out": qualify_ros(ns, "/camera/depth/image_raw"),
                    "info_in": qualify_ros(ns, "/realsense/camera_info"),
                    "rgb_info_out": qualify_ros(ns, "/camera/color/camera_info"),
                    "depth_info_out": qualify_ros(ns, "/camera/depth/camera_info"),
                    "rgb_frame_id": "camera_color_frame",
                    "depth_frame_id": "camera_depth_frame",
                    "depth_to_mm": True,
                },
            ],
        ),
    ]


def launch_setup(context, *args, **kwargs):
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    imu_frame = LaunchConfiguration("imu_frame").perform(context)
    robot_count = int(LaunchConfiguration("robot_count").perform(context))
    name_prefix = LaunchConfiguration("robot_name_prefix").perform(context)

    shims = build_shims("", imu_topic, imu_frame)
    for i in range(2, robot_count + 1):
        shims += build_shims(f"{name_prefix}{i}", imu_topic, imu_frame)
    return shims


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_count",
                default_value="1",
                description="Build shims for this many robots (see module docstring)",
            ),
            DeclareLaunchArgument("robot_name_prefix", default_value="amiga"),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/bno085/imu",
                description="Bridge-native IMU output topic (matches the EKF config). "
                "robot1 publishes this unprefixed; robot i>=2 publishes it "
                "under /<robot_name_prefix><i>/",
            ),
            DeclareLaunchArgument(
                "imu_frame",
                default_value="bno085",
                description="frame_id stamped on IMU messages",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
