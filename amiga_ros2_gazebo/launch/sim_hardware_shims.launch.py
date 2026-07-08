"""Adapters that make the simulator impersonate the real hardware layer.

Each shim republishes a raw Gazebo-bridged topic under the exact topic name,
frame_id and encoding the corresponding hardware driver uses:

  sim_twist_control   /cmd_vel -> diff_drive_controller   (farm-ng canbus cmd)
  sim_canbus_twist    diff_drive odom -> /canbus/twist    (farm-ng canbus fb)
  sim_gps_shim        /navsat -> /gps/pvt                 (farm-ng gps)
  sim_imu_shim        /chassis/imu -> /bno085/imu         (BNO085 driver)
  camera shims        gz rgbd -> depthai / kinova_vision topics

Downstream consumers (wheel odometry, EKFs, Nav2, BTs, kortex_vision) run
with configs identical to the real robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/bno085/imu",
                description="Bridge-native IMU output topic (matches the EKF config)",
            ),
            DeclareLaunchArgument(
                "imu_frame",
                default_value="bno085",
                description="frame_id stamped on IMU messages",
            ),
            Node(
                package="amiga_ros2_gazebo",
                executable="sim_twist_control.py",
                name="sim_twist_control",
                output="screen",
                parameters=[use_sim_time],
            ),
            Node(
                package="amiga_ros2_gazebo",
                executable="sim_canbus_twist.py",
                name="sim_canbus_twist",
                output="screen",
                parameters=[use_sim_time],
            ),
            Node(
                package="amiga_ros2_gazebo",
                executable="sim_gps_shim.py",
                name="sim_gps_shim",
                output="screen",
                parameters=[use_sim_time],
            ),
            Node(
                package="amiga_ros2_gazebo",
                executable="sim_imu_shim.py",
                name="sim_imu_shim",
                output="screen",
                parameters=[
                    use_sim_time,
                    {
                        "output_topic": LaunchConfiguration("imu_topic"),
                        "frame_id": LaunchConfiguration("imu_frame"),
                    },
                ],
            ),
            # oak0 (front camera) -> depthai topics
            Node(
                package="amiga_ros2_gazebo",
                executable="sim_camera_shim.py",
                name="sim_oak0_shim",
                output="screen",
                parameters=[
                    use_sim_time,
                    {
                        "rgb_in": "/oak_camera_front/image",
                        "rgb_out": "/oak0/rgb/image_raw",
                        "depth_in": "/oak_camera_front/depth_image",
                        "depth_out": "/oak0/stereo/image_raw",
                        "info_in": "/oak_camera_front/camera_info",
                        "rgb_info_out": "/oak0/rgb/camera_info",
                        "depth_info_out": "/oak0/stereo/camera_info",
                        "points_in": "/oak_camera_front/points",
                        "points_out": "/oak0/points",
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
                output="screen",
                parameters=[
                    use_sim_time,
                    {
                        "rgb_in": "/oak_camera_back/image",
                        "rgb_out": "/oak1/rgb/image_raw",
                        "depth_in": "/oak_camera_back/depth_image",
                        "depth_out": "/oak1/stereo/image_raw",
                        "info_in": "/oak_camera_back/camera_info",
                        "rgb_info_out": "/oak1/rgb/camera_info",
                        "depth_info_out": "/oak1/stereo/camera_info",
                        "points_in": "/oak_camera_back/points",
                        "points_out": "/oak1/points",
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
                output="screen",
                parameters=[
                    use_sim_time,
                    {
                        "rgb_in": "/realsense/image",
                        "rgb_out": "/camera/color/image_raw",
                        "depth_in": "/realsense/depth_image",
                        "depth_out": "/camera/depth/image_raw",
                        "info_in": "/realsense/camera_info",
                        "rgb_info_out": "/camera/color/camera_info",
                        "depth_info_out": "/camera/depth/camera_info",
                        "rgb_frame_id": "camera_color_frame",
                        "depth_frame_id": "camera_depth_frame",
                        "depth_to_mm": True,
                    },
                ],
            ),
        ]
    )
