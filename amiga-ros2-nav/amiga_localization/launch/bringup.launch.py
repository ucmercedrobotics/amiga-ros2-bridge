from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_vectornav = LaunchConfiguration("use_vectornav")
    use_gps = LaunchConfiguration("use_gps")
    gps_topic = LaunchConfiguration("gps_topic")
    namespace = LaunchConfiguration("namespace")

    launch_dir = os.path.join(
        get_package_share_directory("amiga_localization"), "launch"
    )

    wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "wheel_odom.launch.py")),
        launch_arguments={"namespace": namespace}.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "ekf.launch.py")),
        launch_arguments={
            "use_vectornav": use_vectornav,
            "use_gps": use_gps,
            "gps_topic": gps_topic,
            "namespace": namespace,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_vectornav",
                default_value="false",
                description="Use vectornav_ekf.yaml (true) or base_ekf.yaml (false)",
            ),
            DeclareLaunchArgument(
                "use_gps",
                default_value="true",
                description="Enable GPS/navsat_transform_node (true) or disable (false)",
            ),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="/ublox_gps_node/fix",
                description="GPS fix topic to remap to /gps/fix",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace for the whole localization stack (per-robot, e.g. 'amiga2')",
            ),
            wheel_odom_launch,
            ekf_launch,
        ]
    )
