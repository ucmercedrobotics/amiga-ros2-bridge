from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # -- Amiga Bridge
    bridge_launch_dir = os.path.join(
        get_package_share_directory("amiga_ros2_bridge"), "launch"
    )
    amiga_streams_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_launch_dir, "amiga_streams.launch.py")
        )
    )
    twist_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_launch_dir, "twist_control.launch.py")
        )
    )

    # -- Camera
    oakd_launch_dir = os.path.join(
        get_package_share_directory("amiga_ros2_oakd"), "launch"
    )
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(oakd_launch_dir, "amiga_cameras.launch.py")
        )
    )

    # -- Description
    urdf_launch_dir = os.path.join(
        get_package_share_directory("amiga_ros2_description"), "launch"
    )
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(urdf_launch_dir, "urdf.launch.py"))
    )

    # -- GPS/Localization
    loc_launch_dir = os.path.join(
        get_package_share_directory("amiga_localization"), "launch"
    )
    loc_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(loc_launch_dir, "bringup.launch.py"))
    )

    # -- Navigation
    nav_launch_dir = os.path.join(
        get_package_share_directory("amiga_navigation"), "launch"
    )
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, "navigation.launch.py")
        )
    )
    # Waypoint follower
    waypoint_follower = Node(
        package="amiga_navigation",
        executable="waypoint_follower.py",
        name="waypoint_follower",
        output="screen",
    )

    return LaunchDescription(
        [
            # Amiga bridge
            amiga_streams_launch,
            twist_control_launch,
            # Camera
            oakd_launch,
            # URDF
            urdf_launch,
            # GPS/localization
            loc_bringup_launch,
            # Nav2
            nav_launch,
            waypoint_follower,
        ]
    )
