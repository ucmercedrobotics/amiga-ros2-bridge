from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _p(ns, topic):
    """Absolute topic name, namespaced under `ns` (ns="" leaves it unchanged)."""
    topic = topic.lstrip("/")
    return f"/{ns}/{topic}" if ns else f"/{topic}"


def launch_setup(context, *args, **kwargs):
    use_vectornav = LaunchConfiguration("use_vectornav").perform(context).lower() == "true"
    use_gps = LaunchConfiguration("use_gps")
    gps_topic = LaunchConfiguration("gps_topic")
    ns = LaunchConfiguration("namespace").perform(context)

    pkg_share = get_package_share_directory("amiga_localization")
    ekf_config_path = os.path.join(
        pkg_share, "config", "vectornav_ekf.yaml" if use_vectornav else "base_ekf.yaml"
    )

    # tf2_ros hardcodes /tf, /tf_static as absolute regardless of node
    # namespace, so both EKF publish_tf:true nodes need an explicit remap
    # (same quirk already handled for the sim side in amiga_ros2_gazebo).
    # imu0 (/bno085/imu or /vectornav/imu) is baked into the yaml as an
    # absolute string, but ROS 2 remap rules match the resolved runtime
    # topic name regardless of whether it came from a param or a literal,
    # so remapping it here still works.
    tf_remaps = [
        ("/tf", _p(ns, "tf")),
        ("/tf_static", _p(ns, "tf_static")),
    ]
    imu_remaps = [
        ("/bno085/imu", _p(ns, "bno085/imu")),
        ("/vectornav/imu", _p(ns, "vectornav/imu")),
    ]

    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local_filter_node",
            namespace=ns,
            output="screen",
            parameters=[ekf_config_path],
            remappings=[("odometry/filtered", "odometry/filtered/local")]
            + tf_remaps
            + imu_remaps,
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global_filter_node",
            namespace=ns,
            output="screen",
            parameters=[ekf_config_path],
            remappings=[("odometry/filtered", "odometry/filtered/global")]
            + tf_remaps
            + imu_remaps,
        ),
        # GPS node - only launched if use_gps=true
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            namespace=ns,
            output="screen",
            parameters=[ekf_config_path],
            remappings=[
                # -- Inputs
                ("odometry/filtered", "odometry/filtered/global"),
                # Relative "gps/fix" (not "/gps/fix"): navsat_transform_node
                # subscribes to a RELATIVE "gps/fix" topic internally, which
                # only resolves to "/gps/fix" when ns="" (root) — for a
                # namespaced robot it resolves to "/<ns>/gps/fix" before
                # remapping, so an absolute "from" pattern here would
                # silently never match (same class of bug already found in
                # move_group's "joint_states" and gz_description_server's
                # "robot_description").
                ("gps/fix", gps_topic),
            ]
            + tf_remaps,
            condition=IfCondition(use_gps),
        ),
    ]


def generate_launch_description():
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
                default_value="/gps/pvt",
                description="GPS fix topic to remap to /gps/fix",
            ),
            DeclareLaunchArgument("output_final_position", default_value="false"),
            DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace for the EKF/navsat nodes (per-robot, e.g. 'amiga2')",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
