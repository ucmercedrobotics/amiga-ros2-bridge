"""The mission layer: the tree runner, the TCP mission intake and the orchard.

    ros2 launch amiga_ros2_behavior_tree bt.launch.py
    ros2 launch amiga_ros2_behavior_tree bt.launch.py namespace:=amiga2 port:=12347

`namespace` puts all three nodes under one robot, which is what a simulated
fleet needs and what `sim_bringup.launch.py` passes. Two things make that work:

  * The topic parameters default to *relative* names. A relative name resolves
    against the node's own namespace, so `mission/xml` is `/mission/xml` for an
    unnamespaced robot and `/amiga2/mission/xml` under a namespace, with no
    caller cooperation and no change to the single-robot layout.
  * The mission XML names its actions relatively too (`action_name=
    "follow_tree_id_waypoint"`), and BehaviorTree.ROS2 resolves those against
    bt_runner's node handle -- so a namespaced runner drives its own robot's
    action servers rather than robot 1's.

`port` has to differ per robot: SO_REUSEPORT means two demuxes on one port
would silently split mission connections between robots instead of failing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Robot namespace for all three nodes. Empty is the "
                "single-robot layout, byte-identical to what it was before this "
                "argument existed.",
            ),
            # TCP Demux parameters
            DeclareLaunchArgument(
                "port",
                default_value="12346",
                description="TCP server port for tcp_demux_node",
            ),
            DeclareLaunchArgument(
                "xml_validation",
                default_value="true",
                description="Whether to validate incoming XML against the mission schema",
            ),
            DeclareLaunchArgument(
                "payload_length_included",
                default_value="true",
                description="Whether payload frames include a 4-byte length prefix",
            ),
            DeclareLaunchArgument(
                "expect_json",
                default_value="true",
                description="Whether a JSON frame is expected after the XML frame",
            ),
            # The three topic names are relative on purpose -- see the module
            # docstring. Passing an absolute name here still works and opts out.
            DeclareLaunchArgument(
                "mission_topic",
                default_value="mission/xml",
                description="Topic to publish mission XML",
            ),
            DeclareLaunchArgument(
                "orchard_topic",
                default_value="orchard/tree_info_json",
                description="Topic to publish orchard JSON",
            ),
            DeclareLaunchArgument(
                "fault_topic",
                default_value="bt/status_change",
                description="Topic the tree reports leaf failures on. The "
                "catalyst for the whole replanning and coordination pipeline.",
            ),
            DeclareLaunchArgument(
                "x_offset",
                default_value="0.0",
                description="Offset in meters to add to longitude (east/west)",
            ),
            DeclareLaunchArgument(
                "y_offset",
                default_value="0.0",
                description="Offset in meters to add to latitude (north/south)",
            ),
            Node(
                package="amiga_ros2_behavior_tree",
                executable="bt_runner",
                name="bt_runner",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "mission_topic": LaunchConfiguration("mission_topic"),
                        "fault_topic": LaunchConfiguration("fault_topic"),
                        "xml_validation": LaunchConfiguration("xml_validation"),
                    }
                ],
            ),
            Node(
                package="amiga_ros2_behavior_tree",
                executable="tcp_demux_node",
                name="tcp_demux",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "port": ParameterValue(
                            LaunchConfiguration("port"), value_type=int
                        ),
                        "expect_json": ParameterValue(
                            LaunchConfiguration("expect_json"), value_type=bool
                        ),
                        "payload_length_included": ParameterValue(
                            LaunchConfiguration("payload_length_included"),
                            value_type=bool,
                        ),
                        "mission_topic": LaunchConfiguration("mission_topic"),
                        "orchard_topic": LaunchConfiguration("orchard_topic"),
                    }
                ],
            ),
            Node(
                package="amiga_ros2_behavior_tree",
                executable="orchard_management_node",
                name="orchard_management",
                namespace=namespace,
                output="screen",
                condition=IfCondition(LaunchConfiguration("expect_json")),
                parameters=[
                    {
                        "json_topic": LaunchConfiguration("orchard_topic"),
                        "x_offset": LaunchConfiguration("x_offset"),
                        "y_offset": LaunchConfiguration("y_offset"),
                    }
                ],
                # orchard_management.cpp names its service absolutely. Remapping
                # to the relative name lets the node's namespace place it, which
                # is what amiga_navigation's waypoint_follower already expects
                # (it dials `<ns>/orchard/get_tree_info`). No-op when ns is "".
                remappings=[("/orchard/get_tree_info", "orchard/get_tree_info")],
            ),
        ]
    )
