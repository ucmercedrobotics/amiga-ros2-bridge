from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            # TCP Demux parameters
            DeclareLaunchArgument(
                "port",
                default_value="12346",
                description="TCP server port for tcp_demux_node",
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
            DeclareLaunchArgument(
                "mission_topic",
                default_value="/mission/xml",
                description="Topic to publish mission XML",
            ),
            DeclareLaunchArgument(
                "orchard_topic",
                default_value="/orchard/tree_info_json",
                description="Topic to publish orchard JSON",
            ),

            Node(
                package="amiga_ros2_behavior_tree",
                executable="bt_runner",
                name="bt_runner",
                output="screen",
            ),
            Node(
                package="amiga_ros2_behavior_tree",
                executable="tcp_demux_node",
                name="tcp_demux",
                output="screen",
                parameters=[
                    {
                        "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                        "expect_json": ParameterValue(LaunchConfiguration("expect_json"), value_type=bool),
                        "payload_length_included": ParameterValue(
                            LaunchConfiguration("payload_length_included"), value_type=bool
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
                output="screen",
                condition=IfCondition(LaunchConfiguration("expect_json")),
            ),
        ]
    )
