from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _int_arg(name):
    # Launch substitutions are strings; the node declares these as integers, so
    # the type has to be stated or the parameter set is rejected at startup.
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    # Only the parameters that differ per robot are surfaced as launch args;
    # the rest keep the node's defaults. See docs/lora_frame_contract.md for
    # what baud and max_payload_bytes are assuming.
    args = [
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("max_payload_bytes", default_value="200"),
        DeclareLaunchArgument("tx_queue_depth", default_value="32"),
        DeclareLaunchArgument("rx_queue_depth", default_value="64"),
        DeclareLaunchArgument("tx_overflow_policy", default_value="drop_oldest"),
    ]

    return LaunchDescription(
        args
        + [
            Node(
                package="amiga_ros2_comms",
                executable="lora_bridge",
                name="lora_bridge",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "baud": _int_arg("baud"),
                        "max_payload_bytes": _int_arg("max_payload_bytes"),
                        "tx_queue_depth": _int_arg("tx_queue_depth"),
                        "rx_queue_depth": _int_arg("rx_queue_depth"),
                        "tx_overflow_policy": LaunchConfiguration("tx_overflow_policy"),
                    }
                ],
            )
        ]
    )
