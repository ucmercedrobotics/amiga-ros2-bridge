from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _typed_arg(name, value_type):
    # Launch substitutions are strings; the node declares these as int or
    # float, so the type has to be stated or the parameter set is rejected at
    # startup.
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    # node_id has no default worth shipping: it is this robot's fleet-unique
    # address, and two robots sharing one silently dedup each other's traffic
    # away. The node refuses to start on a bad value rather than guessing.
    #
    # spreading_factor is not used to configure anything here -- it only lets
    # the node check retransmit_timeout_sec against the round-trip time on air
    # and warn if the timeout is short enough to cause a retransmit storm. Keep
    # it matching the radio (or the lora_sim setting) for the check to be
    # meaningful.
    args = [
        DeclareLaunchArgument(
            "node_id", description="This robot's ID, 1..255. Must be fleet-unique."
        ),
        DeclareLaunchArgument("retransmit_timeout_sec", default_value="3.0"),
        DeclareLaunchArgument("max_retries", default_value="3"),
        DeclareLaunchArgument("dedup_ttl_sec", default_value="120.0"),
        DeclareLaunchArgument("spreading_factor", default_value="7"),
    ]

    return LaunchDescription(
        args
        + [
            Node(
                package="amiga_ros2_comms",
                executable="lora_reliability",
                name="lora_reliability",
                output="screen",
                parameters=[
                    {
                        "node_id": _typed_arg("node_id", int),
                        "retransmit_timeout_sec": _typed_arg(
                            "retransmit_timeout_sec", float
                        ),
                        "max_retries": _typed_arg("max_retries", int),
                        "dedup_ttl_sec": _typed_arg("dedup_ttl_sec", float),
                        "spreading_factor": _typed_arg("spreading_factor", int),
                    }
                ],
            )
        ]
    )
