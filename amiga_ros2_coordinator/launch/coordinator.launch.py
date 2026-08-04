from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _typed_arg(name, value_type):
    # Launch substitutions are strings; the node declares these as int or
    # float, so the type has to be stated or the parameter set is rejected at
    # startup.
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def _launch_setup(context, *args, **kwargs):
    # `capabilities` is a STRING_ARRAY parameter on the node, and there is no
    # substitution that turns "DRIVE,SPRAY" into one. Resolving the argument
    # here and splitting it in Python is what an OpaqueFunction is for.
    capabilities = [
        name.strip().upper()
        for name in LaunchConfiguration("capabilities").perform(context).split(",")
        if name.strip()
    ]

    # Deliberately no `name=`: this executable runs *two* nodes in one process
    # (the coordinator and its in-process reliability layer), and a name
    # remapping would rename both of them to the same thing. Each node keeps
    # the name it gives itself.
    #
    # Parameters passed here reach every node in the process, which is exactly
    # what is wanted: node_id and the retransmit settings are consumed by the
    # reliability node and ignored by the coordinator, and the coordination
    # settings the other way around.
    return [
        Node(
            package="amiga_ros2_coordinator",
            executable="coordinator",
            output="screen",
            parameters=[
                {
                    # Consumed by the in-process ReliabilityNode.
                    "node_id": _typed_arg("node_id", int),
                    "retransmit_timeout_sec": _typed_arg(
                        "retransmit_timeout_sec", float
                    ),
                    "spreading_factor": _typed_arg("spreading_factor", int),
                    # Consumed by the coordinator.
                    "capabilities": capabilities,
                    "announce_window_sec": _typed_arg("announce_window_sec", float),
                    "bid_max_backoff_sec": _typed_arg("bid_max_backoff_sec", float),
                    "peer_timeout_sec": _typed_arg("peer_timeout_sec", float),
                    "heartbeat_period_sec": _typed_arg("heartbeat_period_sec", float),
                }
            ],
        )
    ]


def generate_launch_description():
    # One process, two nodes: the `coordinator` executable starts its own
    # ReliabilityNode and adds both to one executor. Do not run this together
    # with lora_reliability.launch.py -- two reliability layers on one radio
    # would each ACK the other's inbound traffic and duplicate every send.
    #
    # node_id has no default worth shipping: it is this robot's fleet-unique
    # address, and two robots sharing one silently dedup each other's traffic
    # away.
    #
    # announce_window_sec must exceed bid_max_backoff_sec across the whole
    # fleet, or the best-fitting bidder transmits after the auction it was
    # answering has already closed. The node refuses to start if its own pair
    # is incoherent.
    args = [
        DeclareLaunchArgument(
            "node_id", description="This robot's ID, 1..255. Must be fleet-unique."
        ),
        DeclareLaunchArgument(
            "capabilities",
            default_value="DRIVE",
            description="Comma-separated Capability names this robot advertises.",
        ),
        DeclareLaunchArgument("announce_window_sec", default_value="5.0"),
        DeclareLaunchArgument("bid_max_backoff_sec", default_value="2.0"),
        DeclareLaunchArgument("peer_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("heartbeat_period_sec", default_value="10.0"),
        DeclareLaunchArgument("retransmit_timeout_sec", default_value="3.0"),
        DeclareLaunchArgument("spreading_factor", default_value="7"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
