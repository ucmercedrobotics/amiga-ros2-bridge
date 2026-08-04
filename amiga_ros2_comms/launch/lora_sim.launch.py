"""Bring up a virtual LoRa medium, optionally with a bridge per robot.

    ros2 launch amiga_ros2_comms lora_sim.launch.py robots:=robot1,robot2,robot3

With bridges:=true (the default) each robot gets a lora_bridge in its own
namespace, so a fleet talks over /<robot>/lora/tx and /<robot>/lora/rx exactly
as it would with real radios. Set bridges:=false when the robots' own bringup
launches their bridges, and just point each one at <symlink_dir>/<robot>.

Launch order does not matter: a bridge whose port does not exist yet keeps
retrying until the sim creates it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument(
        "robots",
        default_value="robot1,robot2,robot3",
        description="Comma-separated robot names; one virtual radio each.",
    ),
    DeclareLaunchArgument("symlink_dir", default_value="/tmp/amiga_lora_sim"),
    DeclareLaunchArgument("spreading_factor", default_value="7"),
    DeclareLaunchArgument("bandwidth_hz", default_value="125000"),
    DeclareLaunchArgument("max_payload_bytes", default_value="200"),
    DeclareLaunchArgument(
        "bridges",
        default_value="true",
        description="Also start a lora_bridge per robot, namespaced by robot name.",
    ),
]


def _typed(name, value_type):
    # Launch substitutions are strings; the nodes declare these as int, float
    # and bool, so the type has to be stated or the parameter set is rejected.
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def _spawn(context, *args, **kwargs):
    # The robot list is only known once substitutions resolve, so the per-robot
    # nodes have to be built here rather than at description time.
    robots = [
        name.strip()
        for name in LaunchConfiguration("robots").perform(context).split(",")
        if name.strip()
    ]
    symlink_dir = LaunchConfiguration("symlink_dir").perform(context)
    want_bridges = (
        LaunchConfiguration("bridges").perform(context).lower() in ("true", "1", "yes")
    )

    nodes = [
        Node(
            package="amiga_ros2_comms",
            executable="lora_sim",
            name="lora_sim",
            output="screen",
            parameters=[
                {
                    "robots": robots,
                    "symlink_dir": symlink_dir,
                    "spreading_factor": _typed("spreading_factor", int),
                    "bandwidth_hz": _typed("bandwidth_hz", int),
                }
            ],
        )
    ]

    if want_bridges:
        nodes += [
            Node(
                package="amiga_ros2_comms",
                executable="lora_bridge",
                name="lora_bridge",
                namespace=robot,
                output="screen",
                parameters=[
                    {
                        "serial_port": f"{symlink_dir}/{robot}",
                        "max_payload_bytes": _typed("max_payload_bytes", int),
                    }
                ],
            )
            for robot in robots
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=_spawn)])
