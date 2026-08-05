"""Launch every agent in this package.

    ros2 launch amiga_ros2_agents agents.launch.py
    ros2 launch amiga_ros2_agents agents.launch.py ap_vocabulary:="[at_tree_1, sampled_tree_1]"
"""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = "amiga_ros2_agents"

# world_state first, so the planner's /world_state window has frames in it by the
# time the first BT failure arrives.
AGENTS = ["world_state", "arbiter", "mission_planner", "triage"]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ap_vocabulary",
                default_value="[]",
                description="Atomic propositions ltl_gen may use. Empty = model invents them.",
            ),
            *(
                Node(package=PACKAGE, executable=agent, output="screen")
                for agent in AGENTS
            ),
            Node(
                package=PACKAGE,
                executable="ltl_gen",
                output="screen",
                parameters=[
                    {
                        "ap_vocabulary": ParameterValue(
                            LaunchConfiguration("ap_vocabulary"), value_type=List[str]
                        )
                    }
                ],
            ),
        ]
    )
