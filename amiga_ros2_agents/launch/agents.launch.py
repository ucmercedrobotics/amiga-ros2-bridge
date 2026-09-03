"""Launch every agent in this package.

    ros2 launch amiga_ros2_agents agents.launch.py
    ros2 launch amiga_ros2_agents agents.launch.py launch_vlm:=true \\
        vlm_url:=http://localhost:8001/v1/chat/completions

This is the one the real machine runs; `robot_agents.launch.py` is the same set
under a namespace, for a simulated fleet. The two are kept in step by hand, so
an agent or a flag added to one belongs in the other -- `launch_vlm` below
mirrors the block there.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = "amiga_ros2_agents"

# world_state first, so the planner's /world_state window has frames in it by the
# time the first BT failure arrives.
AGENTS = ["world_state", "arbiter", "mission_planner", "triage", "note"]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_vlm",
                default_value="false",
                description="Start vlm_server. Every behaviour-tree failure "
                "then carries a description of what the camera saw at that "
                "moment into triage's decisions -- which is what separates a "
                "dead sensor from a robot parked in the wrong place, two "
                "faults the logs describe identically. Needs a vision model on "
                "vlm_url: a separate model and endpoint from the one the "
                "agents reason with. Off by default.",
            ),
            DeclareLaunchArgument(
                "vlm_url",
                default_value="http://localhost:8001/v1/chat/completions",
                description="OpenAI-compatible chat-completions endpoint that "
                "accepts image content parts.",
            ),
            DeclareLaunchArgument(
                "vlm_image_topic",
                default_value="/oak0/rgb/image_raw",
                description="Camera the VLM looks through. The Oak-D front "
                "camera by default -- what amiga_ros2_oakd publishes, and what "
                "the Gazebo shim republishes under the same name.",
            ),
            *(
                Node(
                    package=PACKAGE,
                    executable=agent,
                    output="screen",
                    # Only triage declares this; a parameter a node never
                    # declares is ignored, so it costs nothing to pass to all
                    # of them. Tied to the flag that starts the server, so a
                    # triage told to look always has something to look through.
                    parameters=[
                        {
                            "use_vlm": ParameterValue(
                                LaunchConfiguration("launch_vlm"), value_type=bool
                            )
                        }
                    ],
                )
                for agent in AGENTS
            ),
            # The robot's eyes. Not an agent: no state, no prompt, no decision --
            # it turns the latest camera frame into a sentence when triage asks.
            Node(
                package="amiga_vlm_bridge",
                executable="vlm_server",
                name="vlm_server",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_vlm")),
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("vlm_image_topic"),
                        "vlm_url": LaunchConfiguration("vlm_url"),
                        "system_prompt": "You are looking through the front "
                        "camera of a robot working in a pistachio orchard. "
                        "Describe only what is visible. Never guess at causes.",
                        # Inside triage's own 8 s deadline, so the far end gives
                        # up before the caller does and the log says the model
                        # was slow rather than that the service vanished.
                        "http_timeout_sec": 6.0,
                    }
                ],
            ),
        ]
    )
