"""Launch the replanning loop: world state -> mission planner -> arbiter -> triage.

This is the research stack — `agents.launch.py` adds the note agent and the
VLM bridge on top of it.

world_state starts first so the planner's /world_state window has frames in it by
the time the first BT failure arrives.

triage starts last and sits at the end of the chain: it watches the loop above
it give up, escalates to the coordinator when it does, and answers the
coordinator's interpret_anomaly service. Launching it without a coordinator is
harmless — the escalations go into a topic nobody reads.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE = "amiga_ros2_agents"

AGENTS = ["world_state", "arbiter", "mission_planner", "triage"]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [Node(package=PACKAGE, executable=agent, output="screen") for agent in AGENTS]
    )
