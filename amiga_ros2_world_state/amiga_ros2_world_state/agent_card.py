from a2a.types import AgentCard, AgentCapabilities, AgentSkill
#Agent Card for the World State Node
#Defines the agent's "job" and capabilities: aggregated robot context from
#navigation, tree-waypoint, and leaf-segmentation action feedback/status.
AGENT_CARD = AgentCard(
    name="Amiga World State",
    description=(
        "Provides current robot context: navigation pose/status, tree-waypoint "
        "distance/status, leaf-segmentation state/status, and overall mission status."
    ),
    url="http://localhost:10004",
    version="1.0.0",
    capabilities=AgentCapabilities(streaming=True),
    defaultInputModes=["text"],
    defaultOutputModes=["data"],
    skills=[
        AgentSkill(
            id="get_robot_context",
            name="Get Robot Context",
            description=(
                "Returns nav_pose, nav_distance_remaining, nav_status, "
                "tree_distance_remaining, tree_status, segment_state, "
                "segment_status, mission_status, and last_updated."
            ),
            inputModes=["text"],
            outputModes=["data"],
            tags=["robot", "world-state"],
        )
    ],
)