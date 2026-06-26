from a2a.types import AgentCard, AgentCapabilities, AgentSkill

#Agent Card for the World State Node
#Defines the agent's "job" and capabilities, including the skill to get the robot context (position, battery, mission status)
AGENT_CARD = AgentCard(
    name="Amiga World State",
    description="Provides current robot context: position, battery, and mission status.",
    url="http://localhost:10004",
    version="1.0.0",
    capabilities=AgentCapabilities(streaming=True),
    default_input_modes=["text"],
    default_output_modes=["data"],
    skills=[
        AgentSkill(
            id="get_robot_context",
            name="Get Robot Context",
            description="Returns current position, battery level, and mission status.",
            input_modes=["text"],
            output_modes=["data"],
            tags=["robot", "world-state"],
        )
    ],
)