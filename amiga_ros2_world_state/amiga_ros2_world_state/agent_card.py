from a2a.types import AgentCard, AgentCapabilities, AgentSkill

AGENT_CARD = AgentCard(
    name="Amiga World State",
    description="Provides current robot context: position, battery, and mission status.",
    url="http://localhost:10004",
    version="1.0.0",
    capabilities=AgentCapabilities(streaming=False),
    skills=[
        AgentSkill(
            id="get_robot_context",
            name="Get Robot Context",
            description="Returns current position, battery level, and mission status.",
            inputModes=["text"],
            outputModes=["data"],
        )
    ],
)
