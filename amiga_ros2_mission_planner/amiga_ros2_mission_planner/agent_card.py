from a2a.types import AgentCard, AgentCapabilities, AgentSkill

AGENT_CARD = AgentCard(
    name="Amiga Mission Planner",
    description=(
        "LLM-powered mission planner. On BT failure, fetches robot context "
        "and edits the active XML mission plan by 1-3 lines to recover."
    ),
    url="http://localhost:10001",
    version="1.0.0",
    capabilities=AgentCapabilities(streaming=False),
    defaultInputModes=["text"],
    defaultOutputModes=["data"],
    skills=[
        AgentSkill(
            id="get_planner_status",
            name="Get Planner Status",
            description="Returns the last failure event handled and the most recent edit summary.",
            inputModes=["text"],
            outputModes=["data"],
            tags=["mission", "planning", "status"],
        )
    ],
)