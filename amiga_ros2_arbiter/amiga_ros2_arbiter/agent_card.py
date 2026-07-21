from a2a.types import AgentCard, AgentCapabilities, AgentSkill

AGENT_CARD = AgentCard(
    name="Amiga Arbiter",
    description=(
        "Gatekeeper for mission plan edits. Receives candidate XML plans from "
        "the mission planner, validates them (XSD schema, semantic checks, "
        "edit-size and rate limits), and publishes only accepted plans to the robot."
    ),
    url="http://localhost:10003",
    version="1.0.0",
    capabilities=AgentCapabilities(streaming=False),
    defaultInputModes=["text"],
    defaultOutputModes=["data"],
    skills=[
        AgentSkill(
            id="get_arbiter_status",
            name="Get Arbiter Status",
            description=(
                "Returns counts of accepted/rejected candidates and the reason "
                "for the most recent rejection."
            ),
            inputModes=["text"],
            outputModes=["data"],
            tags=["mission", "validation", "status"],
        )
    ],
)