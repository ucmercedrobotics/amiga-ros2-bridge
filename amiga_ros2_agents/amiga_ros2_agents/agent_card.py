"""Agent cards for every agent served out of this package.

Ports are allocated in the 20000 range to stay clear of the standalone agents
(mission planner 10001, arbiter 10003, world state 10004).
"""

from a2a.types import AgentCapabilities, AgentCard, AgentSkill

LTL_AGENT_CARD = AgentCard(
    name="Amiga LTL Generator",
    description=(
        "Translates a natural-language mission for the Amiga robot into a "
        "Promela/SPIN-compatible LTL formula."
    ),
    url="http://localhost:20001",
    version="1.0.0",
    capabilities=AgentCapabilities(streaming=False),
    defaultInputModes=["text"],
    defaultOutputModes=["text", "data"],
    skills=[
        AgentSkill(
            id="generate_ltl",
            name="Generate LTL",
            description=(
                "Given a mission in plain English (e.g. 'visit trees 1 through 3 "
                "and sample the leaves at each'), returns an LTL formula in "
                "Promela/SPIN syntax."
            ),
            inputModes=["text"],
            outputModes=["text", "data"],
            tags=["ltl", "promela", "mission", "translation"],
        )
    ],
)

DUMMY_AGENT_CARD = AgentCard(
    name="Amiga Nothing",
    description="Placeholder agent — proves multiple agents can be served from this package.",
    url="http://localhost:20002",
    version="1.0.0",
    capabilities=AgentCapabilities(streaming=False),
    defaultInputModes=["text"],
    defaultOutputModes=["data"],
    skills=[
        AgentSkill(
            id="get_status",
            name="Get Status",
            description="Returns a snapshot of the agent's state.",
            inputModes=["text"],
            outputModes=["data"],
            tags=["status"],
        )
    ],
)
