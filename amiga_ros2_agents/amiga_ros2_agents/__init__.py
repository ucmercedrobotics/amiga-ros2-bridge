"""Every LLM agent on the Amiga, and the libraries they decide with.

One package, five directories, and the split between them is what each part is
allowed to know:

    runtime/        how an agent is wired -- model, prompts, status, spin
    mission/        the plan document: its schema and the tasks inside it
    verification/   the mission model compiler
    replanning/     the self-correction loop: planner, arbiter, world state
    coordination/   where this stack meets the fleet: triage, mission bridge

The dependency direction is downward through that list and never back up.
``runtime`` knows nothing about missions. ``mission`` and ``verification``
hold no rclpy at all -- they are libraries, unit-testable with no ROS and no
network. When something needs to reach sideways -- the arbiter needs the
ontology -- it imports the library, never the other agent, because agents
coordinate over topics and services.

``test/test_layering.py`` enforces all three of those rules. They are the kind
that decay silently, and a layout nobody checks is a layout that stops being
true.

Agents talk to each other over ROS topics and services. There is no HTTP layer,
no agent discovery and no A2A: everyone on this network is known ahead of time,
so a DDS participant is the whole story.
"""
