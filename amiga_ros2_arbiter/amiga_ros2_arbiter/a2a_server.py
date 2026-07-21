"""A2A handler that exposes arbiter status for external queries (port 10003)."""

import uuid

from a2a.server.agent_execution import AgentExecutor, RequestContext
from a2a.server.events import EventQueue
from a2a.types import Message, Part


class ArbiterHandler(AgentExecutor):
    """Returns a snapshot of the arbiter's current state when queried."""

    def __init__(self, ros_node):
        self.node = ros_node

    async def execute(self, context: RequestContext, event_queue: EventQueue) -> None:
        status = self.node.get_status()
        await event_queue.enqueue_event(
            Message(
                message_id=str(uuid.uuid4()),
                role="agent",
                parts=[Part(data=status)],
            )
        )

    async def cancel(self, context: RequestContext, event_queue: EventQueue) -> None:
        pass