import uuid
import asyncio

from a2a.server.agent_execution import AgentExecutor, RequestContext
from a2a.server.events import EventQueue
from a2a.types import Message, Part

# publish rate in seconds — 1.0 = 1 Hz
TELEMETRY_RATE = 1.0

class WorldStateHandler(AgentExecutor):
    def __init__(self, ros_node):
        self.node = ros_node

    async def execute(self, context: RequestContext, event_queue: EventQueue) -> None:
        try:
            # loop continuously, pushing robot state at TELEMETRY_RATE Hz
            while True:
                state = self.node.get_state()
                await event_queue.enqueue_event(
                    Message(
                        message_id=str(uuid.uuid4()),
                        role="agent",
                        # data= passes the dict as structured JSON, not a string
                        parts=[Part(data=state)],
                    )
                )
                await asyncio.sleep(TELEMETRY_RATE)
        except asyncio.CancelledError:
            pass

    async def cancel(self, context: RequestContext, event_queue: EventQueue) -> None:
        pass