from a2a.server.agent_execution import AgentExecutor, RequestContext
from a2a.server.events import EventQueue


class WorldStateHandler(AgentExecutor):
    def __init__(self, ros_node):
        self.node = ros_node

    async def execute(self, context: RequestContext, event_queue: EventQueue) -> None:
        state = self.node.get_state()
        await event_queue.enqueue_event(
            context.get_completed_task(
                artifacts=[{"parts": [{"type": "data", "data": state}]}]
            )
        )

    async def cancel(self, context: RequestContext, event_queue: EventQueue) -> None:
        pass
