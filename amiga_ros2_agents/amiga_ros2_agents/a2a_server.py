"""
a2a_server.py

A2A + uvicorn plumbing shared by every agent in this package.

Each agent module supplies three things — a ROS node, an AgentExecutor, and an
AgentCard — and calls serve_agent(). Everything else (spin thread, task store,
Starlette app, uvicorn) lives here so agents stay one file each.
"""

from threading import Thread
from typing import Optional

import rclpy
import uvicorn
from a2a.server.agent_execution import AgentExecutor, RequestContext
from a2a.server.apps import A2AStarletteApplication
from a2a.server.events import EventQueue
from a2a.server.request_handlers import DefaultRequestHandler
from a2a.server.tasks import InMemoryTaskStore
from a2a.types import AgentCard, DataPart, Message, Part
from a2a.utils import new_agent_text_message


def agent_message(text: str = "", data: Optional[dict] = None) -> Message:
    """An agent-role Message carrying text and/or a structured JSON payload.

    Part is a discriminated RootModel, so DataPart has to be wrapped explicitly
    rather than passed as a bare `Part(data=...)` kwarg.
    """
    message = new_agent_text_message(text)
    if data is not None:
        message.parts.append(Part(root=DataPart(data=data)))
    return message


class StatusExecutor(AgentExecutor):
    """Ignores the request and returns a snapshot of the node's state.

    For status-only agents; agents that act on the request implement their own
    executor (see ltl_gen_node.LtlGenExecutor).
    """

    def __init__(self, ros_node):
        self.node = ros_node

    async def execute(self, context: RequestContext, event_queue: EventQueue) -> None:
        await event_queue.enqueue_event(agent_message(data=self.node.get_status()))

    async def cancel(self, context: RequestContext, event_queue: EventQueue) -> None:
        pass


def serve_agent(
    node,
    executor: AgentExecutor,
    agent_card: AgentCard,
    port: int,
) -> None:
    """Spin `node` in a background thread and serve `executor` over A2A.

    Blocks in uvicorn; the ROS spin runs on a daemon thread so the HTTP server
    owns the main thread.
    """
    Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = A2AStarletteApplication(
        agent_card=agent_card,
        http_handler=DefaultRequestHandler(
            agent_executor=executor,
            task_store=InMemoryTaskStore(),
        ),
    )
    uvicorn.run(app.build(), host="0.0.0.0", port=port, log_level="info")
