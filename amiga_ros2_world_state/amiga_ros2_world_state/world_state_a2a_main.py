"""A2A server entry point for World State.

This is the only module in the package that imports a2a-sdk / uvicorn.
world_state_node.py stays pure rclpy so it can be tested or run without
the a2a-sdk dependency.
"""

from threading import Thread

import rclpy
import uvicorn

from a2a.server.apps import A2AStarletteApplication
from a2a.server.request_handlers import DefaultRequestHandler
from a2a.server.tasks import InMemoryTaskStore

from .agent_card import AGENT_CARD
from .a2a_server import WorldStateHandler
from .world_state_node import WorldStateNode


def main():
    rclpy.init()
    node = WorldStateNode()

    Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    handler = WorldStateHandler(node)
    task_store = InMemoryTaskStore()
    app = A2AStarletteApplication(
        agent_card=AGENT_CARD,
        http_handler=DefaultRequestHandler(
            agent_executor=handler,
            task_store=task_store,
        ),
    )
    uvicorn.run(app.build(), host="0.0.0.0", port=10004, log_level="info")