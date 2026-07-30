"""
dummy_node.py

Placeholder agent (A2A, port 20002). Does nothing useful — it exists to prove
two agents can be served from this one package and to serve as the smallest
possible template for the next one.
"""

from threading import Lock
from typing import Dict

import rclpy
from rclpy.node import Node

from .a2a_server import StatusExecutor, serve_agent
from .agent_card import DUMMY_AGENT_CARD

A2A_PORT = 20002


class DummyNode(Node):
    """Counts how many times it has been asked for its status."""

    def __init__(self):
        super().__init__("dummy_agent")
        self._lock = Lock()
        self._queries = 0
        self.get_logger().info("DummyNode started — not doing much")

    def get_status(self) -> Dict:
        with self._lock:
            self._queries += 1
            return {"agent": "dummy", "state": "idle", "queries": self._queries}


def main():
    rclpy.init()
    node = DummyNode()
    serve_agent(node, StatusExecutor(node), DUMMY_AGENT_CARD, A2A_PORT)


if __name__ == "__main__":
    main()
