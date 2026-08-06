"""
spin.py

The process entry point shared by every agent — `main()` is one call to run().

There are two ways an agent is asked to stop and they raise different things:
Ctrl-C surfaces as KeyboardInterrupt, while `ros2 launch` sending SIGTERM makes
rclpy shut the context down underneath us and spin() raise
ExternalShutdownException. In that second case the context is *already* down, so
an unconditional rclpy.shutdown() in the cleanup path raises RCLError on top of
it — hence the rclpy.ok() check. Getting this wrong means every agent prints two
tracebacks and exits non-zero on an ordinary launch shutdown.
"""

from typing import Callable

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


def run(node_factory: Callable[[], Node], *, multithreaded: bool = False) -> None:
    """Init rclpy, spin `node_factory()` until asked to stop, then clean up.

    Pass multithreaded=True for an agent whose callbacks can block — see the
    executor note in the package README.
    """
    rclpy.init()
    node = node_factory()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor() if multithreaded else None)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
