#!/usr/bin/env python3
"""One rclpy context for the suite.

Idempotent on purpose. Under ``colcon test`` each package runs in its own
process and nothing collides, but a developer running ``pytest`` over several
packages at once gets one process and two suites both wanting a context --
which fails on the second ``rclpy.init()`` and looks like a broken test rather
than a broken harness. Whoever gets there first owns the context and shuts it
down; the other waits.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


@pytest.fixture(scope="session", autouse=True)
def ros():
    import rclpy

    ours = not rclpy.ok()
    if ours:
        rclpy.init()
    yield
    if ours and rclpy.ok():
        rclpy.shutdown()
