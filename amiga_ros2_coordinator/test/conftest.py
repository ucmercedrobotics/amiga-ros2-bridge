#!/usr/bin/env python3
"""One rclpy context for the suite, and the package on the path.

Only ``test_coordinator_node.py`` needs ROS at all -- the engine tests import
``CoordinatorSession`` and drive it with a clock they own. The fixture is
session-scoped because initialising and shutting rclpy down repeatedly in one
process invites flakiness, the same reason the comms package does it this way.

Idempotent because a developer running ``pytest`` over several packages at once
gets one process and two suites both wanting a context; whoever gets there first
owns it.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(scope="session", autouse=True)
def ros():
    import rclpy

    ours = not rclpy.ok()
    if ours:
        rclpy.init()
    yield
    if ours and rclpy.ok():
        rclpy.shutdown()
