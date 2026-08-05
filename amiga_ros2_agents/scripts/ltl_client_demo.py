#!/usr/bin/env python3
"""Demo client for the LTL agent — calls the /mission/generate_ltl service.

Usage:
    python3 amiga_ros2_agents/tools/ltl_client_demo.py "visit trees 1 through 3 and sample the leaves at each"

Needs ROS sourced and `ros2 run amiga_ros2_agents ltl_gen` already running.
"""

import sys

import rclpy
from amiga_interfaces.srv import GenerateLTL
from rclpy.node import Node

DEFAULT_MISSION = "visit trees 1 through 3 and sample the leaves at each"
SERVICE = "/mission/generate_ltl"
# The model call can take a while on a local endpoint; don't give up early.
TIMEOUT_SEC = 180.0


def main():
    mission = " ".join(sys.argv[1:]) or DEFAULT_MISSION
    print(f"Mission: {mission}\nService: {SERVICE}\n")

    rclpy.init()
    node = Node("ltl_client_demo")
    client = node.create_client(GenerateLTL, SERVICE)

    if not client.wait_for_service(timeout_sec=10.0):
        print(f"FAILED: {SERVICE} not available — is ltl_gen running?", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    request = GenerateLTL.Request()
    request.mission = mission
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT_SEC)

    response = future.result()
    node.destroy_node()
    rclpy.shutdown()

    if response is None:
        print(f"FAILED: no response within {TIMEOUT_SEC:.0f}s", file=sys.stderr)
        sys.exit(1)

    print(f"model: {response.model}")
    print(f"LTL:   {response.formula}")
    if not response.ok:
        print(f"FAILED: {response.error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
