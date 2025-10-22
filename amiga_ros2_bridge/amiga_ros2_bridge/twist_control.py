#!/usr/bin/env python3
# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
#
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import asyncio
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList, SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from geometry_msgs.msg import Twist


############convert the msg to twist2d and push to queue#####
def cmd_vel_callback(node: Node, msg: Twist, queue: asyncio.Queue) -> None:
    """Callback for the /amiga/cmd_vel topic."""
    twist = Twist2d()
    twist.linear_velocity_x = msg.linear.x
    twist.linear_velocity_y = msg.linear.y
    twist.angular_velocity = msg.angular.z
    try:
        # put_nowait method selected by farmng
        queue.put_nowait(twist)  # Add the twist message to the queue
    except asyncio.QueueFull:
        node.get_logger().warn("Queue is full, dropping Twist message")


############convert the msg to twist2d and push to queue#####


############receive twist2d from queue and send to canbus##############
async def command_task(node: Node, client: EventClient, queue: asyncio.Queue) -> None:
    node.get_logger().info("command_task started")
    while rclpy.ok():
        # #get twist2d message
        # twist = await queue.get()
        # node.get_logger().info(f"Twist2d data received: {twist}")
        # #send twist, twist2d message, to the canbus
        # await client.request_reply("/twist", twist)

        # replaces above with try except for testing
        try:
            # try to get a message from the queue
            twist = await queue.get()
        except Exception as e:
            node.get_logger().error(f"DID NOT GET message from queue: {e}")
            continue  # skip to next
        try:
            await client.request_reply("/twist", twist)
        except Exception as e:
            node.get_logger().error(f"ERROR: {e}")


############receive twist2d from queue and send to canbus##############


async def run(node: Node, service_config: Path) -> None:
    # config with all the configs

    config_list = proto_from_json_file(service_config, EventServiceConfigList())

    # Populate the clients
    clients = {}
    subscriptions = []

    for config in config_list.configs:
        if config.port != 0 and config.name == "canbus":
            clients[config.name] = EventClient(config)
            node.get_logger().info(f"Client added for: {config.name}")
        else:
            subscriptions = config.subscriptions

    # Queue of Twist2d messages to send to the canbus service
    # do not change maxsize =1, set by farmng, max one message can be buffered
    queue = asyncio.Queue(maxsize=1)

    # Define QoS profile, should be same as publisher
    qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

    # Subscriber for /cmd_vel
    node.create_subscription(
        Twist,
        "/cmd_vel",
        lambda msg: cmd_vel_callback(node, msg, queue),
        qos_profile,
    )
    node.get_logger().info("successfully subscribed to /cmd_vel")

    tasks = []
    for subscription in subscriptions:
        service_name = subscription.uri.query.split("=")[-1]
        path = subscription.uri.path
        node.get_logger().info(f"subscription found: {service_name} at path {path}")

        if service_name == "canbus" and path == "/twist":
            tasks.append(
                asyncio.create_task(command_task(node, clients[service_name], queue))
            )

    node.get_logger().info(f"Starting to execute tasks: {len(tasks)} tasks to run.")
    await asyncio.gather(*tasks)


def main(args=None):
    rclpy.init(args=args)
    node = Node("twist_control_node")
    node.get_logger().info("amiga_twist_control started!")

    # HACK: Force the config we know is there
    service_config = Path(
        "/amiga-ros2-bridge/amiga_ros2_bridge/include/service_config.json"
    )
    node.get_logger().info(f"Service config set to {service_config}")

    # asyncio event loop
    loop = asyncio.get_event_loop()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # asyncio event loop
        asyncio.ensure_future(run(node, service_config))
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)  # Process ROS 2 callbacks
            loop.run_until_complete(asyncio.sleep(0.1))  # Process asyncio tasks
    except KeyboardInterrupt:
        node.get_logger().info("keyboard interrupt")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    main()
