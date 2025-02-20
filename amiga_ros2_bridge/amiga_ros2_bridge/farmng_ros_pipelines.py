# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations


# added libraries for ros2
import rclpy
from rclpy.node import Node

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import SubscribeRequest
from .farmng_ros_conversions import farmng_path_to_ros_type
from .farmng_ros_conversions import farmng_to_ros_msg

from rclpy.qos import QoSProfile

# public symbols
__all__ = [
    "create_ros_publisher",
]


async def create_ros_publisher(
    node: Node,
    client: EventClient,
    subscribe_request: SubscribeRequest,
    publish_topic: str = "",
):
    """Create a ROS publisher for a given gRPC EventClient subscribe request.

    Args:
        client (EventClient): The EventClient connected to the farm-ng amiga service.
        subscribe_request (SubscribeRequest): The subscription request for the farm-ng amiga service.
        publish_topic (str, optional): The ROS topic to publish the farm-ng amiga service data to.
            If not provided, the farm-ng subscription topic is used by default.
    """

    farm_ng_topic: str = f"/{client.config.name}{subscribe_request.uri.path}"
    topic: str = publish_topic if publish_topic else farm_ng_topic

    node.get_logger().info(
        f"Subscribing to farm-ng topic: {farm_ng_topic} and publishing on ROS topic: {topic}"
    )

    ros_msg_type = farmng_path_to_ros_type(subscribe_request.uri)
    # swapping the args in publisher
    # queue_size=10 is invalid in ros2, replace with QoSProfile(depth=10)
    ros_publisher = node.create_publisher(ros_msg_type, topic, QoSProfile(depth=10))

    async for event, message in client.subscribe(subscribe_request, decode=True):
        # node.get_logger().info(f"Got reply: {message}")
        ros_msgs = farmng_to_ros_msg(event, message)
        for msg in ros_msgs:
            ros_publisher.publish(msg)
