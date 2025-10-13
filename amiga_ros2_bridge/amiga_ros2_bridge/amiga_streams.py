# !/usr/bin/env python3
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


import asyncio
from pathlib import Path

# added libraries for ros2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList
from farm_ng.core.event_service_pb2 import SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from .farmng_ros_pipelines import create_ros_publisher


async def run(node, service_config: Path) -> None:
    # config with all the configs
    config_list: EventServiceConfigList = proto_from_json_file(
        service_config, EventServiceConfigList()
    )

    # populate the clients
    clients: dict[str, EventClient] = {}
    subscriptions: list[SubscribeRequest] = []

    for config in config_list.configs:
        if config.port != 0:
            clients[config.name] = EventClient(config)
        else:
            subscriptions = config.subscriptions

    # Create a ROS publisher for all the farm-ng service subscriptions
    tasks: list[asyncio.Task] = []

    for subscription in subscriptions:
        service_name = subscription.uri.query.split("=")[-1]
        service_tasks = asyncio.create_task(
            create_ros_publisher(node, clients[service_name], subscription)
        )
        tasks.append(service_tasks)

    await asyncio.gather(*tasks)


def main(args=None):
    # TODO: Get the arg as required from the roslaunch file
    # parser = argparse.ArgumentParser(description='Amiga ROS Bridge')
    # parser.add_argument('--service-config', type=Path, required=True, help='Path to config file')
    # args = parser.parse_args()

    # HACK: Force the config we know is there
    service_config = Path(
        "/amiga-ros2-bridge/amiga_ros2_bridge/include/service_config.json"
    )

    # start the ros node
    loop = asyncio.get_event_loop()
    rclpy.init(args=args)
    # initialize node, name of node
    node = Node("amiga_streams_node")
    # node = rclpy.create_node("amiga_streams_node")
    node.get_logger().info("amiga_streams_node started!")
    loop.run_until_complete(run(node, service_config))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
