from dataclasses import dataclass, field, asdict
from threading import Lock, Thread

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

import uvicorn
from a2a.server.apps import A2AStarletteApplication
from a2a.server.request_handlers import DefaultRequestHandler
from a2a.server.tasks import InMemoryTaskStore

from .agent_card import AGENT_CARD
from .a2a_server import WorldStateHandler


@dataclass
class RobotContext:
    # holds the current state of the robot, including position, battery status, and mission status
    position: dict = field(default_factory=dict)
    battery_pct: float = 0.0
    battery_voltage: float = 0.0
    mission_status: str = "idle"
    last_updated: str = ""


class WorldStateNode(Node):
    def __init__(self):
        #calls the constructor of the Node class with the name "world_state"
        super().__init__("world_state")
        #lock so only on thread can read and write at the same time, since the ROS subscriptions will be running in a separate thread from the A2A server
        self._lock = Lock()
        #initialize the robot context
        self._state = RobotContext()

        #subscribe to the ROS topics 
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10
        )
        self.create_subscription(
            BatteryState, "/battery_state", self._on_battery, 10
        )
        self.create_subscription(
            String, "/mission_status", self._on_mission_status, 10
        )
        #print a log message to indicate that the node has started and is listening to the relevant topics
        self.get_logger().info(
            "WorldStateNode started — listening on /amcl_pose, /battery_state, /mission_status"
        )
    #returns curr state or robot as a dict
    def get_state(self) -> dict:
        with self._lock:
            return asdict(self._state)

    #get x, y , z from pose msg & update state   
    def _on_pose(self, msg):
        with self._lock:
            p = msg.pose.pose.position
            self._state.position = {"x": p.x, "y": p.y, "z": p.z}
            self._state.last_updated = str(msg.header.stamp.sec)
    
    #get battery percentage and voltage & update state
    def _on_battery(self, msg):
        with self._lock:
            self._state.battery_pct = msg.percentage
            self._state.battery_voltage = msg.voltage
    
    #get status string & update state        
    def _on_mission_status(self, msg):
        with self._lock:
            self._state.mission_status = msg.data


def main():
    #innitialize ROS2 and create the WorldStateNode
    rclpy.init()
    node = WorldStateNode()

    #start the ROS2 spin loop in a separate thread so that it doesn't block the A2A server
    Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    #create the WorldStateHandler and A2AStarletteApplication app, then run the server on port 10004
    handler = WorldStateHandler(node)
    task_store = InMemoryTaskStore()
    app = A2AStarletteApplication(
        agent_card=AGENT_CARD,
        http_handler=DefaultRequestHandler(
            agent_executor=handler,
            task_store=task_store,
        ),
    )
    #run the A2A server using uvicorn on port 10004
    uvicorn.run(app.build(), host="0.0.0.0", port=10004, log_level="info")
