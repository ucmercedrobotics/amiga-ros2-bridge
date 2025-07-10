import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
import os

from amiga_ros2_teleop.controller_utils import load_controller_config, ControllerMap, ButtonTrigger

class JoyExample(Node):
    def __init__(self, controller_config: dict):
        super().__init__("joy_example_node")

        self.controller_config = controller_config

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # Subscribe to Controller topics /joy
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_callback, qos_profile
        )
        self.button = ButtonTrigger(key="b", max_hold_frames=10)

    def joy_callback(self, msg: Joy):
        self.get_logger().debug(f"Got command from /joy: {msg}")
        cmap = ControllerMap(msg, self.controller_config)
        self.get_logger().debug(f"Got `a` button: {cmap.a}")
        
        if self.button.query(cmap):
            self.get_logger().debug(f"Pressed `b` button: {cmap.b}")
        
        if cmap.get("a", 0.0):
            self.get_logger().debug(f"Got `a` button: {cmap.a}")


def main(args=None):
    rclpy.init(args=args)
    
    # -- Load example controller config
    config_path = os.path.join(
        get_package_share_directory("amiga_ros2_teleop"),
        "config",
        "ps4.yaml"
    )
    controller_config = load_controller_config(config_path)
    
    node = JoyExample(controller_config)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
