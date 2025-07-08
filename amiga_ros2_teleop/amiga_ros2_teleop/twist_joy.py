import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
import os

from amiga_ros2_teleop.controller_utils import load_controller_config, ControllerMap

class TwistJoy(Node):
    def __init__(self, controller_config: dict):
        super().__init__("twist_joy")

        self.controller_config = controller_config

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # Publish to /amiga/cmd_vel
        self.vel_pub = self.create_publisher(
            TwistStamped, "/amiga/cmd_vel", qos_profile
        )

        # Subscribe to Controller topics /joy /cmd_vel
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_callback, qos_profile
        )
        self.vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.vel_callback, qos_profile
        )

    def joy_callback(self, msg: Joy):
        self.get_logger().debug(f"Got command from /joy: {msg}")
        cmap = ControllerMap(msg, self.controller_config)
        self.get_logger().debug(f"Got `a` button: {cmap.a}")

    def vel_callback(self, msg: Twist):
        self.get_logger().debug(f"Got command from /cmd_vel: {msg}")
        self.publish_vel(msg)

    def publish_vel(self, msg: Twist):
        twist_msg = TwistStamped()
        twist_msg.twist = msg

        # same header used by farmng
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "robot"

        # TODO: Publish to amiga once we verify the timestamp works as intended
        self.vel_pub.publish(twist_msg)
        self.get_logger().debug(
            f"publishing to /amiga/cmd_vel: linear.x={twist_msg.twist.linear.x}, angular.z={twist_msg.twist.angular.z}, timestamp={twist_msg.header.stamp}"
        )


def main(args=None):
    rclpy.init(args=args)
    
    # -- Load example controller config
    config_path = os.path.join(
        get_package_share_directory("amiga_ros2_teleop"),
        "config",
        "ps4.yaml"
    )
    controller_config = load_controller_config(config_path)
    
    node = TwistJoy(controller_config)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
