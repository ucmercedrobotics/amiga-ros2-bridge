#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# define qos_profle should be same as subscribver
# do not chnage
qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)


class TwistJoy(Node):
    def __init__(self):
        super().__init__("twist_joy")

        # Publish to /amiga/cmd_vel
        self.vel_pub = self.create_publisher(
            TwistStamped, "/amiga/cmd_vel", qos_profile
        )
        
        # Subscribe to Controller topics /joy /cmd_vel
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, qos_profile)
        self.vel_sub = self.create_subscription(Twist, "/cmd_vel", self.vel_callback, qos_profile)
        
    def joy_callback(self, msg: Joy):
        self.get_logger().info(f"Got command from /joy: {msg}")

    def vel_callback(self, msg: Twist):
        self.get_logger().info(f"Got command from /cmd_vel: {msg}")
        self.publish_vel(msg)

    def publish_vel(self, msg: Twist):
        twist_msg = TwistStamped()
        twist_msg.twist = msg

        # same header used by farmng
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "robot"

        # TODO: Publish to amiga once we verify the timestamp works as intended
        self.vel_pub.publish(twist_msg)
        self.get_logger().info(
            f"publishing to /amiga/cmd_vel: linear.x={twist_msg.twist.linear.x}, angular.z={twist_msg.twist.angular.z}, timestamp={twist_msg.header.stamp}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()