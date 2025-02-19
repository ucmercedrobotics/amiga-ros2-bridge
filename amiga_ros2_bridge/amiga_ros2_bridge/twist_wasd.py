#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

#define qos_profle should be same as subscribver
#do not chnage
qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)



class TwistToAmigaCmdVel(Node):
    def __init__(self):
        super().__init__('twist_to_amiga_cmd_vel')

        # Publish to /amiga/cmd_vel
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/amiga/cmd_vel', qos_profile)
        

        self.timer = self.create_timer(1.0, self.publish_velocity)

    def publish_velocity(self):
        twist_msg = TwistStamped()

        # same header used by farmng 
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "robot"

        twist_msg.twist.linear.x = 1.0  
        twist_msg.twist.angular.z = 0.0  

        self.velocity_pub.publish(twist_msg)
        self.get_logger().info(f"publishing to /amiga/cmd_vel: linear.x={twist_msg.twist.linear.x}, angular.z={twist_msg.twist.angular.z}, timestamp={twist_msg.header.stamp}")


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAmigaCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




#VERSION 2: GET TIMESTAMP FROM CANBUS
# class TwistToAmigaCmdVel(Node):
#     def __init__(self):
#         super().__init__('twist_to_amiga_cmd_vel')

#         self.sub_canbus_twist = self.create_subscription(
#             TwistStamped, '/canbus/twist', self.canbus_twist_callback, 10)
        
#       
#         #self.velocity_pub = self.create_publisher(TwistStamped, '/amiga/cmd_vel', 10)
#         self.velocity_pub = self.create_publisher(
#     TwistStamped, '/amiga/cmd_vel', qos_profile)

#     def canbus_twist_callback(self, msg):
#         twist_msg = TwistStamped()

#         # extract timestamp from /canbus/twist header
#         twist_msg.header.stamp = msg.header.stamp
#         twist_msg.header.frame_id = "robot"

#         twist_msg.twist.linear.x = 1.0 
#         twist_msg.twist.angular.z = 0.0  

#         # publoish to /amiga/cmd_vel
#         self.velocity_pub.publish(twist_msg)
#         self.get_logger().info(f"ppublish to /amiga/cmd_vel: linear.x={twist_msg.twist.linear.x}, angular.z={twist_msg.twist.angular.z}, timestamp={twist_msg.header.stamp}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = TwistToAmigaCmdVel()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
