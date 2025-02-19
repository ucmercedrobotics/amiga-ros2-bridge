#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
import time

#define qos_profle should be same as subscribver 
qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)




#TEST: drive 2 meters in 10 secs (works perfect!)
# class TwistToAmigaCmdVel(Node):
#     def __init__(self):
#         super().__init__('twist_to_amiga_cmd_vel')

#         # Publisher for /amiga/cmd_vel
#         self.velocity_pub = self.create_publisher(
#             TwistStamped, '/amiga/cmd_vel', 10)

#         # Timer to publish velocity commands
#         self.timer = self.create_timer(0.1, self.publish_velocity)  # Publish every 0.1 seconds

#         # Start time to track duration
#         self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
#         self.duration = 10  # Move for 10 seconds

#     def publish_velocity(self):
#         twist_msg = TwistStamped()

#         # Set header
#         twist_msg.header.stamp = self.get_clock().now().to_msg()
#         twist_msg.header.frame_id = "robot"

#         # Move forward at 0.2 m/s
#         twist_msg.twist.linear.x = 0.2  
#         twist_msg.twist.angular.z = 0.0  

#         # Publish velocity command
#         self.velocity_pub.publish(twist_msg)
#         self.get_logger().info(f"Moving forward: linear.x={twist_msg.twist.linear.x}")

#         # Check if the duration has elapsed
#         current_time = self.get_clock().now().seconds_nanoseconds()[0]
#         if current_time - self.start_time >= self.duration:
#             # Stop the robot
#             twist_msg.twist.linear.x = 0.0  
#             self.velocity_pub.publish(twist_msg)
#             self.get_logger().info("Stopping the robot.")
#             self.timer.cancel()  # Stop the timer

# def main(args=None):
#     rclpy.init(args=args)
#     node = TwistToAmigaCmdVel()
#     rclpy.spin(node)  # Keep the node running
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




#TEST: turn 90° (works perfect!)
class TwistToAmigaCmdVel(Node):
    def __init__(self):
        super().__init__('twist_to_amiga_cmd_vel')

        # Publisher for /amiga/cmd_vel
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/amiga/cmd_vel', 10)

        # Timer to publish velocity commands
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Publish every 0.1 seconds

        # Start time to track duration
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.duration = 3.14  # Turn for approximately 3.14 seconds (90 degrees at 0.5 rad/s)

    def publish_velocity(self):
        twist_msg = TwistStamped()

        # Set header
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "robot"

        # Rotate left at 0.5 rad/s
        twist_msg.twist.linear.x = 0.0  # No linear movement
        twist_msg.twist.angular.z = 0.5  # Angular velocity for left turn

        # Publish velocity command
        self.velocity_pub.publish(twist_msg)
        self.get_logger().info(f"Turning left: angular.z={twist_msg.twist.angular.z}")

        # Check if the duration has elapsed
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.start_time >= self.duration:
            # Stop the robot
            twist_msg.twist.angular.z = 0.0  
            self.velocity_pub.publish(twist_msg)
            self.get_logger().info("Stopping the robot.")
            self.timer.cancel()  # Stop the timer

def main(args=None):
    rclpy.init(args=args)
    node = TwistToAmigaCmdVel()
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






#VERSION1: DRIVE ROBOT AT CONST F
# class TwistToAmigaCmdVel(Node):
#     def __init__(self):
#         super().__init__('twist_to_amiga_cmd_vel')

#         # Publish to /amiga/cmd_vel
#         self.velocity_pub = self.create_publisher(
#             TwistStamped, '/amiga/cmd_vel', qos_profile)
        

#         self.timer = self.create_timer(1.0, self.publish_velocity)

#     def publish_velocity(self):
#         twist_msg = TwistStamped()

#         # same header used by farmng 
#         twist_msg.header.stamp = self.get_clock().now().to_msg()
#         twist_msg.header.frame_id = "robot"

#         twist_msg.twist.linear.x = 1.0  
#         twist_msg.twist.angular.z = 0.0  

#         self.velocity_pub.publish(twist_msg)
#         self.get_logger().info(f"publishing to /amiga/cmd_vel: linear.x={twist_msg.twist.linear.x}, angular.z={twist_msg.twist.angular.z}, timestamp={twist_msg.header.stamp}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = TwistToAmigaCmdVel()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




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