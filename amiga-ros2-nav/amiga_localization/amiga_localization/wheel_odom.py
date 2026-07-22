import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Quaternion

import math

"""
Wheel Odometry Node:

The Amiga doesn't have wheel encoders so we will use 
Twist messages to approximate the wheel odometry.
"""


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry_node")

        # -- Configuration
        # Default values measured from Amiga
        self.declare_parameter("wheel_params.wheel_radius", 0.20)
        self.declare_parameter("wheel_params.track_width", 1.10)

        # TODO: Figure out consistent initial pose
        self.declare_parameter("initial_pose.x", 0.0)
        self.declare_parameter("initial_pose.y", 0.0)
        self.declare_parameter("initial_pose.theta", 0.0)

        # -- Parameters
        self.wheel_radius = self.get_parameter("wheel_params.wheel_radius").value
        self.track_width = self.get_parameter("wheel_params.track_width").value

        self.x = self.get_parameter("initial_pose.x").value
        self.y = self.get_parameter("initial_pose.y").value
        self.theta = self.get_parameter("initial_pose.theta").value
        self.v = 0.0
        self.omega = 0.0

        self.last_time = self.get_clock().now()
        self.current_time = self.last_time

        # -- Pub/Sub
        self.create_subscription(TwistStamped, "/canbus/twist", self.twist_callback, 10)
        self.wheel_odom_publisher = self.create_publisher(
            Odometry, "/wheel/odometry", 10
        )

    def update(self, msg: TwistStamped):
        # -- Get v and omega from msg
        v = msg.twist.linear.x
        omega = msg.twist.angular.z

        # -- Update time
        # TODO: Should we use time from the TwistStamped msg?
        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_time).nanoseconds / 1e9
        self.last_time = self.current_time

        # -- Compute simulated v and omega
        v_left = v - (omega * self.track_width / 2.0)
        v_right = v + (omega * self.track_width / 2.0)
        v_sim = (v_left + v_right) / 2
        omega_sim = (v_right - v_left) / self.track_width

        # -- Update pose
        self.theta += omega_sim * dt
        # normalize theta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        self.x += v_sim * math.cos(self.theta) * dt
        self.y += v_sim * math.sin(self.theta) * dt
        self.v = v_sim
        self.omega = omega_sim
        
    # Just pass raw v and omega values.
    def update_raw(self, msg: TwistStamped):
        v = msg.twist.linear.x
        omega = msg.twist.angular.z
        self.v = v
        self.omega = omega
        

    def create_odom_msg(self) -> Odometry:
        msg = Odometry()
        msg.header.stamp = self.current_time.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = get_quaternion_from_yaw(self.theta)

        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.omega

        return msg

    def twist_callback(self, msg: TwistStamped):
        # self.update(msg)
        self.update_raw(msg)
        odom_msg = self.create_odom_msg()
        # Stamp with the incoming twist's time (monotonic, sim-time consistent).
        # update_raw() never advances self.current_time, so relying on it would
        # freeze the stamp at node startup and the EKF would reject every
        # subsequent measurement.
        odom_msg.header.stamp = msg.header.stamp
        self.wheel_odom_publisher.publish(odom_msg)
        self.get_logger().debug(
            f"Wheel Odometry: x={self.x}, y={self.y}, theta={self.theta}, v={self.v}, omega={self.omega}"
        )


# -- Utilities
def get_quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()