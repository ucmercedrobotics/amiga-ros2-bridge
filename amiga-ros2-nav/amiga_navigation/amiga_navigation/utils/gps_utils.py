# Taken from:
# https://github.com/ros-navigation/navigation2_tutorials/blob/rolling/nav2_gps_waypoint_follower_demo/nav2_gps_waypoint_follower_demo/utils/gps_utils.py

import math
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
import rclpy
import utm

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def euler_from_quaternion(q: Quaternion):
    """
    Convert a quaternion into euler angles
    taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
    Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose

# -- UTM
def to_utm(lat: float, lon: float):
    easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
    return easting, northing

# -- ENU Transform
def get_yaw_from_tf(transform: TransformStamped) -> float:
    """
    Extracts yaw (in radians) from a TransformStamped message.
    """
    q = transform.transform.rotation
    _, _, yaw = euler_from_quaternion(q)
    return yaw

def apply_yaw_offset(yaw: float, offset: float) -> float:
    """
    Applies an offset to a yaw angle and normalizes it to [-pi, pi]
    """
    adjusted = yaw + offset
    return (adjusted + math.pi) % (2 * math.pi) - math.pi
