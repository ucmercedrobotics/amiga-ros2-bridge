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



import numpy as np

#added libraries for ros2
import rclpy
from rclpy.node import Node 
from rclpy.executors import ExternalShutdownException

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core import uri_pb2
from farm_ng.core.event_pb2 import Event
from farm_ng.core.lie_pb2 import Isometry3F64
from farm_ng.core.lie_pb2 import Isometry3F64Tangent
from farm_ng.core.stamp import get_stamp_by_semantics_and_clock_type
from farm_ng.core.stamp import StampSemantics
from farm_ng.filter.filter_pb2 import FilterState
from farm_ng.gps.gps_pb2 import GpsFrame
from farm_ng.oak.oak_pb2 import OakFrame
from farm_ng.oak.oak_pb2 import OakImuPacket
from farm_ng.oak.oak_pb2 import OakImuPackets
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from google.protobuf.any_pb2 import Any
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

# public symbols
__all__ = [
    "farmng_path_to_ros_type",
    "farmng_stamp_to_ros_time",
    "farmng_to_ros_msg",
]


def farmng_stamp_to_ros_time(event: Event) -> rclpy.Time:
    """Convert a float timestamp to a ROS time.

    Args:
        event (float): The farm-ng event to extract the timestamp from.

    Returns:
        rospy.Time: The ROS time.
    """
    # Unpack the stamp data from the event
    # Use only the Amiga brain monotonic clock
    # Prefer driver receive, fallback to service send
    for semantics in [StampSemantics.DRIVER_RECEIVE, StampSemantics.SERVICE_SEND]:
        stamp: float | None = get_stamp_by_semantics_and_clock_type(
            event, semantics, "monotonic"
        )
        if stamp is not None:
            break

    if stamp is None:
        raise ValueError(
            f"Could not find appropriate timestamp for event with path: {event.uri.path}"
        )
    return rclpy.Time.from_sec(stamp)


def farmng_path_to_ros_type(uri: uri_pb2.Uri):
    """Map the farmng message type to the corresponding ROS message type.

    Args:
        uri (uri_pb2.Uri): The farmng URI representing the message type.

    Returns:
        type: The ROS message type corresponding to the farmng message type.
    """

    if "canbus" in uri.query and uri.path == "/twist":
        return TwistStamped
    elif "gps" in uri.query and uri.path == "/pvt":
        return NavSatFix
    elif "oak" in uri.query:
        if uri.path == "/imu":
            return Imu
        elif uri.path in ["/left", "/right", "/rgb", "/disparity"]:
            return CompressedImage
    elif "filter" in uri.query and uri.path == "/state":
        return Odometry

    raise NotImplementedError(f"Unknown farmng message type: {uri}")


def farmng_to_ros_msg(event: Event, farmng_msg: Any) -> list:
    """Convert a farm-ng event & message to a ROS message.

    Args:
        event (Event): The event data associated with the farm-ng message.
        farmng_msg (Any): The farm-ng message to be converted, wrapped in a google.protobuf.Any message.

    Returns:
        list: A list of converted ROS messages that correspond to the farm-ng event & message.
    """
    # parse Twist2d message
    if isinstance(farmng_msg, Twist2d):
        return [Twist2d_to_TwistStamped(farmng_msg, event)]
    # parse state estimation filter state
    elif isinstance(farmng_msg, FilterState):
        return [FilterState_to_Odometry(farmng_msg, event)]
    # parse GPS pvt message
    elif isinstance(farmng_msg, GpsFrame):
        return [GpsFrame_to_NavSatFix(farmng_msg, event)]
    # parse Oak IMU message
    elif isinstance(farmng_msg, OakImuPackets):
        return OakImuPackets_to_Imus(farmng_msg, event)
    # parse Oak Compressed Image message
    elif isinstance(farmng_msg, OakFrame):
        return [OakFrame_to_CompressedImage(farmng_msg, event)]

    raise NotImplementedError(
        f"Unknown farmng message: {type(farmng_msg)} at path: {event.uri.path}"
    )


def Twist2d_to_TwistStamped(twist2d: Twist2d, event: Event) -> TwistStamped:
    """Converts a farm-ng Twist2d message, and corresponding event, to a ROS TwistStamped message.

    Args:
        twist2d (Twist2d): The farm-ng Twist2d message to be converted.
        event (Event): The event data associated with the farm-ng message.
    Returns:
        TwistStamped: The ROS TwistStamped message.
    """
    if not isinstance(twist2d, Twist2d):
        raise TypeError(f"Expected canbus_pb2.Twist2d, received {type(twist2d)}")

    twist_stamped: TwistStamped = TwistStamped()
    # Unpack the stamp and frame_id data
    twist_stamped.header.stamp = farmng_stamp_to_ros_time(event)
    twist_stamped.header.frame_id = "robot"
    # Unpack the farmng twist data
    twist_stamped.twist = Twist2d_to_Twist(twist2d)
    return twist_stamped


def Twist2d_to_Twist(twist2d: Twist2d) -> Twist:
    """Converts a farm-ng Twist2d message to a ROS Twist message.

    Args:
        twist2d (Twist2d): The farm-ng Twist2d message to be converted.
    Returns:
        Twist: The ROS Twist message.
    """
    if not isinstance(twist2d, Twist2d):
        raise TypeError(f"Expected canbus_pb2.Twist2d, received {type(twist2d)}")
    twist: Twist = Twist()
    # Unpack the farmng twist data
    twist.linear.x = twist2d.linear_velocity_x
    twist.linear.y = twist2d.linear_velocity_y
    twist.angular.z = twist2d.angular_velocity
    return twist


def OakFrame_to_CompressedImage(oak_frame: OakFrame, event: Event) -> CompressedImage:
    """Converts a farm-ng OakFrame message, and corresponding event, to a ROS CompressedImage message.

    Args:
        oak_frame (OakFrame): The farm-ng OakFrame message to be converted.
        event (Event): The event data associated with the farm-ng message.
    Returns:
        CompressedImage: The ROS CompressedImage message.
    """
    if not isinstance(oak_frame, OakFrame):
        raise TypeError(f"Expected oak_pb2.OakFrame, received {type(oak_frame)}")

    # Name of the service (which is also the camera name)
    service_name: str = event.uri.query.split("=")[-1]

    compressed_img: CompressedImage = CompressedImage()
    # Unpack the stamp and frame_id data
    compressed_img.header.stamp = farmng_stamp_to_ros_time(event)
    compressed_img.header.frame_id = f"{service_name}{event.uri.path}"
    # Unpack the image data
    compressed_img.format = "jpeg"
    compressed_img.data = oak_frame.image_data
    return compressed_img


def OakImuPacket_to_Imu(oak_imu_packet: OakImuPacket, event: Event) -> Imu:
    """Converts a farm-ng OakImuPacket message, and corresponding event, to a ROS Imu message.

    Args:
        oak_imu_packet (OakImuPacket): The farm-ng OakImuPacket message to be converted.
        event (Event): The event data associated with the farm-ng message.
    Returns:
        Imu: The ROS Imu message.
    """
    if not isinstance(oak_imu_packet, OakImuPacket):
        raise TypeError(
            f"Expected oak_pb2.OakImuPacket, received {type(oak_imu_packet)}"
        )

    # Name of the service (which is also the camera name)
    service_name: str = event.uri.query.split("=")[-1]

    imu_ros: Imu = Imu()
    # Unpack the stamp and frame_id data
    imu_ros.header.stamp = rclpy.Time.from_sec(oak_imu_packet.gyro_packet.timestamp)
    imu_ros.header.frame_id = f"{service_name}{event.uri.path}"
    # Unpack the gyroscope data
    imu_ros.angular_velocity.x = oak_imu_packet.gyro_packet.gyro.x
    imu_ros.angular_velocity.y = oak_imu_packet.gyro_packet.gyro.y
    imu_ros.angular_velocity.z = oak_imu_packet.gyro_packet.gyro.z
    # Unpack the accelerometer data
    imu_ros.linear_acceleration.x = oak_imu_packet.accelero_packet.accelero.x
    imu_ros.linear_acceleration.y = oak_imu_packet.accelero_packet.accelero.y
    imu_ros.linear_acceleration.z = oak_imu_packet.accelero_packet.accelero.z
    return imu_ros


def OakImuPackets_to_Imus(oak_imu_packets: OakImuPackets, event: Event) -> list[Imu]:
    """Converts a farm-ng OakImuPackets message, and corresponding event, to list of ROS Imu messages.

    Args:
        oak_imu_packets (OakImuPackets): The farm-ng OakImuPackets message to be converted.
        event (Event): The event data associated with the farm-ng message.
    Returns:
        list[Imu]: The list of ROS Imu messages.
    """
    if not isinstance(oak_imu_packets, OakImuPackets):
        raise TypeError(
            f"Expected oak_pb2.OakImuPackets, received {type(oak_imu_packets)}"
        )

    ros_imu_msgs: list[Imu] = []
    # Iterate over the OakImuPacket messages
    for packet in oak_imu_packets.packets:
        ros_imu_msgs.append(OakImuPacket_to_Imu(packet, event))
    return ros_imu_msgs


def GpsFrame_to_NavSatFix(gps_frame: GpsFrame, event: Event) -> NavSatFix:
    """Converts a farm-ng GpsFrame message, and corresponding event, to a ROS NavSatFix message.

    Args:
        gps_frame (GpsFrame): The farm-ng GpsFrame message to be converted.
        event (Event): The event data associated with the farm-ng message.
    Returns:
        NavSatFix: The ROS NavSatFix message.
    """
    if not isinstance(gps_frame, GpsFrame):
        raise TypeError(f"Expected gps_pb2.GpsFrame, received {type(gps_frame)}")

    nav_sat_fix: NavSatFix = NavSatFix()
    # Unpack the stamp and frame_id data
    nav_sat_fix.header.stamp = farmng_stamp_to_ros_time(event)
    nav_sat_fix.header.frame_id = "gps_antenna"
    # Unpack the GPS data
    nav_sat_fix.latitude = gps_frame.latitude
    nav_sat_fix.longitude = gps_frame.longitude
    nav_sat_fix.altitude = gps_frame.altitude
    return nav_sat_fix


def FilterState_to_Odometry(filter_state: FilterState, event: Event) -> Odometry:
    """Converts a farm-ng FilterState message, and corresponding event, to a ROS Odometry message.

    Args:
        filter_state (FilterState): The farm-ng FilterState message to be converted.
        event (Event): The event data associated with the farm-ng message.
    Returns:
        Odometry: The ROS Odometry message.
    """
    odometry: Odometry = Odometry()
    # Unpack the stamp and frame_id data
    odometry.header.stamp = farmng_stamp_to_ros_time(event)
    odometry.header.frame_id = filter_state.pose.frame_b
    odometry.child_frame_id = filter_state.pose.frame_b
    # Unpack the Pose and Twist data
    odometry.pose.pose = Isometry3F64_to_Pose(filter_state.pose.a_from_b)
    odometry.twist.twist = Isometry3F64Tangent_to_Twist(
        filter_state.pose.tangent_of_b_in_a
    )

    # Unpack the covariance
    std_pos_lin_x = filter_state.uncertainty_diagonal.data[0]
    std_pos_lin_y = filter_state.uncertainty_diagonal.data[1]
    std_pos_ang_z = filter_state.uncertainty_diagonal.data[2]
    std_vel_lin_x = filter_state.uncertainty_diagonal.data[3]
    std_vel_ang_z = filter_state.uncertainty_diagonal.data[4]
    odometry.pose.covariance = uncertainties_to_covariance_matrix(
        [std_pos_lin_x, std_pos_lin_y, 0.0, 0.0, 0.0, std_pos_ang_z]
    )
    odometry.twist.covariance = uncertainties_to_covariance_matrix(
        [std_vel_lin_x, 0.0, 0.0, 0.0, 0.0, std_vel_ang_z]
    )
    return odometry


def uncertainties_to_covariance_matrix(uncertainties: list[float]) -> list[float]:
    """Packages a list of uncertainties (the std. dev of the covariance matrix diagonal) into a list of floats
    representing the covariance matrix in row-major order.

    Args:
        uncertainties (list[float]): The list of uncertainties (the std. dev values of the covariance matrix diagonal).
    Returns:
        list[float]: The list of floats representing the covariance matrix in row-major order.
    """
    return np.diag([x**2 for x in uncertainties]).flatten().tolist()


def Isometry3F64_to_Pose(isometry_3f64: Isometry3F64) -> Pose:
    """Converts a farm-ng Isometry3F64 message, to a ROS Pose message.

    Args:
        isometry_3f64 (Isometry3F64): The farm-ng Isometry3F64 proto message to be converted.
    Returns:
        Pose: The ROS Pose message.
    """
    if not isinstance(isometry_3f64, Isometry3F64):
        raise TypeError(
            f"Expected lie_pb2.Isometry3F64, received {type(isometry_3f64)}"
        )

    ros_pose: Pose = Pose()
    # Unpack the translation
    ros_pose.position.x = isometry_3f64.translation.x
    ros_pose.position.y = isometry_3f64.translation.y
    ros_pose.position.z = isometry_3f64.translation.z
    # Unpack the rotation (quaternion)
    ros_pose.orientation.x = isometry_3f64.rotation.unit_quaternion.imag.x
    ros_pose.orientation.y = isometry_3f64.rotation.unit_quaternion.imag.y
    ros_pose.orientation.z = isometry_3f64.rotation.unit_quaternion.imag.z
    ros_pose.orientation.w = isometry_3f64.rotation.unit_quaternion.real
    return ros_pose


def Isometry3F64Tangent_to_Twist(isometry_3f64_tangent: Isometry3F64Tangent) -> Twist:
    """Converts a farm-ng Isometry3F64Tangent message, to a ROS Twist message.

    Args:
        isometry_3f64_tangent (Isometry3F64Tangent): The farm-ng Isometry3F64Tangent message to be converted.
    Returns:
        Twist: The ROS Twist message.
    """
    if not isinstance(isometry_3f64_tangent, Isometry3F64Tangent):
        raise TypeError(
            f"Expected pose_pb2.Pose, received {type(isometry_3f64_tangent)}"
        )

    twist: Twist = Twist()
    # Unpack the linear velocities
    twist.linear.x = isometry_3f64_tangent.linear_velocity.x
    twist.linear.y = isometry_3f64_tangent.linear_velocity.y
    twist.linear.z = isometry_3f64_tangent.linear_velocity.z
    # Unpack the angular velocities
    twist.angular.x = isometry_3f64_tangent.angular_velocity.x
    twist.angular.y = isometry_3f64_tangent.angular_velocity.y
    twist.angular.z = isometry_3f64_tangent.angular_velocity.z
    return twist
