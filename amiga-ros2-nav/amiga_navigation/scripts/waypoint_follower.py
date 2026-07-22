#!/usr/bin/env python3
import time
import math
import utm
import json

import numpy as np

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

from amiga_navigation_interfaces.action import GPSWaypoint, TreeIDWaypoint, NavigateViaLidar, MoveToAisleHead
from amiga_interfaces.srv import GetTreeInfo

DISTANCE_TOLERANCE: float = 2.0  # meters


class WaypointFollowerActionServer(Node):

    def __init__(self):
        """
        After the inizialization of some usefull variables the node is created,
        the navigator is istanciated and the action server is defined, as well
        as the subscription to the /ublox_gps_node/fix topic (providing the
        robot gps position)

        """
        super().__init__("waypoint_follower")

        self.gps_position = []
        self.utm_position = []
        self.robot_yaw_world = 0.0
        # for north facing IMU VectorNav
        self.declare_parameter("yaw_offset", math.pi/2)
        self.yaw_offset = self.get_parameter("yaw_offset").value
        self.declare_parameter("orchard_tree_service", "/orchard/get_tree_info")
        service_name = (
            self.get_parameter("orchard_tree_service").get_parameter_value().string_value
        )

        self.navigator = BasicNavigator()

        self.navigator.waitUntilNav2Active(localizer="bt_navigator")

        self._gps_sub = self.create_subscription(
            msg_type=NavSatFix,
            topic="/gps/filtered",
            callback=self.pose_cb,
            qos_profile=10,
        )

        self._robot_pose_sub = self.create_subscription(
            msg_type=Odometry,
            topic="/odometry/filtered/local",
            callback=self.robot_pose_cb,
            qos_profile=1,
        )

        self._action_server_waypoint_follow = ActionServer(
            node=self,
            action_type=GPSWaypoint,
            action_name="follow_gps_waypoints",
            execute_callback=self.goto_callback,
        )

        self._action_server_approach_gps = ActionServer(
            node=self,
            action_type=GPSWaypoint,
            action_name="approach_gps_waypoints",
            execute_callback=self.approach_gps_callback,
        )

        self._tree_info_client = self.create_client(GetTreeInfo, service_name)

        self._action_server_treeid_follow = ActionServer(
            node=self,
            action_type=TreeIDWaypoint,
            action_name="follow_tree_id_waypoint",
            execute_callback=self.goto_treeid_callback,
        )

        self._action_server_aisle_head = ActionServer(
            node=self,
            action_type=MoveToAisleHead,
            action_name="move_to_aisle_head",
            execute_callback=self.move_to_aisle_head_callback,
        )

        self._navigate_via_lidar_client = ActionClient(
            node=self,
            action_type=NavigateViaLidar,
            action_name="/navigate_via_lidar"
        )

    def pose_cb(self, msg):
        """
        Method called each time a new robot gps position is received, it replace
        the old gps robot position (self.gps_position) with the update one.

        Args:
        msg (NavSatFix): message received under the /ublox_gps_node/fix topic
        (provided by the sender)
        """
        self.gps_position = (msg.latitude, msg.longitude)
        self.utm_position = utm.from_latlon(self.gps_position[0], self.gps_position[1])

    def robot_pose_cb(self, msg):
        """
        Method called each time a new robot pose is received, it extracts
        the robot's yaw (heading) in world frame from the pose quaternion.

        Args:
        msg (PoseWithCovarianceStamped): message received
        """
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(quaternion_list)
        self.robot_yaw_world = yaw - self.yaw_offset

    def goto_callback(self, goal_handle):
        """
        Method used when the action is called by another node, after initializing
        the feedbak message, the required new waypoint is converted in utm coordinates.
        the goal message to send to Nav2 is created setting the goal heading equal
        to the direction between the current robot position and the goal waypoint.
        finally the method send to Nav2 the new waypoint and keep listen to it in
        orther to give feedback on the action, until the task is completed and its
        result is sent to the action client.

        Args:
        goal_handle (GPSWaypoint): new waypoint to reach expressed in gps coordinates
        (provided by the action client)

        Returns:
        result (GPSWaypoint.result): current gps position of the robot at goal reached
        """
        feedback_msg = GPSWaypoint.Feedback()

        self.get_logger().info(
            "going to: %f, %f" % (goal_handle.request.lat, goal_handle.request.lon)
        )
        utm_coord = utm.from_latlon(
            goal_handle.request.lat, goal_handle.request.lon
        )

        while not self.utm_position:
            self.get_logger().info("waiting for current location...")
            rclpy.spin_once(self)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "utm"
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = utm_coord[0]
        goal_pose.pose.position.y = utm_coord[1]

        yaw = np.arctan2(
            utm_coord[1] - self.utm_position[1], utm_coord[0] - self.utm_position[0]
        ) - self.yaw_offset
        goal_pose.pose.orientation.w = np.cos(yaw / 2)
        goal_pose.pose.orientation.z = np.sin(yaw / 2)

        self.get_logger().info(
            f"Desired orientation: {goal_pose.pose.orientation.z, goal_pose.pose.orientation.w} and yaw: {yaw} rad"
        )

        self.navigator.followWaypoints([goal_pose])

        while not self.navigator.isTaskComplete():
            feedback_msg.dist = math.dist(
                [self.utm_position[0], self.utm_position[1]],
                [goal_pose.pose.position.x, goal_pose.pose.position.y],
            )
            self.get_logger().info("distance to goal %f" % feedback_msg.dist)
            goal_handle.publish_feedback(feedback_msg)
            rclpy.spin_once(self, timeout_sec=0.5)

        goal_handle.succeed()
        self.get_logger().info(f"Waypoint finished with: {self.navigator.getResult()}")

        result = GPSWaypoint.Result()
        result.lat = self.gps_position[0]
        result.lon = self.gps_position[1]
        return result

    def approach_gps_callback(self, goal_handle):
        """
        Similar to goto_callback but with a different feedback and result message, used when the robot needs to approach a gps waypoint without necessarily reaching it (e.g. for tree approach)

        Args:
        goal_handle (GPSWaypoint): new waypoint to approach expressed in gps coordinates
        (provided by the action client)

        Returns:
        result (GPSWaypoint.result): current gps position of the robot at completion and object angle to target
        """
        feedback_msg = GPSWaypoint.Feedback()

        self.get_logger().info(
            "approaching: %f, %f" % (goal_handle.request.lat, goal_handle.request.lon)
        )
        utm_coord = utm.from_latlon(
            goal_handle.request.lat, goal_handle.request.lon
        )

        while not self.utm_position:
            self.get_logger().info("waiting for current location...")
            rclpy.spin_once(self)

        object_angle = np.arctan2(
            utm_coord[1] - self.utm_position[1], utm_coord[0] - self.utm_position[0]
        )
        theta_robot = self.robot_yaw_world
        self.get_logger().info(f"Approaching GPS waypoint via LIDAR navigation at absolute angle {object_angle} facing {theta_robot}")

        object_angle = object_angle - theta_robot

        time.sleep(5.0)
        self.get_logger().info(f"Approaching GPS waypoint via LIDAR navigation at relative angle {object_angle}")
        while not self._navigate_via_lidar_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Waiting for NavigateViaLidar action server...")
        nav_goal = NavigateViaLidar.Goal()
        nav_goal.object_angle = object_angle
        future = self._navigate_via_lidar_client.send_goal_async(nav_goal)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        goal_handle_nav = future.result()
        if not goal_handle_nav.accepted:
            self.get_logger().error("NavigateViaLidar action goal rejected")
            goal_handle.abort()
            result = GPSWaypoint.Result()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result
        
        result_future = goal_handle_nav.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        nav_result = result_future.result()
        if nav_result is None or not nav_result.result.success:
            self.get_logger().error("NavigateViaLidar action failed")
            goal_handle.abort()
            result = GPSWaypoint.Result()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result
        self.get_logger().info("Successfully approached GPS waypoint via LIDAR navigation")

        goal_handle.succeed()
        
        result = GPSWaypoint.Result()
        result.lat = self.gps_position[0]
        result.lon = self.gps_position[1]
        return result

    def goto_treeid_callback(self, goal_handle):
        """
        Action callback for TreeIDWaypoint:
        - Queries orchard management via GetTreeInfo for the tree location and row waypoints
        - Chooses the closest row waypoint relative to current robot UTM position
        - Navigates to that row waypoint using Nav2 and streams feedback

        Args:
        goal_handle (TreeIDWaypoint): request containing `tree_id` and `approach_tree`.

        Returns:
        result (TreeIDWaypoint.Result): current gps position at completion and object angle to target.
        """
        feedback_msg = TreeIDWaypoint.Feedback()

        tree_id = int(goal_handle.request.tree_id)
        # because this actually takes you to the row waypoint near the tree, you can choose to approach the tree or not
        approach_tree = goal_handle.request.approach_tree
        self.get_logger().info(f"Received TreeID: {tree_id}")
        result = TreeIDWaypoint.Result()

        while not self.utm_position:
            self.get_logger().info("waiting for current location...")
            rclpy.spin_once(self)

        # first we gather information about the tree in question
        if not self._tree_info_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Orchard GetTreeInfo service unavailable")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        req = GetTreeInfo.Request()
        req.index_type = GetTreeInfo.Request.TREE_INDEX
        req.indicies = [tree_id]
        req.fields = ["lat", "lon", "row_waypoints"]

        future = self._tree_info_client.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        response = future.result()
        if response is None or not hasattr(response, "json"):
            self.get_logger().error("Invalid response from GetTreeInfo")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        try:
            data = json.loads(response.json)
        except Exception as e:
            self.get_logger().error(f"Failed parsing GetTreeInfo JSON: {e}")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        record = None
        if isinstance(data, list) and data:
            record = data[0]
        elif isinstance(data, dict):
            record = data

        if record is None:
            self.get_logger().error("GetTreeInfo returned empty result")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        row_wps = record.get("row_waypoints", [])
        tree_lat = record.get("lat")
        tree_lon = record.get("lon")

        candidate_latlon, candidate_utm = self.select_closest_point(row_wps)

        if candidate_latlon is None or candidate_utm is None:
            self.get_logger().error("No valid target waypoint available from orchard data")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        tree_utm = None
        if tree_lat is not None and tree_lon is not None:
            try:
                tree_utm = utm.from_latlon(float(tree_lat), float(tree_lon))
            except Exception as exc:
                self.get_logger().warn(f"Failed to convert tree lat/lon to UTM: {exc}")

        self.get_logger().info(
            f"Navigating to row waypoint near tree {tree_id}: {candidate_latlon[0]:.6f}, {candidate_latlon[1]:.6f}"
        )

        do_row_nav = True
        if approach_tree and tree_utm is not None:
            dist_to_tree = math.dist(
                [self.utm_position[0], self.utm_position[1]],
                [tree_utm[0], tree_utm[1]],
            )
            if dist_to_tree < DISTANCE_TOLERANCE:
                do_row_nav = False
                self.get_logger().info(
                    "Skipping row waypoint: already closer to tree than row waypoint or within distance tolerance"
                )

        if do_row_nav:
            self.get_logger().info(
                f"Navigating to row waypoint near tree {tree_id}: {candidate_latlon[0]:.6f}, {candidate_latlon[1]:.6f}"
            )

            self.navigate_to_utm(candidate_utm)

            while not self.navigator.isTaskComplete():
                feedback_msg.dist = math.dist(
                    [self.utm_position[0], self.utm_position[1]],
                    [candidate_utm[0], candidate_utm[1]],
                )
                self.get_logger().info("distance to goal %f" % feedback_msg.dist)
                goal_handle.publish_feedback(feedback_msg)
                rclpy.spin_once(self, timeout_sec=0.5)

        if approach_tree:
            object_angle = 0.0
            if tree_utm is not None:
                object_angle = np.arctan2(
                    tree_utm[1] - self.utm_position[1],
                    tree_utm[0] - self.utm_position[0],
                )
            else:
                self.get_logger().warn("Tree lat/lon unavailable; object_angle set to 0")

            theta_robot = self.robot_yaw_world
            self.get_logger().info(f"Approaching tree via LIDAR navigation at absolute angle {object_angle} facing {theta_robot}")

            object_angle = object_angle - theta_robot

            time.sleep(5.0)
            self.get_logger().info(f"Approaching tree via LIDAR navigation at relative angle {object_angle}")
            while not self._navigate_via_lidar_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info("Waiting for NavigateViaLidar action server...")
            nav_goal = NavigateViaLidar.Goal()
            nav_goal.object_angle = object_angle
            future = self._navigate_via_lidar_client.send_goal_async(nav_goal)
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
            goal_handle_nav = future.result()
            if not goal_handle_nav.accepted:
                self.get_logger().error("NavigateViaLidar action goal rejected")
                goal_handle.abort()
                result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
                result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
                return result
            
            result_future = goal_handle_nav.get_result_async()
            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
            nav_result = result_future.result()
            if nav_result is None or not nav_result.result.success:
                self.get_logger().error("NavigateViaLidar action failed")
                goal_handle.abort()
                result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
                result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
                return result
            self.get_logger().info("Successfully approached tree via LIDAR navigation")

        goal_handle.succeed()
        self.get_logger().info(f"TreeID waypoint finished with: {self.navigator.getResult()}")

        result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
        result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
        return result
        

    def move_to_aisle_head_callback(self, goal_handle):
        
        feedback_msg = MoveToAisleHead.Feedback()

        aisle_id = int(goal_handle.request.aisle_id)
        self.get_logger().info(f"Received AisleID: {aisle_id}")
        result = MoveToAisleHead.Result()

        while not self.utm_position:
            self.get_logger().info("waiting for current location...")
            rclpy.spin_once(self)

        # first we gather information about the tree in question
        if not self._tree_info_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Orchard GetTreeInfo service unavailable")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        req = GetTreeInfo.Request()
        req.index_type = GetTreeInfo.Request.AISLE_INDEX
        req.indicies = [aisle_id - 1, aisle_id]
        # no fields needed. everything will return

        future = self._tree_info_client.call_async(req)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        response = future.result()
        if response is None or not hasattr(response, "json"):
            self.get_logger().error("Invalid response from GetTreeInfo")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        try:
            data = json.loads(response.json)
        except Exception as e:
            self.get_logger().error(f"Failed parsing GetTreeInfo JSON: {e}")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        record = None
        if isinstance(data, list) and data:
            record = data[0]
        elif isinstance(data, dict):
            record = data

        if record is None:
            self.get_logger().error("GetTreeInfo returned empty result")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        aisle_heads = record.get("aisle_entrances", [])

        candidate_latlon, candidate_utm = self.select_closest_point(aisle_heads)

        if candidate_latlon is None or candidate_utm is None:
            self.get_logger().error("No valid target waypoint available from orchard data")
            goal_handle.abort()
            result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
            result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
            return result

        self.get_logger().info(
            f"Navigating to aisle head {aisle_id}: {candidate_latlon[0]:.6f}, {candidate_latlon[1]:.6f}"
        )

        self.navigate_to_utm(candidate_utm)

        while not self.navigator.isTaskComplete():
            feedback_msg.dist = math.dist(
                [self.utm_position[0], self.utm_position[1]],
                [candidate_utm[0], candidate_utm[1]],
            )
            self.get_logger().info("distance to goal %f" % feedback_msg.dist)
            goal_handle.publish_feedback(feedback_msg)
            rclpy.spin_once(self, timeout_sec=0.5)

        goal_handle.succeed()
        self.get_logger().info(f"Aisle head waypoint finished with: {self.navigator.getResult()}")

        result.lat = float(self.gps_position[0]) if self.gps_position else 0.0
        result.lon = float(self.gps_position[1]) if self.gps_position else 0.0
        return result


    def navigate_to_utm(self, utm_coord):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "utm"
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = utm_coord[0]
        goal_pose.pose.position.y = utm_coord[1]

        yaw = np.arctan2(
            utm_coord[1] - self.utm_position[1], utm_coord[0] - self.utm_position[0]
        ) - self.yaw_offset
        goal_pose.pose.orientation.w = np.cos(yaw / 2)
        goal_pose.pose.orientation.z = np.sin(yaw / 2)

        self.navigator.followWaypoints([goal_pose])


    def select_closest_point(self, points):
        closest_point = None
        min_dist = float("inf")
        for item in points:
            ll = self.latlon_from_item(item)
            if ll is None:
                continue
            utm_point = utm.from_latlon(ll[0], ll[1])
            d = math.dist([self.utm_position[0], self.utm_position[1]], [utm_point[0], utm_point[1]])
            if d < min_dist:
                min_dist = d
                closest_point = (ll, utm_point)
        return closest_point

    def latlon_from_item(self, item):
        if isinstance(item, dict):
            lat = item.get("lat")
            lon = item.get("lon")
            if lat is not None and lon is not None:
                return float(lat), float(lon)
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            return float(item[0]), float(item[1])
        return None

def main(args=None):
    rclpy.init(args=args)

    wp_folower_server = WaypointFollowerActionServer()

    rclpy.spin(wp_folower_server)


if __name__ == "__main__":
    main()
