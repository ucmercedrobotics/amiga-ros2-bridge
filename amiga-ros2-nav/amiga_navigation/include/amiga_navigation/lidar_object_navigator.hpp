#pragma once

#include "amiga_navigation_interfaces/action/move_in_frame.hpp"
#include "amiga_navigation_interfaces/action/rotate_in_frame.hpp"
#include "amiga_navigation_interfaces/action/navigate_via_lidar.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amiga_navigation {

class LidarObjectNavigator : public rclcpp::Node {
 public:
  using MoveInFrameAction = amiga_navigation_interfaces::action::MoveInFrame;
  using GoalHandleMoveInFrame =
      rclcpp_action::ClientGoalHandle<MoveInFrameAction>;
  using RotateInFrameAction = amiga_navigation_interfaces::action::RotateInFrame;
  using GoalHandleRotateInFrame =
      rclcpp_action::ClientGoalHandle<RotateInFrameAction>;
  using NavigateViaLidar = amiga_navigation_interfaces::action::NavigateViaLidar;
  using GoalHandleNavigateViaLidar =
      rclcpp_action::ServerGoalHandle<NavigateViaLidar>;

  explicit LidarObjectNavigator(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LidarObjectNavigator() = default;

 private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const NavigateViaLidar::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle);
  void handle_accepted(
      const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle);
  void execute(const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle);

  // MoveInFrame action client callbacks
  void goal_response_callback(
      const GoalHandleMoveInFrame::SharedPtr& goal_handle);
  void feedback_callback(
      GoalHandleMoveInFrame::SharedPtr,
      const std::shared_ptr<const MoveInFrameAction::Feedback> feedback);
  void result_callback(
      const GoalHandleMoveInFrame::WrappedResult& result);
  void rotate_goal_response_callback(
      const GoalHandleRotateInFrame::SharedPtr& goal_handle);
  void rotate_feedback_callback(
      GoalHandleRotateInFrame::SharedPtr,
      const std::shared_ptr<const RotateInFrameAction::Feedback> feedback) {};
  void rotate_result_callback(
      const GoalHandleRotateInFrame::WrappedResult& result);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp_action::Client<MoveInFrameAction>::SharedPtr move_in_frame_client_;
    rclcpp_action::Client<RotateInFrameAction>::SharedPtr rotate_in_frame_client_;
    rclcpp_action::Server<NavigateViaLidar>::SharedPtr action_server_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_scan_;
    std::string base_frame_;
    std::string lidar_link_;
    float safety_distance_;
    float lidar_offset_distance_;
    float current_yaw_ = 0.0f;

    // Object selection tuning (ROS parameters).
    float azimuth_tolerance_;
    float min_object_height_;
    float max_object_height_;
    float min_object_distance_;
    float max_object_distance_;

    std::shared_ptr<GoalHandleNavigateViaLidar> active_goal_handle_;
};

}  // namespace amiga_navigation
