#pragma once

#include <atomic>

#include "amiga_navigation_interfaces/action/move_in_frame.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amiga_navigation {

class NavigateToPoseInFrame : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using MoveInFrameAction =
      amiga_navigation_interfaces::action::MoveInFrame;
  using GoalHandleNavigateToPoseInFrame =
      rclcpp_action::ServerGoalHandle<MoveInFrameAction>;

  explicit NavigateToPoseInFrame(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~NavigateToPoseInFrame() = default;

 private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveInFrameAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle);
  void handle_accepted(
      const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle);
  void execute(
      const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle);

  void feedback_callback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(
      const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult&
          wrapped_result);

  void global_frame_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_frame_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Server<MoveInFrameAction>::SharedPtr action_server_;

  std::shared_ptr<GoalHandleNavigateToPoseInFrame> active_goal_handle_;
  GoalHandleNavigateToPose::SharedPtr nav2_goal_handle_;
  std::atomic<bool> cancel_requested_{false};

  std::shared_ptr<MoveInFrameAction::Feedback> fb_ =
      std::make_shared<MoveInFrameAction::Feedback>();

  geometry_msgs::msg::Pose global_frame_pose_;

  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
  double target_yaw_{0.0};
};

}  // namespace amiga_navigation
