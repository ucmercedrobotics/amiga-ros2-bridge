#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amiga_navigation_interfaces/action/move_in_frame.hpp"
#include "amiga_navigation_interfaces/action/rotate_in_frame.hpp"

namespace amiga_navigation {

class LinearVelo : public rclcpp::Node {
 public:
  using MoveInFrameAction = amiga_navigation_interfaces::action::MoveInFrame;
  using RotateInFrameAction = amiga_navigation_interfaces::action::RotateInFrame;
  using GoalHandleMoveInFrame =
      rclcpp_action::ServerGoalHandle<MoveInFrameAction>;
  using GoalHandleRotateInFrame =
      rclcpp_action::ServerGoalHandle<RotateInFrameAction>;

  explicit LinearVelo(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LinearVelo() = default;

 private:
  rclcpp_action::GoalResponse handle_move_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveInFrameAction::Goal> goal);
  rclcpp_action::CancelResponse handle_move_cancel(
      const std::shared_ptr<GoalHandleMoveInFrame> goal_handle);
  void handle_move_accepted(
      const std::shared_ptr<GoalHandleMoveInFrame> goal_handle);
  void execute_move(
      const std::shared_ptr<GoalHandleMoveInFrame> goal_handle);

  rclcpp_action::GoalResponse handle_rotate_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const RotateInFrameAction::Goal> goal);
  rclcpp_action::CancelResponse handle_rotate_cancel(
      const std::shared_ptr<GoalHandleRotateInFrame> goal_handle);
  void handle_rotate_accepted(
      const std::shared_ptr<GoalHandleRotateInFrame> goal_handle);
  void execute_rotate(
      const std::shared_ptr<GoalHandleRotateInFrame> goal_handle);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void stop_robot();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<MoveInFrameAction>::SharedPtr move_action_server_;
  rclcpp_action::Server<RotateInFrameAction>::SharedPtr rotate_action_server_;

  double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
  double current_linear_speed_ = 0.0;
  double current_angular_speed_ = 0.0;
  double min_linear_velocity_ = 0.1;
  double max_linear_velocity_ = 0.5;
  double max_angular_velocity_ = 0.5;
  double heading_tol_ = 0.2;
  double yaw_tol_ = 0.2;
  double yaw_slowdown_ = 1.0;
  double yaw_offset_ = 0.0;
  double forward_speed_cmd_ = 0.0;
  double angular_speed_cmd_ = 0.0;
};

}  // namespace amiga_navigation
