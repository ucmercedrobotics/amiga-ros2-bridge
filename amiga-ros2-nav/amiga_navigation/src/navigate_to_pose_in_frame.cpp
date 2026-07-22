#include "amiga_navigation/navigate_to_pose_in_frame.hpp"

#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::placeholders;

namespace amiga_navigation {

NavigateToPoseInFrame::NavigateToPoseInFrame(const rclcpp::NodeOptions& options)
    : Node("navigate_to_pose_in_frame", options) {
  nav_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  this->declare_parameter<std::string>("global_frame",
                                       "/odometry/filtered/global");
  std::string global_frame = this->get_parameter("global_frame").as_string();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  global_frame_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      global_frame, qos,
      std::bind(&NavigateToPoseInFrame::global_frame_callback, this, _1));

  action_server_ = rclcpp_action::create_server<MoveInFrameAction>(
      this, "move_in_frame",
      std::bind(&NavigateToPoseInFrame::handle_goal, this, _1, _2),
      std::bind(&NavigateToPoseInFrame::handle_cancel, this, _1),
      std::bind(&NavigateToPoseInFrame::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "NavigateToPoseInFrame initialized");
}

void NavigateToPoseInFrame::global_frame_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  global_frame_pose_ = msg->pose.pose;
}

rclcpp_action::GoalResponse NavigateToPoseInFrame::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const MoveInFrameAction::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(this->get_logger(),
              "Received goal to move relative: x=%.2f, y=%.2f",
              goal->x, goal->y);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigateToPoseInFrame::handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  cancel_requested_.store(true);
  if (nav2_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Forwarding cancel to Nav2");
    nav_client_->async_cancel_goal(nav2_goal_handle_);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigateToPoseInFrame::handle_accepted(
    const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle) {
  std::thread{std::bind(&NavigateToPoseInFrame::execute, this, _1), goal_handle}
      .detach();
}

void NavigateToPoseInFrame::execute(
    const std::shared_ptr<GoalHandleNavigateToPoseInFrame> goal_handle) {
  active_goal_handle_ = goal_handle;
  RCLCPP_INFO(this->get_logger(), "Executing relative move goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveInFrameAction::Result>();

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
    result->success = false;
    result->message = "Nav2 action server not available";
    goal_handle->abort(result);
    return;
  }

  double current_yaw = tf2::getYaw(global_frame_pose_.orientation);

  RCLCPP_INFO(this->get_logger(), "Global frame: x=%.2f, y=%.2f, yaw=%.2f rad",
              global_frame_pose_.position.x, global_frame_pose_.position.y,
              current_yaw);

  geometry_msgs::msg::PoseStamped pose_in_map;
  pose_in_map.header.stamp = this->now();
  pose_in_map.header.frame_id = "map";
  double rotated_x =
      goal->x * std::cos(current_yaw) - goal->y * std::sin(current_yaw);
  double rotated_y =
      goal->x * std::sin(current_yaw) + goal->y * std::cos(current_yaw);
  pose_in_map.pose.position.x = global_frame_pose_.position.x + rotated_x;
  pose_in_map.pose.position.y = global_frame_pose_.position.y + rotated_y;
  pose_in_map.pose.position.z = 0.0;
  double relative_yaw = std::atan2(goal->y, goal->x);
  pose_in_map.pose.orientation.x = 0.0;
  pose_in_map.pose.orientation.y = 0.0;
  pose_in_map.pose.orientation.z = std::sin(relative_yaw / 2.0);
  pose_in_map.pose.orientation.w = std::cos(relative_yaw / 2.0);

  auto nav_goal = NavigateToPose::Goal();
  nav_goal.pose = pose_in_map;

  RCLCPP_INFO(this->get_logger(),
              "Sending goal to Nav2: frame=%s, x=%.2f, y=%.2f, yaw=%.2f rad ",
              nav_goal.pose.header.frame_id.c_str(),
              nav_goal.pose.pose.position.x, nav_goal.pose.pose.position.y,
              relative_yaw);

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.feedback_callback =
      std::bind(&NavigateToPoseInFrame::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavigateToPoseInFrame::result_callback, this, _1);

  auto goal_handle_future =
      nav_client_->async_send_goal(nav_goal, send_goal_options);

  if (goal_handle_future.wait_for(std::chrono::seconds(5)) !=
      std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Timed out waiting for Nav2 goal acceptance");
    result->success = false;
    result->message = "Timed out waiting for Nav2 goal acceptance";
    goal_handle->abort(result);
    active_goal_handle_.reset();
    return;
  }

  nav2_goal_handle_ = goal_handle_future.get();
  if (!nav2_goal_handle_) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2");
    result->success = false;
    result->message = "Nav2 goal rejected";
    goal_handle->abort(result);
    active_goal_handle_.reset();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2, navigating...");
}

void NavigateToPoseInFrame::feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
  auto current_time = this->now();
  if ((current_time - last_feedback_time_).seconds() < 1.0) {
    return;
  }
  last_feedback_time_ = current_time;

  fb_->distance_remaining = static_cast<double>(feedback->distance_remaining);
  
  RCLCPP_INFO(this->get_logger(),
              "Distance remaining: %.2f m",
              fb_->distance_remaining);

  active_goal_handle_->publish_feedback(fb_);
}

void NavigateToPoseInFrame::result_callback(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult&
        wrapped_result) {
  auto result = std::make_shared<MoveInFrameAction::Result>();

  if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
    result->success = true;
    result->message = "Navigation succeeded";
    active_goal_handle_->succeed(result);
  } else if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "Navigation was canceled");
    result->success = false;
    result->message = "Navigation canceled";
    active_goal_handle_->canceled(result);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Navigation failed with code: %d",
                 int(wrapped_result.code));
    result->success = false;
    result->message = "Navigation failed";
    active_goal_handle_->abort(result);
  }

  active_goal_handle_.reset();
  nav2_goal_handle_.reset();
  cancel_requested_.store(false);
}

}  // namespace amiga_navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amiga_navigation::NavigateToPoseInFrame>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
