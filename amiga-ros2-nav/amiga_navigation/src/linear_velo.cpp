#include "amiga_navigation/linear_velo.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>  // for std::clamp
#include <cmath>
#include <thread>

#include "amiga_navigation_interfaces/action/move_in_frame.hpp"
#include "amiga_navigation_interfaces/action/rotate_in_frame.hpp"

namespace amiga_navigation {

LinearVelo::LinearVelo(const rclcpp::NodeOptions &options)
    : Node("navigate_to_pose_in_frame", options) {
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10);

  std::string odom_topic =
    this->declare_parameter<std::string>("odom_topic", "/odometry/filtered/local");
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 1,
    std::bind(&LinearVelo::odom_callback, this, std::placeholders::_1));

  // Parameters
  this->declare_parameter<double>("min_linear_velocity", min_linear_velocity_);
  this->declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
  this->declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
  this->declare_parameter<double>("heading_tol", heading_tol_);
  this->declare_parameter<double>("yaw_tol", yaw_tol_);
  this->declare_parameter<double>("yaw_slowdown", yaw_slowdown_);
  this->declare_parameter<double>("yaw_offset", yaw_offset_);
  this->declare_parameter<double>("forward_speed", max_linear_velocity_);
  this->declare_parameter<double>("angular_speed", max_angular_velocity_);

  this->get_parameter("min_linear_velocity", min_linear_velocity_);
  this->get_parameter("max_linear_velocity", max_linear_velocity_);
  this->get_parameter("max_angular_velocity", max_angular_velocity_);
  this->get_parameter("heading_tol", heading_tol_);
  this->get_parameter("yaw_tol", yaw_tol_);
  this->get_parameter("yaw_slowdown", yaw_slowdown_);
  this->get_parameter("yaw_offset", yaw_offset_);
  this->get_parameter("forward_speed", forward_speed_cmd_);
  this->get_parameter("angular_speed", angular_speed_cmd_);

    move_action_server_ = rclcpp_action::create_server<MoveInFrameAction>(
      this, "move_in_frame",
      std::bind(&LinearVelo::handle_move_goal, this, std::placeholders::_1,
          std::placeholders::_2),
      std::bind(&LinearVelo::handle_move_cancel, this, std::placeholders::_1),
      std::bind(&LinearVelo::handle_move_accepted, this, std::placeholders::_1));

    rotate_action_server_ = rclcpp_action::create_server<RotateInFrameAction>(
      this, "rotate_in_frame",
      std::bind(&LinearVelo::handle_rotate_goal, this, std::placeholders::_1,
          std::placeholders::_2),
      std::bind(&LinearVelo::handle_rotate_cancel, this, std::placeholders::_1),
      std::bind(&LinearVelo::handle_rotate_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Linear Velocity Control Action Servers ready.");
}

void LinearVelo::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
}

rclcpp_action::GoalResponse LinearVelo::handle_move_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveInFrameAction::Goal> goal) {
  RCLCPP_INFO(get_logger(), "Received move goal: x=%.2f, y=%.2f", goal->x,
              goal->y);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LinearVelo::handle_move_cancel(
    const std::shared_ptr<GoalHandleMoveInFrame> goal_handle) {
  RCLCPP_WARN(get_logger(), "Move goal canceled.");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LinearVelo::handle_move_accepted(
    const std::shared_ptr<GoalHandleMoveInFrame> goal_handle) {
  std::thread{std::bind(&LinearVelo::execute_move, this, goal_handle)}.detach();
}

void LinearVelo::execute_move(
    const std::shared_ptr<GoalHandleMoveInFrame> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveInFrameAction::Feedback>();
  auto result = std::make_shared<MoveInFrameAction::Result>();

  rclcpp::Rate rate(10);
  RCLCPP_INFO(get_logger(), "Executing move goal: x=%.2f, y=%.2f", goal->x,
              goal->y);

  const double start_x = current_x_;
  const double start_y = current_y_;
  const double start_yaw = current_yaw_;

  bool position_done = false;
  bool heading_done = false;

  const double c = std::cos(start_yaw);
  const double s = std::sin(start_yaw);
  const double goal_vec_x = c * goal->x - s * goal->y;
  const double goal_vec_y = s * goal->x + c * goal->y;
  const double target_distance =
      std::sqrt(goal_vec_x * goal_vec_x + goal_vec_y * goal_vec_y);
  const double angle_to_goal = std::atan2(goal_vec_y, goal_vec_x);

  double traveled_distance = 0.0;
  const double forward_speed_cmd = forward_speed_cmd_;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stop_robot();
      result->success = false;
      result->message = "Move goal canceled.";
      goal_handle->canceled(result);
      return;
    }

    geometry_msgs::msg::Twist cmd;

    const double dx = current_x_ - start_x;
    const double dy = current_y_ - start_y;
    traveled_distance = std::sqrt(dx * dx + dy * dy);

    double distance_remaining =
        std::max(0.0, target_distance - traveled_distance);
    feedback->distance_remaining = distance_remaining;
    goal_handle->publish_feedback(feedback);

    // --- Rotate-in-place to face goal ---
    if (!position_done && !heading_done) {
      const double heading_error =
          atan2(sin(angle_to_goal - current_yaw_),
                cos(angle_to_goal - current_yaw_));
      if (std::fabs(heading_error) > heading_tol_) {
        const double scaled =
            std::clamp(heading_error / yaw_slowdown_, -max_angular_velocity_, max_angular_velocity_);
        cmd.angular.z = angular_speed_cmd_ * scaled;
      } else {
        heading_done = true;
      }
    }
    // --- Translation mode ---
    else if (!position_done) {
      if (traveled_distance < target_distance && target_distance > 0.0) {
        const double decel_distance = 2.0;
        double speed_scale;

        if (distance_remaining > decel_distance) {
          speed_scale = 1.0;
        } else {
          speed_scale = std::clamp(distance_remaining / decel_distance,
                                   min_linear_velocity_ / max_linear_velocity_,
                                   1.0);
        }

        cmd.linear.x = forward_speed_cmd * speed_scale;
      } else {
        position_done = true;
        cmd.linear.x = 0.0;
      }
    }

    cmd.linear.x =
      std::clamp(cmd.linear.x, -max_linear_velocity_, max_linear_velocity_);
    cmd.angular.z =
      std::clamp(cmd.angular.z, -max_angular_velocity_, max_angular_velocity_);

    cmd_pub_->publish(cmd);

    if (position_done || target_distance <= 1e-4) {
      stop_robot();
      result->success = true;
      result->message = "Move goal reached successfully.";
      goal_handle->succeed(result);
      return;
    }

    rate.sleep();
  }
}

rclcpp_action::GoalResponse LinearVelo::handle_rotate_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const RotateInFrameAction::Goal> goal) {
  RCLCPP_INFO(get_logger(), "Received rotate goal: yaw=%.2f, absolute=%d",
              goal->yaw, goal->absolute);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LinearVelo::handle_rotate_cancel(
    const std::shared_ptr<GoalHandleRotateInFrame> goal_handle) {
  RCLCPP_WARN(get_logger(), "Rotate goal canceled.");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LinearVelo::handle_rotate_accepted(
    const std::shared_ptr<GoalHandleRotateInFrame> goal_handle) {
  std::thread{std::bind(&LinearVelo::execute_rotate, this, goal_handle)}.detach();
}

void LinearVelo::execute_rotate(
    const std::shared_ptr<GoalHandleRotateInFrame> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<RotateInFrameAction::Feedback>();
  auto result = std::make_shared<RotateInFrameAction::Result>();

  rclcpp::Rate rate(10);
  RCLCPP_INFO(get_logger(), "Executing rotate goal: yaw=%.2f (absolute=%d)",
              goal->yaw, goal->absolute);

  const double start_yaw = current_yaw_;
  const double target_yaw = goal->absolute ? (goal->yaw + yaw_offset_)  : (start_yaw + goal->yaw);

  auto normalize_angle = [](double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  };

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stop_robot();
      result->success = false;
      result->message = "Rotate goal canceled.";
      goal_handle->canceled(result);
      return;
    }

    geometry_msgs::msg::Twist cmd;
    const double yaw_error = normalize_angle(target_yaw - current_yaw_);
    feedback->yaw_remaining = yaw_error;
    goal_handle->publish_feedback(feedback);

    if (std::fabs(yaw_error) > yaw_tol_) {
      const double scaled = std::clamp(yaw_error / yaw_slowdown_, -1.0, 1.0);
      cmd.angular.z = angular_speed_cmd_ * scaled;
    } else {
      stop_robot();
      result->success = true;
      result->message = "Rotate goal reached successfully.";
      goal_handle->succeed(result);
      return;
    }

    cmd.angular.z =
      std::clamp(cmd.angular.z, -max_angular_velocity_, max_angular_velocity_);
    cmd_pub_->publish(cmd);

    rate.sleep();
  }
}

void LinearVelo::stop_robot() {
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

}  // namespace amiga_navigation

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amiga_navigation::LinearVelo>());
  rclcpp::shutdown();
  return 0;
}
