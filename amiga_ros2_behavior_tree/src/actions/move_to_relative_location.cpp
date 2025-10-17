#include "amiga_ros2_behavior_tree/actions/move_to_relative_location.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amiga_bt {

MoveToRelativeLocation::MoveToRelativeLocation(const std::string &name,
                                               const BT::NodeConfig &config,
                                               const BT::RosNodeParams &params)
    : BT::RosActionNode<NavigateToPose>(name, config, params) {}

BT::PortsList MoveToRelativeLocation::providedPorts() {
  return providedBasicPorts(
      {BT::InputPort<double>("x"), BT::InputPort<double>("y"),
       BT::InputPort<bool>("absolute"), BT::InputPort<double>("yaw")});
}

bool MoveToRelativeLocation::setGoal(Goal &goal) {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  // specific to orientation
  // TODO: absolute handling is not supported
  bool absolute = false;
  std::string frame_id = "base_link";
  double z = 0.0;
  double w = 1.0;

  if (getInput("x", x) && getInput("y", y)) {
    RCLCPP_DEBUG(logger(), "Received x/y input. Making relative movement.");
  } else if (getInput("absolute", absolute) && getInput("yaw", yaw)) {
    RCLCPP_DEBUG(logger(), "Received yaw input. Making relative turn.");
    z = sin(yaw / 2.0);
    w = cos(yaw / 2.0);
  } else {
    RCLCPP_ERROR(logger(), "Missing x/y or yaw input");
    return false;
  }

  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger(), "Node is not available");
    return false;
  }

  goal.pose.header.frame_id = frame_id.c_str();
  goal.pose.header.stamp = node->now();

  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;

  goal.pose.pose.orientation.x = 0.0;
  goal.pose.pose.orientation.y = 0.0;
  goal.pose.pose.orientation.z = z;
  goal.pose.pose.orientation.w = w;

  RCLCPP_INFO(
      logger(),
      "Sending NavigateToPose goal: (%.2f, %.2f, %.2f) - orientation free", x,
      y, yaw);
  return true;
}

BT::NodeStatus MoveToRelativeLocation::onResultReceived(
    const WrappedResult &result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger(), "Navigation succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(logger(), "Navigation failed with code: %d", int(result.code));
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveToRelativeLocation::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  tf2::Quaternion q(feedback->current_pose.pose.orientation.x,
                    feedback->current_pose.pose.orientation.y,
                    feedback->current_pose.pose.orientation.z,
                    feedback->current_pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, current_yaw;
  m.getRPY(roll, pitch, current_yaw);

  RCLCPP_INFO(logger(),
              "Distance remaining: %.2f m, Current pose: (%.2f, %.2f), Yaw: "
              "%.2f rad (%.1f deg)",
              feedback->distance_remaining,
              feedback->current_pose.pose.position.x,
              feedback->current_pose.pose.position.y, current_yaw,
              current_yaw * 180.0 / M_PI);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
