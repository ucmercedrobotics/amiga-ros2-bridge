#include "amiga_ros2_behavior_tree/actions/orient_robot_heading.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

namespace amiga_bt {

OrientRobotHeading::OrientRobotHeading(const std::string &name,
                                               const BT::NodeConfig &config,
                                               const BT::RosNodeParams &params)
    : BT::RosActionNode<RotateInFrame>(name, config, params) {}

BT::PortsList OrientRobotHeading::providedPorts() {
  return providedBasicPorts(
      {BT::InputPort<double>("yaw"), BT::InputPort<bool>("absolute")});
}

bool OrientRobotHeading::setGoal(Goal &goal) {
  double yaw = 0.0;
  bool absolute = false;

  if (getInput("yaw", yaw) && getInput("absolute", absolute)) {
    RCLCPP_DEBUG(logger(), "Received yaw/absolute input.");
  } else {
    RCLCPP_ERROR(logger(), "Missing yaw/absolute input");
    return false;
  }

  // Set the goal pose
  goal.yaw = yaw;
  goal.absolute = absolute;

  RCLCPP_INFO(logger(),
              "Orienting robot heading: yaw=%.2f rad, absolute=%s",
              yaw, absolute ? "true" : "false");

  return true;
}

BT::NodeStatus OrientRobotHeading::onResultReceived(
    const WrappedResult &result) {
  RCLCPP_INFO(logger(), "Navigation finished with code: %d", int(result.code));

  // Check if the action succeeded
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger(), "Navigation succeeded!");
    return BT::NodeStatus::SUCCESS;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger(), "Navigation was canceled");
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_ERROR(logger(), "Navigation failed or was aborted");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus OrientRobotHeading::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(),
              "Yaw remaining: %.2f rad (%.1f deg)",
              feedback->yaw_remaining,
              feedback->yaw_remaining * 180.0 / M_PI);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
