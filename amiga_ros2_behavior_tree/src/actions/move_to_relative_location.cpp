#include "amiga_ros2_behavior_tree/actions/move_to_relative_location.hpp"

namespace amiga_bt {

MoveToRelativeLocation::MoveToRelativeLocation(const std::string &name,
                                               const BT::NodeConfig &config,
                                               const BT::RosNodeParams &params)
    : BT::RosActionNode<NavigateToPoseInFrame>(name, config, params) {}

BT::PortsList MoveToRelativeLocation::providedPorts() {
  return providedBasicPorts(
      {BT::InputPort<double>("x"), BT::InputPort<double>("y"),
       BT::InputPort<bool>("absolute"), BT::InputPort<double>("yaw")});
}

bool MoveToRelativeLocation::setGoal(Goal &goal) {
  double x, y, yaw;
  bool absolute;

  if (getInput("x", x) && getInput("y", y)) {
    RCLCPP_DEBUG(logger(), "Received x/y input. Making relative movement.");
  } else if (getInput("absolute", absolute) && getInput("yaw", yaw)) {
    RCLCPP_DEBUG(logger(), "Received yaw input. Making relative turn.");
  } else {
    RCLCPP_ERROR(logger(), "Missing x/y or yaw input");
    return false;
  }

  // Set the goal pose
  goal.x = x;
  goal.y = y;
  goal.yaw = yaw;
  goal.absolute = absolute;

  RCLCPP_INFO(logger(),
              "Moving to relative location: (x=%.2f, y=%.2f, yaw=%.2f rad, "
              "absolute=%s)",
              x, y, yaw, absolute ? "true" : "false");

  return true;
}

BT::NodeStatus MoveToRelativeLocation::onResultReceived(
    const WrappedResult &result) {
  RCLCPP_INFO(logger(), "Navigation finished successfully: %d",
              int(result.code));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveToRelativeLocation::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(),
              "Distance remaining: %.2f m, Yaw Remaining: "
              "%.2f rad (%.1f deg)",
              feedback->distance_remaining, feedback->yaw_remaining,
              feedback->yaw_remaining * 180.0 / M_PI);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
