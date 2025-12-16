#include "amiga_ros2_behavior_tree/actions/move_to_tree_id.hpp"

namespace amiga_bt {

MoveToTreeID::MoveToTreeID(const std::string &name,
                                     const BT::NodeConfig &config,
                                     const BT::RosNodeParams &params)
    : BT::RosActionNode<TreeIDWaypoint>(name, config, params) {}

BT::PortsList MoveToTreeID::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("id"),
                             BT::OutputPort<double>("object_angle")});
}

bool MoveToTreeID::setGoal(Goal &goal) {
  uint32_t id;

  if (!getInput("id", id)) {
    RCLCPP_ERROR(logger(), "Missing tree ID input");
    return false;
  }

  goal.tree_id = id;

  RCLCPP_INFO(logger(), "Moving to tree ID: %u", id);
  return true;
}

BT::NodeStatus MoveToTreeID::onResultReceived(
    const WrappedResult &result) {
  RCLCPP_INFO(logger(), "Navigation finished with code: %d", int(result.code));
  // Check if the action succeeded
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger(), "Navigation succeeded!");
    setOutput("object_angle", result.result->object_angle);
    return BT::NodeStatus::SUCCESS;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger(), "Navigation was canceled");
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_ERROR(logger(), "Navigation failed or was aborted");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveToTreeID::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Distance from goal: (%.6f)", feedback->dist);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
