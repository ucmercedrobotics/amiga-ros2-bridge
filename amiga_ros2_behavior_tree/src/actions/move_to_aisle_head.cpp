#include "amiga_ros2_behavior_tree/actions/move_to_aisle_head.hpp"

namespace amiga_bt {

MoveToAisleHead::MoveToAisleHead(const std::string &name,
                                     const BT::NodeConfig &config,
                                     const BT::RosNodeParams &params)
    : BT::RosActionNode<MoveToAisleHeadAction>(name, config, params) {}

BT::PortsList MoveToAisleHead::providedPorts() {
  return providedBasicPorts({BT::InputPort<uint32_t>("id")});
}

bool MoveToAisleHead::setGoal(Goal &goal) {
  uint32_t id;

  if (!getInput("id", id)) {
    RCLCPP_ERROR(logger(), "Missing aisle ID input");
    return false;
  }

  goal.aisle_id = id;
  RCLCPP_INFO(logger(), "Moving out of aisle: %u", id);
  return true;
}

BT::NodeStatus MoveToAisleHead::onResultReceived(
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

BT::NodeStatus MoveToAisleHead::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Distance from goal: (%.6f)", feedback->dist);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
