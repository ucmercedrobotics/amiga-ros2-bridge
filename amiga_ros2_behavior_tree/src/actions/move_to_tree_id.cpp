#include "amiga_ros2_behavior_tree/actions/move_to_tree_id.hpp"

namespace amiga_bt {

MoveToTreeID::MoveToTreeID(const std::string &name,
                                     const BT::NodeConfig &config,
                                     const BT::RosNodeParams &params)
    : BT::RosActionNode<TreeIDWaypoint>(name, config, params) {
  // Transient-local: which tree the robot is at is a fact that persists until
  // it moves to another one, so a subscriber that starts late -- or restarts --
  // still learns it instead of waiting for the next move.
  if (auto node = node_.lock()) {
    target_pub_ = node->create_publisher<std_msgs::msg::UInt32>(
        "tree/target", rclcpp::QoS(1).transient_local());
  }
}

BT::PortsList MoveToTreeID::providedPorts() {
  return providedBasicPorts({BT::InputPort<uint32_t>("id"),
                             BT::InputPort<bool>("approach_tree")});
}

bool MoveToTreeID::setGoal(Goal &goal) {
  uint32_t id;
  bool approach_tree;

  if (!getInput("id", id) || !getInput("approach_tree", approach_tree)) {
    RCLCPP_ERROR(logger(), "Missing tree ID input or approach_tree flag");
    return false;
  }

  goal.tree_id = id;
  goal.approach_tree = approach_tree;
  if (target_pub_) {
    std_msgs::msg::UInt32 msg;
    msg.data = id;
    target_pub_->publish(msg);
  }
  RCLCPP_INFO(logger(), "Moving to tree ID: %u", id);
  return true;
}

BT::NodeStatus MoveToTreeID::onResultReceived(
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

BT::NodeStatus MoveToTreeID::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Distance from goal: (%.6f)", feedback->dist);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
