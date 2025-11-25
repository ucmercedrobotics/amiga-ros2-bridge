#include "amiga_ros2_behavior_tree/actions/sample_leaf.hpp"

#include <cmath>

namespace amiga_bt {

SampleLeaf::SampleLeaf(const std::string &name, const BT::NodeConfig &config,
                       const BT::RosNodeParams &params)
    : BT::RosActionNode<NavigateViaLidar>(name, config, params) {}

BT::PortsList SampleLeaf::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("tree_angle")});
}

bool SampleLeaf::setGoal(Goal &goal) {
  double tree_angle;

  if (!getInput("tree_angle", tree_angle)) {
    RCLCPP_ERROR(logger(), "Missing tree angle input");
    return false;
  }

  goal.object_angle = tree_angle;

  RCLCPP_INFO(logger(),
              "Sampling leaves from tree at object angle: %.2f radians",
              goal.object_angle);
  return true;
}

BT::NodeStatus SampleLeaf::onResultReceived(const WrappedResult &result) {
  RCLCPP_INFO(logger(), "Sampling finished with code: %d", int(result.code));
  // Check if the action succeeded
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger(), "Sampling succeeded!");
    return BT::NodeStatus::SUCCESS;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger(), "Sampling was canceled");
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_ERROR(logger(), "Sampling failed or was aborted");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SampleLeaf::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Distance from goal: (%.6f)",
              feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
