#include "amiga_ros2_behavior_tree/actions/sample_leaf.hpp"

namespace amiga_bt {

SampleLeaf::SampleLeaf(const std::string &name, const BT::NodeConfig &config,
                       const BT::RosNodeParams &params)
    : BT::RosActionNode<SegmentLeaves>(name, config, params) {}

BT::PortsList SampleLeaf::providedPorts() {
  // SegmentLeaves.action has no goal fields, so no input ports needed
  return providedBasicPorts({});
}

bool SampleLeaf::setGoal(Goal &goal) {
  // SegmentLeaves.action has an empty goal, nothing to set
  // I might add number of leaves to sample as a goal in the future
  (void)goal;

  RCLCPP_INFO(logger(), "Starting leaf sampling/segmentation...");
  return true;
}

BT::NodeStatus SampleLeaf::onResultReceived(const WrappedResult &result) {
  RCLCPP_INFO(logger(), "Leaf sampling finished with code: %d", int(result.code));

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    if (result.result->success) {
      RCLCPP_INFO(logger(), "Leaf sampling succeeded: %s",
                  result.result->message.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(logger(), "Leaf sampling failed: %s",
                   result.result->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger(), "Leaf sampling was canceled");
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_ERROR(logger(), "Leaf sampling was aborted");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SampleLeaf::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Leaf sampling state: %s",
              feedback->current_state.c_str());
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
