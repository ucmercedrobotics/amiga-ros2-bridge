#include "amiga_ros2_behavior_tree/actions/harvest_fruit.hpp"

namespace amiga_bt {

HarvestFruit::HarvestFruit(const std::string &name, const BT::NodeConfig &config,
                           const BT::RosNodeParams &params)
    : BT::RosActionNode<SegmentLeaves>(name, config, params) {}

BT::PortsList HarvestFruit::providedPorts() {
  // Nothing to aim: it picks from whatever the robot was last moved to, which
  // is the same reason the schema gives it no id.
  return providedBasicPorts({});
}

bool HarvestFruit::setGoal(Goal &goal) {
  (void)goal;
  RCLCPP_INFO(logger(), "Starting fruit harvest...");
  return true;
}

BT::NodeStatus HarvestFruit::onResultReceived(const WrappedResult &result) {
  RCLCPP_INFO(logger(), "Fruit harvest finished with code: %d",
              int(result.code));

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    if (result.result->success) {
      RCLCPP_INFO(logger(), "Fruit harvest succeeded: %s",
                  result.result->message.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    // The server's sentence, unedited. It is the only thing downstream has to
    // go on before the camera is asked, and paraphrasing it here would put a
    // second account of the same event into the log the agents read.
    RCLCPP_ERROR(logger(), "Fruit harvest failed: %s",
                 result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger(), "Fruit harvest was canceled");
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_ERROR(logger(), "Fruit harvest was aborted: %s",
                 result.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus HarvestFruit::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Fruit harvest state: %s",
              feedback->current_state.c_str());
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
