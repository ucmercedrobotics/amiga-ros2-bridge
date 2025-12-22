#include "amiga_ros2_behavior_tree/actions/follow_person.hpp"

namespace amiga_bt {

FollowPerson::FollowPerson(const std::string &name,
                                     const BT::NodeConfig &config,
                                     const BT::RosNodeParams &params)
    : BT::RosActionNode<FollowPersonAction>(name, config, params) {}

BT::PortsList FollowPerson::providedPorts() {
  return providedBasicPorts({});
}

bool FollowPerson::setGoal(Goal &goal) {
  RCLCPP_INFO(logger(), "Following person");

  (void)goal;

  return true;
}

BT::NodeStatus FollowPerson::onResultReceived(
    const WrappedResult &result) {
  RCLCPP_INFO(logger(), "Following finished with code: %d", int(result.code));
  // Check if the action succeeded
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger(), "Following succeeded!");
    return BT::NodeStatus::SUCCESS;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger(), "Following was canceled");
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_ERROR(logger(), "Following failed or was aborted");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus FollowPerson::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Following status: (%s)", feedback->status.c_str());
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
