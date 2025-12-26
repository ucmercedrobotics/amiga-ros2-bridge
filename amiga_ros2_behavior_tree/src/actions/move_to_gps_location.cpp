#include "amiga_ros2_behavior_tree/actions/move_to_gps_location.hpp"

namespace amiga_bt {

MoveToGPSLocation::MoveToGPSLocation(const std::string &name,
                                     const BT::NodeConfig &config,
                                     const BT::RosNodeParams &params)
    : BT::RosActionNode<GPSWaypoint>(name, config, params) {}

BT::PortsList MoveToGPSLocation::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("latitude"),
                             BT::InputPort<double>("longitude")});
}

bool MoveToGPSLocation::setGoal(Goal &goal) {
  double lat, lon;

  if (!getInput("latitude", lat) || !getInput("longitude", lon)) {
    RCLCPP_ERROR(logger(), "Missing latitude/longitude input");
    return false;
  }

  goal.lat = lat;
  goal.lon = lon;

  RCLCPP_INFO(logger(), "Moving to GPS location: (%.8f, %.8f)", lat, lon);
  return true;
}

BT::NodeStatus MoveToGPSLocation::onResultReceived(
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

BT::NodeStatus MoveToGPSLocation::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Distance from goal: (%.6f)", feedback->dist);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
