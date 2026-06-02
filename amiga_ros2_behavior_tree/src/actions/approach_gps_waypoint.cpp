#include "amiga_ros2_behavior_tree/actions/approach_gps_waypoint.hpp"

namespace amiga_bt {

ApproachGPSWaypoint::ApproachGPSWaypoint(const std::string &name,
                                         const BT::NodeConfig &config,
                                         const BT::RosNodeParams &params)
    : BT::RosActionNode<GPSWaypoint>(name, config, params) {}

BT::PortsList ApproachGPSWaypoint::providedPorts() {
  return providedBasicPorts({BT::InputPort<double>("latitude"),
                             BT::InputPort<double>("longitude")});
}

bool ApproachGPSWaypoint::setGoal(Goal &goal) {
  double lat, lon;

  if (!getInput("latitude", lat) || !getInput("longitude", lon)) {
    RCLCPP_ERROR(logger(), "Missing latitude/longitude input");
    return false;
  }

  goal.lat = lat;
  goal.lon = lon;

  RCLCPP_INFO(logger(), "Approaching GPS waypoint: (%.8f, %.8f)", lat, lon);
  return true;
}

BT::NodeStatus ApproachGPSWaypoint::onResultReceived(
    const WrappedResult &result) {
  RCLCPP_INFO(logger(), "Approach finished with code: %d", int(result.code));
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger(), "Approach succeeded!");
    return BT::NodeStatus::SUCCESS;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(logger(), "Approach was canceled");
    return BT::NodeStatus::FAILURE;
  } else {
    RCLCPP_ERROR(logger(), "Approach failed or was aborted");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus ApproachGPSWaypoint::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Distance from goal: (%.6f)", feedback->dist);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
