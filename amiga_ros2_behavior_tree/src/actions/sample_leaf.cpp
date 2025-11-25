#include "amiga_ros2_behavior_tree/actions/sample_leaf.hpp"

#include <cmath>

namespace amiga_bt {

SampleLeaf::SampleLeaf(const std::string &name, const BT::NodeConfig &config,
                       const BT::RosNodeParams &params)
    : BT::RosActionNode<NavigateViaLidar>(name, config, params) {
  auto node = node_.lock();
  if (node) {
    gps_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/filtered", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          last_gps_ = *msg;
        });
  }
}

BT::PortsList SampleLeaf::providedPorts() {
  return providedBasicPorts(
      {BT::InputPort<double>("latitude"), BT::InputPort<double>("longitude")});
}

bool SampleLeaf::setGoal(Goal &goal) {
  double lat, lon;

  if (!getInput("latitude", lat) || !getInput("longitude", lon)) {
    RCLCPP_ERROR(logger(), "Missing latitude/longitude input");
    return false;
  }

  if (!last_gps_.has_value()) {
    RCLCPP_ERROR(logger(), "No GPS data available yet");
    return false;
  }

  double current_lat = last_gps_->latitude;
  double current_lon = last_gps_->longitude;
  double lat1_rad = current_lat * M_PI / 180.0;
  double lon1_rad = current_lon * M_PI / 180.0;
  double lat2_rad = lat * M_PI / 180.0;
  double lon2_rad = lon * M_PI / 180.0;
  double dlon = lon2_rad - lon1_rad;
  double y = std::sin(dlon) * std::cos(lat2_rad);
  double x = std::cos(lat1_rad) * std::sin(lat2_rad) -
             std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon);
  double bearing = std::atan2(y, x);

  goal.object_angle = bearing * 180.0 / M_PI;

  RCLCPP_INFO(logger(),
              "Moving to GPS location: (%.8f, %.8f) from (%.8f, %.8f), angle: "
              "%.2f degrees",
              lat, lon, current_lat, current_lon, goal.object_angle);
  return true;
}

BT::NodeStatus SampleLeaf::onResultReceived(const WrappedResult &result) {
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

BT::NodeStatus SampleLeaf::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "Distance from goal: (%.6f)",
              feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
