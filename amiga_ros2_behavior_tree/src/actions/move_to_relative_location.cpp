#include "amiga_ros2_behavior_tree/actions/move_to_relative_location.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace amiga_bt {

MoveToRelativeLocation::MoveToRelativeLocation(const std::string &name,
                                               const BT::NodeConfig &config,
                                               const BT::RosNodeParams &params)
    : BT::RosActionNode<NavigateToPose>(name, config, params) {
  auto node = node_.lock();
  if (node) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
}

BT::PortsList MoveToRelativeLocation::providedPorts() {
  return providedBasicPorts(
      {BT::InputPort<double>("x"), BT::InputPort<double>("y"),
       BT::InputPort<bool>("absolute"), BT::InputPort<double>("yaw")});
}

bool MoveToRelativeLocation::setGoal(Goal &goal) {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  // specific to orientation
  // TODO: absolute handling is not supported
  bool absolute = false;
  std::string frame_id = "base_link";
  double z = 0.0;
  double w = 1.0;

  if (getInput("x", x) && getInput("y", y)) {
    RCLCPP_DEBUG(logger(), "Received x/y input. Making relative movement.");
  } else if (getInput("absolute", absolute) && getInput("yaw", yaw)) {
    RCLCPP_DEBUG(logger(), "Received yaw input. Making relative turn.");
    z = sin(yaw / 2.0);
    w = cos(yaw / 2.0);
  } else {
    RCLCPP_ERROR(logger(), "Missing x/y or yaw input");
    return false;
  }

  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger(), "Node is not available");
    return false;
  }

  geometry_msgs::msg::PoseStamped pose_in_base_link;
  pose_in_base_link.header.frame_id = frame_id;
  pose_in_base_link.header.stamp = rclcpp::Time(0);
  pose_in_base_link.pose.position.x = x;
  pose_in_base_link.pose.position.y = y;
  pose_in_base_link.pose.position.z = 0.0;
  pose_in_base_link.pose.orientation.x = 0.0;
  pose_in_base_link.pose.orientation.y = 0.0;
  pose_in_base_link.pose.orientation.z = z;
  pose_in_base_link.pose.orientation.w = w;

  geometry_msgs::msg::PoseStamped pose_in_map;
  const int max_retries = 10;
  const auto retry_delay = std::chrono::milliseconds(100);

  for (int attempt = 0; attempt < max_retries; ++attempt) {
    try {
      pose_in_map = tf_buffer_->transform(pose_in_base_link, "map",
                                          tf2::durationFromSec(0.5));

      goal.pose = pose_in_map;

      RCLCPP_INFO(
          logger(),
          "Transformed pose from base_link (%.2f, %.2f) to map (%.2f, %.2f)", x,
          y, pose_in_map.pose.position.x, pose_in_map.pose.position.y);

      return true;

    } catch (const tf2::TransformException &ex) {
      if (attempt < max_retries - 1) {
        RCLCPP_WARN(logger(), "Transform attempt %d/%d failed: %s. Retrying...",
                    attempt + 1, max_retries, ex.what());
        std::this_thread::sleep_for(retry_delay);
      } else {
        RCLCPP_ERROR(logger(), "Could not transform pose after %d attempts: %s",
                     max_retries, ex.what());
        return false;
      }
    }
  }

  return true;
}

BT::NodeStatus MoveToRelativeLocation::onResultReceived(
    const WrappedResult &result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(logger(), "Navigation succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(logger(), "Navigation failed with code: %d", int(result.code));
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveToRelativeLocation::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  tf2::Quaternion q(feedback->current_pose.pose.orientation.x,
                    feedback->current_pose.pose.orientation.y,
                    feedback->current_pose.pose.orientation.z,
                    feedback->current_pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, current_yaw;
  m.getRPY(roll, pitch, current_yaw);

  RCLCPP_INFO(logger(),
              "Distance remaining: %.2f m, Current pose: (%.2f, %.2f), Yaw: "
              "%.2f rad (%.1f deg)",
              feedback->distance_remaining,
              feedback->current_pose.pose.position.x,
              feedback->current_pose.pose.position.y, current_yaw,
              current_yaw * 180.0 / M_PI);
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
