#include "amiga_ros2_behavior_tree/actions/arm_move_to.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose.hpp>

namespace amiga_bt {

MoveArmToPosition::MoveArmToPosition(const std::string &name, const BT::NodeConfig &config,
                     const BT::RosNodeParams &params)
    : BT::RosActionNode<MoveArmToPositionAction>(name, config, params) {}

BT::PortsList MoveArmToPosition::providedPorts() {
  return providedBasicPorts(
      {BT::InputPort<std::string>("movement_link"), BT::InputPort<double>("x"),
       BT::InputPort<double>("y"), BT::InputPort<double>("z"),
       BT::InputPort<double>("roll"), BT::InputPort<double>("pitch"),
       BT::InputPort<double>("yaw")});
}

bool MoveArmToPosition::setGoal(Goal &goal) {
  std::string movement_link;
  double x, y, z;
  double roll, pitch, yaw;
  geometry_msgs::msg::Pose target_pose;

  if (!getInput("movement_link", movement_link)) {
    RCLCPP_ERROR(logger(), "Missing movement_link input");
    return false;
  }
  goal.movement_link = movement_link;

  if (getInput("x", x)) {
    target_pose.position.x = x;
  } else {
    RCLCPP_WARN(logger(), "No X movement specified, defaulting to 0.0.");
  }

  if (getInput("y", y)) {
    target_pose.position.y = y;
  } else {
    RCLCPP_WARN(logger(), "No Y movement specified, defaulting to 0.0.");
  }
  if (getInput("z", z)) {
    target_pose.position.z = z;
  } else {
    RCLCPP_WARN(logger(), "No Z movement specified, defaulting to 0.0.");
  }
  if (!getInput("roll", roll)) {
    roll = 0.0;
    RCLCPP_WARN(logger(), "No roll specified, defaulting to 0.0.");
  }
  if (!getInput("pitch", pitch)) {
    pitch = 0.0;
    RCLCPP_WARN(logger(), "No pitch specified, defaulting to 0.0.");
  }
  if (!getInput("yaw", yaw)) {
    yaw = 0.0;
    RCLCPP_WARN(logger(), "No yaw specified, defaulting to 0.0.");
  }
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  target_pose.orientation.x = quat.x();
  target_pose.orientation.y = quat.y();
  target_pose.orientation.z = quat.z();
  target_pose.orientation.w = quat.w();

  goal.pose = target_pose;

  RCLCPP_INFO(logger(), "Moving to position %f, %f, %f ",
              target_pose.position.x, target_pose.position.y,
              target_pose.position.z);
  return true;
}

BT::NodeStatus MoveArmToPosition::onResultReceived(const WrappedResult &result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (result.result->success) {
        RCLCPP_INFO(logger(), "MoveArmToPosition action successfully.");
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_ERROR(logger(), "MoveArmToPosition action failed.");
        return BT::NodeStatus::FAILURE;
      }
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "MoveArmToPosition action was aborted.");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(logger(), "MoveArmToPosition action was canceled.");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown result code received.");
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveArmToPosition::onFeedback(
    const std::shared_ptr<const Feedback> feedback) {
  RCLCPP_INFO(logger(), "%s", feedback->processing_status.c_str());
  return BT::NodeStatus::RUNNING;
}

}  // namespace amiga_bt
