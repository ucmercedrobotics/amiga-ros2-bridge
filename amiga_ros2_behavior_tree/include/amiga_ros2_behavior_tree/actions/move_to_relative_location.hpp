#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace amiga_bt {

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class MoveToRelativeLocation : public BT::RosActionNode<NavigateToPose> {
 public:
  MoveToRelativeLocation(const std::string &name, const BT::NodeConfig &config,
                         const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;

 private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace amiga_bt
