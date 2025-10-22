#pragma once

#include <amiga_interfaces/action/navigate_to_pose_in_frame.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

namespace amiga_bt {

using NavigateToPoseInFrame = amiga_interfaces::action::NavigateToPoseInFrame;

class MoveToRelativeLocation : public BT::RosActionNode<NavigateToPoseInFrame> {
 public:
  MoveToRelativeLocation(const std::string &name, const BT::NodeConfig &config,
                         const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace amiga_bt
