#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <kortex_interfaces/action/move_to.hpp>

namespace amiga_bt {

using MoveArmToPositionAction = kortex_interfaces::action::MoveTo;

class MoveArmToPosition : public BT::RosActionNode<MoveArmToPositionAction> {
 public:
  MoveArmToPosition(const std::string &name, const BT::NodeConfig &config,
            const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace amiga_bt
