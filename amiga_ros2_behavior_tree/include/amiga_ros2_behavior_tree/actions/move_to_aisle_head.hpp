#pragma once

#include <amiga_navigation_interfaces/action/move_to_aisle_head.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

namespace amiga_bt {

using MoveToAisleHeadAction = amiga_navigation_interfaces::action::MoveToAisleHead;

class MoveToAisleHead : public BT::RosActionNode<MoveToAisleHeadAction> {
 public:
  MoveToAisleHead(const std::string &name, const BT::NodeConfig &config,
                    const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace amiga_bt
