#pragma once

#include <kortex_interfaces/action/segment_leaves.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

namespace amiga_bt {

using SegmentLeaves = kortex_interfaces::action::SegmentLeaves;

class SampleLeaf : public BT::RosActionNode<SegmentLeaves> {
 public:
  SampleLeaf(const std::string &name, const BT::NodeConfig &config,
             const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace amiga_bt
