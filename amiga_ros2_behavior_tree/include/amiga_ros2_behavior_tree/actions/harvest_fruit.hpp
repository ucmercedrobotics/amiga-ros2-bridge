#pragma once

#include <kortex_interfaces/action/segment_leaves.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

namespace amiga_bt {

using SegmentLeaves = kortex_interfaces::action::SegmentLeaves;

/// Pick the fruit off the tree the robot is standing at.
///
/// Shares SampleLeaf's action *type* and not its action *name*: both are "put
/// the arm on the thing in front of you and report how it went", and the
/// result already carries the success flag and the sentence this needs. What
/// differs is which server answers, and therefore what the robot actually did
/// -- a mission that asked for fruit is not satisfied by a leaf.
///
/// Reporting that it picked nothing is an ordinary outcome of harvesting, so
/// it comes back the way every other failure does: FAILURE, with the server's
/// own words on it. Nothing here decides what that means. Whether an empty
/// tree is worth another robot's time is a judgement made further up, from the
/// camera, and a node that answered it would be answering it for every tree.
class HarvestFruit : public BT::RosActionNode<SegmentLeaves> {
 public:
  HarvestFruit(const std::string &name, const BT::NodeConfig &config,
               const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;
};

}  // namespace amiga_bt
