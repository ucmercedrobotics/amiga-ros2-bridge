#pragma once

#include <amiga_navigation_interfaces/action/tree_id_waypoint.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <std_msgs/msg/u_int32.hpp>

namespace amiga_bt {

using TreeIDWaypoint = amiga_navigation_interfaces::action::TreeIDWaypoint;

class MoveToTreeID : public BT::RosActionNode<TreeIDWaypoint> {
 public:
  MoveToTreeID(const std::string &name, const BT::NodeConfig &config,
                    const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;

 private:
  // The tree this plan step is working on, announced for anything that acts at
  // a tree and cannot tell from where it is standing which tree that is. A
  // lane runs between two rows, so position alone puts the robot exactly
  // between the tree it was sent to and the one behind it -- see
  // sim_harvest_fruit_server's tree_here(). The plan is the only thing that
  // knows, so the plan says.
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr target_pub_;
};

}  // namespace amiga_bt
