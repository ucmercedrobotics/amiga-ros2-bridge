#pragma once

#include <amiga_navigation_interfaces/action/navigate_via_lidar.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace amiga_bt {

using NavigateViaLidar = amiga_navigation_interfaces::action::NavigateViaLidar;

class SampleLeaf : public BT::RosActionNode<NavigateViaLidar> {
 public:
  SampleLeaf(const std::string &name, const BT::NodeConfig &config,
             const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult &result) override;
  BT::NodeStatus onFeedback(
      const std::shared_ptr<const Feedback> feedback) override;

 private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  std::optional<sensor_msgs::msg::NavSatFix> last_gps_;
};

}  // namespace amiga_bt
