#include "amiga_ros2_behavior_tree/actions/sample_leaf.hpp"

namespace amiga_bt
{

SampleLeaf::SampleLeaf(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList SampleLeaf::providedPorts()
{
    return {BT::InputPort<std::string>("action_name")};
}

BT::NodeStatus SampleLeaf::tick()
{
    RCLCPP_INFO(rclcpp::get_logger("SampleLeaf"), "SampleLeaf executed");
    return BT::NodeStatus::SUCCESS;
}

} // namespace amiga_bt
