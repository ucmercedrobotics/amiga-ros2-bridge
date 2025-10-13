#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace amiga_bt
{

class SampleLeaf : public BT::SyncActionNode
{
public:
    SampleLeaf(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

} // namespace amiga_bt
