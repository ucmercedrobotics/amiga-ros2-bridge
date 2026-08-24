#pragma once

#include <chrono>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace amiga_bt
{

/// Hold position for a fixed time, then succeed.
///
/// The only action here that does nothing. It exists because some obstructions
/// clear themselves and some do not, and until now a plan had no way to say
/// which it was looking at. A person walking an aisle and a fallen branch
/// produce the same Nav2 failure -- "failed to create plan" -- so retrying
/// immediately fails identically both times, and re-planning around a body
/// that will be gone in a minute is wasted travel. Waiting is the correct
/// answer to exactly one of those two, and the camera is what tells them
/// apart.
class Wait : public BT::StatefulActionNode
{
public:
    Wait(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::chrono::steady_clock::time_point deadline_;
};

} // namespace amiga_bt
