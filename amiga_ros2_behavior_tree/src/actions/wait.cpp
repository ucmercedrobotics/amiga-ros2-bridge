#include "amiga_ros2_behavior_tree/actions/wait.hpp"

namespace amiga_bt
{

Wait::Wait(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList Wait::providedPorts()
{
    return {BT::InputPort<double>("seconds",
                                  "How long to hold position before succeeding")};
}

BT::NodeStatus Wait::onStart()
{
    double seconds = 0.0;
    if (!getInput("seconds", seconds))
    {
        throw BT::RuntimeError("missing required input [seconds]");
    }
    if (seconds < 0.0)
    {
        throw BT::RuntimeError("[seconds] must not be negative");
    }
    RCLCPP_INFO(rclcpp::get_logger("Wait"), "Holding position for %.1f s", seconds);
    // steady_clock rather than the node clock, to match the wall-clock
    // rclcpp::Rate the tree is ticked on. Under a real-time factor below 1
    // this waits the stated number of *wall* seconds, which is the same
    // timebase every other deadline in bt.cpp uses.
    deadline_ = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(seconds));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Wait::onRunning()
{
    if (std::chrono::steady_clock::now() < deadline_)
    {
        return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(rclcpp::get_logger("Wait"), "Wait finished");
    return BT::NodeStatus::SUCCESS;
}

void Wait::onHalted()
{
    RCLCPP_INFO(rclcpp::get_logger("Wait"), "Wait halted before its deadline");
}

} // namespace amiga_bt
