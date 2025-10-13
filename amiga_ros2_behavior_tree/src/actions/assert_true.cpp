#include "amiga_ros2_behavior_tree/actions/assert_true.hpp"

namespace amiga_bt
{

AssertTrue::AssertTrue(const std::string &name, const BT::NodeConfig &config)
    : BT::ConditionNode(name, config) {}

BT::PortsList AssertTrue::providedPorts()
{
    return {BT::InputPort<bool>("result")};
}

BT::NodeStatus AssertTrue::tick()
{
    bool result;
    if (!getInput("result", result))
    {
        throw BT::RuntimeError("missing required input [result]");
    }
    RCLCPP_INFO(rclcpp::get_logger("AssertTrue"), "Asserting result is true: %s", result ? "true" : "false");
    return (result == true) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace amiga_bt
