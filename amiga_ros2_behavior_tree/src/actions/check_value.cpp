#include "amiga_ros2_behavior_tree/actions/check_value.hpp"

namespace amiga_bt
{

CheckValue::CheckValue(const std::string &name, const BT::NodeConfig &config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckValue::providedPorts()
{
    return {BT::InputPort<bool>("value")};
}

BT::NodeStatus CheckValue::tick()
{
    bool value;
    if (!getInput("value", value))
    {
        throw BT::RuntimeError("missing required input [value]");
    }
    RCLCPP_INFO(rclcpp::get_logger("CheckValue"), "Asserting value is true: %s", value ? "true" : "false");
    return (value == true) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace amiga_bt
