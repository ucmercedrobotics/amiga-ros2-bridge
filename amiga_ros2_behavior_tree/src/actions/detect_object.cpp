#include "amiga_ros2_behavior_tree/actions/detect_object.hpp"

#include <cstdlib>

namespace amiga_bt
{

DetectObject::DetectObject(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList DetectObject::providedPorts()
{
    return {BT::InputPort<std::string>("object"), BT::InputPort<std::string>("action_name"), BT::OutputPort<bool>("detected")};
}

BT::NodeStatus DetectObject::tick()
{
    std::string object;
    bool detected = (std::rand() % 2) == 0;

    if (!getInput("object", object))
    {
        throw BT::RuntimeError("missing required input [object]");
    }
    setOutput("detected", detected);

    RCLCPP_INFO(rclcpp::get_logger("DetectObject"), "Object '%s' detected: %s", object.c_str(), detected ? "true" : "false");
    return BT::NodeStatus::SUCCESS;
}

} // namespace amiga_bt
