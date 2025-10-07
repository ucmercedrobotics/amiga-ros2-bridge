// main.cpp (sketch)
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

// BehaviorTree.ROS2 wrappers (headers live in the BehaviorTree.ROS2 package)
#include <behaviortree_ros2/bt_action_node.hpp>  // for RosActionNode examples
#include <behaviortree_ros2/ros_node_params.hpp> // RosNodeParams (used when registering)
#include "behaviortree_ros2/plugins.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace BT;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

// ---- Example node implementations ----
// NOTE: constructors accept (const std::string&, const NodeConfig&, const RosNodeParams&)
// so factory.registerNodeType<T>("Name", params) can inject the rclcpp::Node.

class TakeAmbientTemperature : public SyncActionNode
{
public:
    TakeAmbientTemperature(const std::string &name, const NodeConfig &config)
        : SyncActionNode(name, config) {}
    static PortsList providedPorts()
    {
        return {InputPort<std::string>("action_name"), OutputPort<double>("temperature")};
    }
    NodeStatus tick() override
    {
        double temperature = 20.0 + (std::rand() % 1000) / 100.0; // Simulate reading temperature
        setOutput("temperature", temperature);
        RCLCPP_INFO(rclcpp::get_logger("TakeAmbientTemperature"), "Ambient temperature: %.2f C", temperature);
        return NodeStatus::SUCCESS;
    }
};

class DetectObject : public SyncActionNode
{
public:
    DetectObject(const std::string &name, const NodeConfig &config)
        : SyncActionNode(name, config) {}
    static PortsList providedPorts()
    {
        return {InputPort<std::string>("object"), InputPort<std::string>("action_name"), OutputPort<bool>("detected")};
    }

    NodeStatus tick() override
    {
        std::string object;
        // Simulate object detection
        bool detected = (std::rand() % 2) == 0;

        if (!getInput("object", object))
        {
            throw RuntimeError("missing required input [object]");
        }
        setOutput("detected", detected);

        RCLCPP_INFO(rclcpp::get_logger("DetectObject"), "Object '%s' detected: %s", object.c_str(), detected ? "true" : "false");
        return NodeStatus::SUCCESS;
    }
};

class SampleLeaf : public SyncActionNode
{
public:
    SampleLeaf(const std::string &name, const NodeConfig &config)
        : SyncActionNode(name, config) {}
    static PortsList providedPorts()
    {
        return {InputPort<std::string>("action_name")};
    }

    NodeStatus tick() override
    {
        RCLCPP_INFO(rclcpp::get_logger("SampleLeaf"), "SampleLeaf executed");
        return NodeStatus::SUCCESS;
    }
};

class MoveToGPSLocation : public RosActionNode<NavigateToPose>
{
public:
    MoveToGPSLocation(const std::string &name,
                      const BT::NodeConfig &config,
                      const BT::RosNodeParams &params)
        : RosActionNode<NavigateToPose>(name, config, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::InputPort<double>("latitude"), BT::InputPort<double>("longitude")});
    }

    // Convert GPS (lat, lon) -> PoseStamped
    bool setGoal(Goal &goal) override
    {
        double lat, lon;

        if (!getInput("latitude", lat) || !getInput("longitude", lon))
        {
            RCLCPP_ERROR(logger(), "Missing latitude/longitude input");
            return false;
        }

        // For this example, we'll do a simple conversion (this would need proper GPS->XY conversion in real use)
        // This is a simplified conversion - in real applications you'd use proper coordinate transformation
        goal.pose.header.frame_id = "map";
        auto node_ptr = node_.lock();
        if (node_ptr) {
            goal.pose.header.stamp = node_ptr->now();
        }

        // Simple conversion: treat lat/lon as x/y coordinates (for demonstration)
        // In a real application, you'd use proper GPS coordinate conversion libraries
        goal.pose.pose.position.x = lon * 111320.0; // rough meters per degree longitude at equator
        goal.pose.pose.position.y = lat * 110540.0; // rough meters per degree latitude
        goal.pose.pose.position.z = 0.0;

        // Set orientation (facing north)
        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z = 0.0;
        goal.pose.pose.orientation.w = 1.0;

        goal.behavior_tree = "";

        RCLCPP_INFO(logger(), "Moving to GPS location: (%.6f, %.6f) -> Pose: (%.2f, %.2f)",
                    lat, lon, goal.pose.pose.position.x, goal.pose.pose.position.y);
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &result) override
    {
        RCLCPP_INFO(logger(), "Navigation finished successfully: %d", int(result.code));
        // NavigateToPose::Result doesn't have error_code field like FollowGPSWaypoints did
        // The result is simple - if we get here, navigation completed
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
    {
        RCLCPP_INFO(logger(), "Current position: (%.2f, %.2f)",
                    feedback->current_pose.pose.position.x,
                    feedback->current_pose.pose.position.y);
        return BT::NodeStatus::RUNNING;
    }
};

class CheckValue : public ConditionNode
{
public:
    CheckValue(const std::string &name, const NodeConfig &config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {InputPort<bool>("value")};
    }

    NodeStatus tick() override
    {
        bool value;
        if (!getInput("value", value))
        {
            throw RuntimeError("missing required input [value]");
        }
        RCLCPP_INFO(rclcpp::get_logger("CheckValue"), "Asserting value is true: %s", value ? "true" : "false");
        return (value == true) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class AssertTrue : public ConditionNode
{
public:
    AssertTrue(const std::string &name, const NodeConfig &config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {InputPort<bool>("result")};
    }

    NodeStatus tick() override
    {
        bool result;
        if (!getInput("result", result))
        {
            throw RuntimeError("missing required input [result]");
        }
        RCLCPP_INFO(rclcpp::get_logger("AssertTrue"), "Asserting result is true: %s", result ? "true" : "false");
        return (result == true) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

// ---- main ----
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("bt_runner");

    std::srand(std::time(nullptr));

    BehaviorTreeFactory factory;

    // Provide the ROS node to the factory so it can inject it into node constructors.
    RosNodeParams ros_params;
    ros_params.nh = nh;
    // optional: ros_params.default_port_value = "...";

    // Register nodes (these overloads let the factory pass ros_params to constructors)
    factory.registerNodeType<MoveToGPSLocation>("MoveToGPSLocation", ros_params);
    factory.registerNodeType<DetectObject>("DetectObject");
    factory.registerNodeType<AssertTrue>("AssertTrue");
    factory.registerNodeType<TakeAmbientTemperature>("TakeAmbientTemperature");
    factory.registerNodeType<SampleLeaf>("SampleLeaf");

    // Create tree from XML
    auto tree = factory.createTreeFromFile("/amiga-ros2-bridge/amiga_ros2_behavior_tree/examples/sample_leafs.xml");

    // Run loop: tick the tree and spin the ROS node
    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        auto status = tree.tickOnce();
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_INFO(nh->get_logger(), "Mission finished with status: %s",
                        toStr(status, true).c_str());
            break;
        }
        rclcpp::spin_some(nh);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
