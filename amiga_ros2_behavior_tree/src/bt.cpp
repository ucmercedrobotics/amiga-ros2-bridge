// main.cpp (sketch)
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

// BehaviorTree.ROS2 wrappers (headers live in the BehaviorTree.ROS2 package)
#include <behaviortree_ros2/bt_action_node.hpp>  // for RosActionNode examples
#include <behaviortree_ros2/ros_node_params.hpp> // RosNodeParams (used when registering)
#include "behaviortree_ros2/plugins.hpp"
#include <nav2_msgs/action/follow_gps_waypoints.hpp> // for MoveToGPSLocation example

using namespace BT;
using FollowGPSWaypoints = nav2_msgs::action::FollowGPSWaypoints;

// ---- Example node implementations ----
// NOTE: constructors accept (const std::string&, const NodeConfig&, const RosNodeParams&)
// so factory.registerNodeType<T>("Name", params) can inject the rclcpp::Node.

class DetectObject : public SyncActionNode
{
public:
    DetectObject(const std::string &name, const NodeConfig &config)
        : SyncActionNode(name, config) {}
    static PortsList providedPorts()
    {
        return {InputPort<std::string>("object"), OutputPort<bool>("detected")};
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

class MoveToGPSLocation : public RosActionNode<FollowGPSWaypoints>
{
public:
    MoveToGPSLocation(const std::string &name,
                      const BT::NodeConfig &config,
                      const BT::RosNodeParams &params)
        : RosActionNode<FollowGPSWaypoints>(name, config, params) {}

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

        geographic_msgs::msg::GeoPose pose;
        pose.position.latitude = lat;
        pose.position.longitude = lon;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        goal.gps_poses = {pose};
        goal.number_of_loops = 1;
        goal.goal_index = 0;
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &result) override
    {
        RCLCPP_INFO(logger(), "Navigation finished with status: %d", int(result.result->error_code));
        RCLCPP_DEBUG(logger(), "Number of missed waypoints: %ld", result.result->missed_waypoints.size());
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
    {
        (void)feedback;
        RCLCPP_INFO(logger(), "Navigating...");
        return BT::NodeStatus::RUNNING;
    }
};

class BoolCondition : public ConditionNode
{
public:
    BoolCondition(const std::string &name, const NodeConfig &config)
        : ConditionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {InputPort<bool>("value"), InputPort<bool>("expected")};
    }

    NodeStatus tick() override
    {
        bool value;
        if (!getInput("value", value))
        {
            throw RuntimeError("missing required input [value]");
        }
        bool expected;
        if (!getInput("expected", expected))
        {
            throw RuntimeError("missing required input [expected]");
        }
        return (value == expected) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
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
    factory.registerNodeType<BoolCondition>("BoolCondition");

    // Create tree from XML
    auto tree = factory.createTreeFromFile("/amiga-ros2-bridge/amiga_ros2_behavior_tree/examples/test.xml");

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
