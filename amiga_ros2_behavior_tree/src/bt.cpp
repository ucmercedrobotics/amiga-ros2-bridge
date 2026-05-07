#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <optional>
#include <string>

#include "amiga_ros2_behavior_tree/actions/assert_true.hpp"
#include "amiga_ros2_behavior_tree/actions/check_value.hpp"
#include "amiga_ros2_behavior_tree/actions/detect_object.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_gps_location.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_aisle_head.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_tree_id.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_relative_location.hpp"
#include "amiga_ros2_behavior_tree/actions/orient_robot_heading.hpp"
#include "amiga_ros2_behavior_tree/actions/sample_leaf.hpp"
#include "amiga_ros2_behavior_tree/actions/follow_person.hpp"
#include "amiga_ros2_behavior_tree/xml_validation.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using namespace BT;
using namespace amiga_bt;

// ---------------------------------------------------------------------------
// BTStatusPublisher
// Hooks into BT.CPP's StatusChangeLogger and publishes node status changes
// to /bt/status_change as a JSON string. The agent subscribes to this topic
// to detect capability gaps at runtime.
//
// Only FAILURE and RUNNING→IDLE (unexpected halt) transitions are published
// to avoid flooding the topic during normal execution.
// ---------------------------------------------------------------------------
class BTStatusPublisher : public BT::StatusChangeLogger
{
public:
  BTStatusPublisher(BT::Tree & tree, rclcpp::Node::SharedPtr node)
  : BT::StatusChangeLogger(tree.rootNode()),
    pub_(node->create_publisher<std_msgs::msg::String>("/bt/status_change", 10)),
    logger_(node->get_logger())
  {
  }

  void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override
  {
    // Only publish leaf node failures — control nodes (Sequence, Fallback, etc.)
    // propagate failure upward and are not capability gaps
    if (node.type() == BT::NodeType::CONTROL || node.type() == BT::NodeType::DECORATOR) {
      return;
    }

    if (status != BT::NodeStatus::FAILURE &&
      !(prev_status == BT::NodeStatus::RUNNING && status == BT::NodeStatus::IDLE))
    {
      return;
    }

    auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(timestamp).count();

    // Build JSON manually — no external dep required
    std::string payload =
      "{"
      "\"node\":\"" + node.name() + "\","
      "\"type\":\"" + node.registrationName() + "\","
      "\"prev_status\":\"" + BT::toStr(prev_status) + "\","
      "\"status\":\"" + BT::toStr(status) + "\","
      "\"timestamp_ms\":" + std::to_string(elapsed_ms) +
      "}";

    std_msgs::msg::String msg;
    msg.data = payload;
    pub_->publish(msg);

    RCLCPP_WARN(
      logger_,
      "[BTStatusPublisher] %s (%s): %s → %s",
      node.name().c_str(),
      node.registrationName().c_str(),
      BT::toStr(prev_status).c_str(),
      BT::toStr(status).c_str());
  }

  void flush() override {}

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Logger logger_;
};

// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("bt_runner");

  std::srand(static_cast<unsigned int>(std::time(nullptr)));

  nh->declare_parameter<std::string>("mission_topic", std::string("/mission/xml"));
  nh->declare_parameter<bool>("xml_validation", true);

  std::string mission_topic;
  bool xml_validation_enabled;
  nh->get_parameter("mission_topic", mission_topic);
  nh->get_parameter("xml_validation", xml_validation_enabled);

  BehaviorTreeFactory factory;

  RosNodeParams ros_params;
  ros_params.nh = nh;

  factory.registerNodeType<MoveToGPSLocation>("MoveToGPSLocation", ros_params);
  factory.registerNodeType<MoveToTreeID>("MoveToTreeID", ros_params);
  factory.registerNodeType<MoveToAisleHead>("MoveToAisleHead", ros_params);
  factory.registerNodeType<MoveToRelativeLocation>("MoveToRelativeLocation", ros_params);
  factory.registerNodeType<OrientRobotHeading>("OrientRobotHeading", ros_params);
  factory.registerNodeType<FollowPerson>("FollowPerson", ros_params);
  factory.registerNodeType<SampleLeaf>("SampleLeaf", ros_params);
  factory.registerNodeType<DetectObject>("DetectObject");

  // conditional nodes
  factory.registerNodeType<AssertTrue>("AssertTrue");
  factory.registerNodeType<CheckValue>("CheckValue");

  std::string schema_path;
  try {
    std::string share_dir =
      ament_index_cpp::get_package_share_directory("amiga_ros2_behavior_tree");
    schema_path = share_dir + AMIGA_SCHEMA_DEFAULT_PATH;
  } catch (const std::exception &e) {
    RCLCPP_WARN(nh->get_logger(),
      "Could not resolve package share for default schema: %s", e.what());
    schema_path = AMIGA_SCHEMA_DEFAULT_PATH;
  }

  nh->declare_parameter<std::string>("mission_schema", schema_path);
  nh->get_parameter("mission_schema", schema_path);

  std::mutex mtx;
  std::optional<std::string> pending_mission;

  auto sub = nh->create_subscription<std_msgs::msg::String>(
    mission_topic, 10,
    [&](const std_msgs::msg::String &msg) {
      std::lock_guard<std::mutex> lk(mtx);
      pending_mission = msg.data;
    });

  rclcpp::Rate spin_rate(20);

  while (rclcpp::ok()) {
    std::optional<std::string> mission_in_opt;
    {
      std::lock_guard<std::mutex> lk(mtx);
      mission_in_opt.swap(pending_mission);
    }

    if (!mission_in_opt.has_value()) {
      rclcpp::spin_some(nh);
      spin_rate.sleep();
      continue;
    }

    const std::string &mission_in = *mission_in_opt;

    Tree tree;
    try {
      std::string err;
      if (xml_validation_enabled &&
        !xml_validation::validate(mission_in, schema_path, err))
      {
        RCLCPP_ERROR(nh->get_logger(),
          "Mission XML schema validation failed: %s", err.c_str());
        continue;
      }
      tree = factory.createTreeFromText(mission_in);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(nh->get_logger(), "Failed to create BT from mission: %s", e.what());
      continue;
    }

    // Attach publisher — scoped to this mission's tree lifetime
    BTStatusPublisher status_publisher(tree, nh);

    RCLCPP_INFO(nh->get_logger(), "Starting mission execution...");

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
      auto status = tree.tickOnce();

      if (status == BT::NodeStatus::SUCCESS ||
        status == BT::NodeStatus::FAILURE)
      {
        RCLCPP_INFO(nh->get_logger(), "Mission finished with status: %s",
          toStr(status, true).c_str());
        break;
      }

      rclcpp::spin_some(nh);
      rate.sleep();
    }
  }

  rclcpp::shutdown();
  return 0;
}