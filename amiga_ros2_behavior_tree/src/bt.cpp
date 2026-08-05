#include <behaviortree_cpp/bt_factory.h>

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
#include "amiga_ros2_behavior_tree/actions/approach_gps_waypoint.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_gps_location.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_aisle_head.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_tree_id.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_relative_location.hpp"
#include "amiga_ros2_behavior_tree/actions/orient_robot_heading.hpp"
#include "amiga_ros2_behavior_tree/actions/sample_leaf.hpp"
#include "amiga_ros2_behavior_tree/actions/follow_person.hpp"
#include "amiga_ros2_behavior_tree/actions/arm_move_to.hpp"
#include "amiga_ros2_behavior_tree/fault_reporter.hpp"
#include "amiga_ros2_behavior_tree/xml_validation.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using namespace BT;
using namespace amiga_bt;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("bt_runner");

  std::srand(static_cast<unsigned int>(std::time(nullptr)));

  nh->declare_parameter<std::string>("mission_topic", std::string("/mission/xml"));
  nh->declare_parameter<bool>("xml_validation", true);
  // The tree is the catalyst for the whole replanning and coordination
  // pipeline; this is the topic it fires on. See fault_reporter.hpp.
  nh->declare_parameter<std::string>("fault_topic", std::string("/bt/status_change"));
  nh->declare_parameter<bool>("fault_reporting", true);
  // One report per node per interval. A ReactiveSequence re-ticks a failing
  // condition at the tick rate, and every report downstream costs an LLM call.
  nh->declare_parameter<double>("fault_min_interval_sec", 5.0);
  std::string mission_topic;
  bool xml_validation_enabled;
  std::string fault_topic;
  bool fault_reporting_enabled;
  double fault_min_interval_sec;
  nh->get_parameter("mission_topic", mission_topic);
  nh->get_parameter("xml_validation", xml_validation_enabled);
  nh->get_parameter("fault_topic", fault_topic);
  nh->get_parameter("fault_reporting", fault_reporting_enabled);
  nh->get_parameter("fault_min_interval_sec", fault_min_interval_sec);

  BehaviorTreeFactory factory;
  RosNodeParams ros_params;
  ros_params.nh = nh;

  factory.registerNodeType<MoveToGPSLocation>("MoveToGPSLocation", ros_params);
  factory.registerNodeType<ApproachGPSWaypoint>("ApproachGPSWaypoint", ros_params);
  factory.registerNodeType<MoveToTreeID>("MoveToTreeID", ros_params);
  factory.registerNodeType<MoveToAisleHead>("MoveToAisleHead", ros_params);
  factory.registerNodeType<MoveToRelativeLocation>("MoveToRelativeLocation",
                                                   ros_params);
  factory.registerNodeType<OrientRobotHeading>("OrientRobotHeading",
                                                   ros_params);
  factory.registerNodeType<FollowPerson>("FollowPerson", ros_params);
  factory.registerNodeType<SampleLeaf>("SampleLeaf", ros_params);
  factory.registerNodeType<MoveArmToPosition>("MoveArmToPosition", ros_params);
  factory.registerNodeType<DetectObject>("DetectObject");
  // conditional nodes
  factory.registerNodeType<AssertTrue>("AssertTrue");
  factory.registerNodeType<CheckValue>("CheckValue");

  std::string schema_path;
  try {
    std::string share_dir = ament_index_cpp::get_package_share_directory(
        "amiga_ros2_behavior_tree");
    schema_path = share_dir + AMIGA_SCHEMA_DEFAULT_PATH;
  } catch (const std::exception &e) {
    RCLCPP_WARN(nh->get_logger(),
                "Could not resolve package share for default schema: %s",
                e.what());
    schema_path = AMIGA_SCHEMA_DEFAULT_PATH;
  }
  nh->declare_parameter<std::string>("mission_schema", schema_path);
  nh->get_parameter("mission_schema", schema_path);

  // Created once, outside the mission loop: a fault must outlive the tree that
  // produced it, because what reads it starts up in response to it.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub;
  if (fault_reporting_enabled) {
    fault_pub = makeFaultPublisher(nh, fault_topic);
  }

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
      if (xml_validation_enabled && !xml_validation::validate(mission_in, schema_path, err)) {
        RCLCPP_ERROR(nh->get_logger(),
                     "Mission XML schema validation failed: %s", err.c_str());
        continue;
      }
      tree = factory.createTreeFromText(mission_in);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(nh->get_logger(), "Failed to create BT from mission: %s",
                   e.what());
      continue;
    }

    // Constructed per mission, because it subscribes to this tree's nodes and
    // the previous tree no longer exists. Destroyed at the end of the scope,
    // which unsubscribes before the tree is replaced.
    std::unique_ptr<FaultReporter> fault_reporter;
    if (fault_pub) {
      fault_reporter = std::make_unique<FaultReporter>(
          tree, nh, fault_pub, fault_min_interval_sec);
    }

    RCLCPP_INFO(nh->get_logger(), "Starting mission execution...");

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
      auto status = tree.tickOnce();
      if (status == BT::NodeStatus::SUCCESS ||
          status == BT::NodeStatus::FAILURE) {
        RCLCPP_INFO(nh->get_logger(), "Mission finished with status: %s",
                    toStr(status, true).c_str());
        if (fault_reporter) {
          fault_reporter->reportTreeOutcome(status);
        }
        break;
      }
      rclcpp::spin_some(nh);
      rate.sleep();
    }
  }

  rclcpp::shutdown();
  return 0;
}
