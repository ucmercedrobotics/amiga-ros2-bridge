#include <behaviortree_cpp/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "amiga_ros2_behavior_tree/actions/assert_true.hpp"
#include "amiga_ros2_behavior_tree/actions/check_value.hpp"
#include "amiga_ros2_behavior_tree/actions/detect_object.hpp"
#include "amiga_ros2_behavior_tree/actions/move_to_gps_location.hpp"
#include "amiga_ros2_behavior_tree/actions/sample_leaf.hpp"
#include "amiga_ros2_behavior_tree/mission_tcp.hpp"
#include "amiga_ros2_behavior_tree/xml_validation.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

using namespace BT;
using namespace amiga_bt;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("bt_runner");

  std::srand(static_cast<unsigned int>(std::time(nullptr)));

  int port = MISSION_TCP_DEFAULT_PORT;
  nh->declare_parameter<int>("mission_port", port);
  nh->get_parameter("mission_port", port);

  BehaviorTreeFactory factory;
  RosNodeParams ros_params;
  ros_params.nh = nh;

  factory.registerNodeType<MoveToGPSLocation>("MoveToGPSLocation", ros_params);
  factory.registerNodeType<DetectObject>("DetectObject");
  factory.registerNodeType<SampleLeaf>("SampleLeaf");
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

  while (rclcpp::ok()) {
    std::string mission_in;
    try {
      mission_in = mission_tcp::wait_for_mission_tcp(port, nh->get_logger());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(nh->get_logger(), "Failed to receive mission: %s", e.what());
      continue;
    }

    Tree tree;
    try {
      std::string err;
      if (!xml_validation::validate(mission_in, schema_path, err)) {
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

    RCLCPP_INFO(nh->get_logger(), "Starting mission execution...");

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      auto status = tree.tickOnce();
      if (status == BT::NodeStatus::SUCCESS ||
          status == BT::NodeStatus::FAILURE) {
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
