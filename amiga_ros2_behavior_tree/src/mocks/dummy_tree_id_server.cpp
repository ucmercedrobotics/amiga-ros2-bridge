#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amiga_navigation_interfaces/action/tree_id_waypoint.hpp"

using TreeIDWaypoint = amiga_navigation_interfaces::action::TreeIDWaypoint;

class DummyTreeIDActionServer : public rclcpp::Node {
 public:
  DummyTreeIDActionServer() : Node("dummy_tree_id_action_server") {
    server_ = rclcpp_action::create_server<TreeIDWaypoint>(
        this, "follow_tree_id_waypoint",
        std::bind(&DummyTreeIDActionServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&DummyTreeIDActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&DummyTreeIDActionServer::handle_accepted, this,
                  std::placeholders::_1));
  }

 private:
  rclcpp_action::Server<TreeIDWaypoint>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const TreeIDWaypoint::Goal> goal) {
    std::string uuid_str = "Received goal request with UUID: ";
    for (auto const &id : uuid) uuid_str += std::to_string(id) + " ";

    RCLCPP_DEBUG(this->get_logger(), "%s", uuid_str.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Received navigation goal to tree ID: %u", goal->tree_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<TreeIDWaypoint>>) {
    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<TreeIDWaypoint>>
          goal_handle) {
    // Run asynchronously
    std::thread([this, goal_handle]() {
      auto feedback = std::make_shared<TreeIDWaypoint::Feedback>();
      auto result = std::make_shared<TreeIDWaypoint::Result>();
      
      // Generate pseudo-random values based on tree_id
      uint32_t tree_id = goal_handle->get_goal()->tree_id;
      double lat = 37.0 + (tree_id % 1000) / 10000.0;  // ~37.0-37.1
      double lon = -122.0 + (tree_id % 1000) / 10000.0;  // ~-122.0--121.9
      
      uintptr_t addr = reinterpret_cast<uintptr_t>(this);
      double random_radians = ((addr % 10000) / 10000.0) * 2 * M_PI -
                              M_PI;  // Random value between -π and π
      
      result->lat = lat;
      result->lon = lon;
      result->object_angle = random_radians;

      RCLCPP_INFO(this->get_logger(), "Executing goal...");
      RCLCPP_INFO(this->get_logger(), "Navigating to tree ID: %u", tree_id);

      // Simulate navigation progress with feedback
      for (int i = 5; i > 0; i--) {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
        
        feedback->dist = i * 1.5;  // Distance decreasing
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m", feedback->dist);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), 
                  "Goal succeeded! Arrived at tree %u (%.6f, %.6f), angle: %.3f rad",
                  tree_id, lat, lon, random_radians);
    }).detach();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyTreeIDActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
