#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amiga_navigation_interfaces/action/tree_id_waypoint.hpp"
#include "amiga_ros2_behavior_tree/mocks/failure_modes.hpp"

using TreeIDWaypoint = amiga_navigation_interfaces::action::TreeIDWaypoint;

class DummyTreeIDActionServer : public rclcpp::Node {
 public:
  explicit DummyTreeIDActionServer(const std::string &name)
      : Node(name) {
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

      result->lat = lat;
      result->lon = lon;

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

      // The failure path, if this scenario has one for this tree. Placed
      // after the feedback loop so the robot gets most of the way there first,
      // which is what the real failures do: the orchard lookup happens on
      // arrival at the row, and the navigation abort comes from partway down
      // it. A goal that failed instantly would be distinguishable from a real
      // one by its timing alone.
      if (policy_ && policy_->shouldFail(tree_id)) {
        RCLCPP_ERROR(this->get_logger(), "%s",
                     amiga_bt::mocks::failureLog(policy_->mode()));
        goal_handle->abort(result);
        return;
      }

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(),
                  "Goal succeeded! Arrived at tree %u (%.6f, %.6f)",
                  tree_id, lat, lon);
    }).detach();
  }

  std::unique_ptr<amiga_bt::mocks::FailurePolicy> policy_;

 public:
  void configureFailures() {
    policy_ = std::make_unique<amiga_bt::mocks::FailurePolicy>(
        shared_from_this());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // The node name is a parameter because it is part of the evidence: the
  // triage agent's log window shows who logged each line, and a scenario that
  // wants a failure indistinguishable from a real one needs it attributed to
  // the node that would really have logged it.
  auto node = std::make_shared<DummyTreeIDActionServer>(
      amiga_bt::mocks::nodeName(argc, argv, "dummy_tree_id_action_server"));
  node->configureFailures();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
