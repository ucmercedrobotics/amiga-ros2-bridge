#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class DummyNav2NavigateToPoseServer : public rclcpp::Node {
 public:
  DummyNav2NavigateToPoseServer() : Node("dummy_nav2_navigate_to_pose_server") {
    server_ = rclcpp_action::create_server<NavigateToPose>(
        this, "navigate_to_pose",
        std::bind(&DummyNav2NavigateToPoseServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&DummyNav2NavigateToPoseServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&DummyNav2NavigateToPoseServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "Dummy nav2 NavigateToPose action server ready on 'navigate_to_pose'");
  }

 private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const NavigateToPose::Goal> goal) {
    (void)uuid;
    const auto &p = goal->pose.pose.position;
    RCLCPP_INFO(
        this->get_logger(),
        "Received nav2 NavigateToPose goal -> position: (%.3f, %.3f, %.3f)",
        p.x, p.y, p.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToPose>>) {
    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToPose>>
          goal_handle) {
    std::thread([this, goal_handle]() {
      auto feedback = std::make_shared<NavigateToPose::Feedback>();
      auto result = std::make_shared<NavigateToPose::Result>();

      const auto goal = goal_handle->get_goal();
      const auto &p = goal->pose.pose.position;
      RCLCPP_INFO(this->get_logger(), "Executing goal to (%.3f, %.3f, %.3f)...",
                  p.x, p.y, p.z);

      const int steps = 10;
      for (int i = 1; i <= steps; ++i) {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal was canceled");
          return;
        }

        double progress = static_cast<double>(i) / steps;
        (void)progress;  // progress placeholder if not publishing custom
                         // feedback fields

        // Publish default-constructed feedback to keep clients alive
        goal_handle->publish_feedback(feedback);
        RCLCPP_DEBUG(this->get_logger(), "Published feedback step %d/%d", i,
                     steps);

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      }

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
    }).detach();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyNav2NavigateToPoseServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
