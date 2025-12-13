#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amiga_navigation_interfaces/action/navigate_to_pose_in_frame.hpp"

using NavigateToPoseInFrame =
    amiga_navigation_interfaces::action::NavigateToPoseInFrame;

class DummyNavigateToPoseServer : public rclcpp::Node {
 public:
  DummyNavigateToPoseServer() : Node("dummy_navigate_to_pose_server") {
    server_ = rclcpp_action::create_server<NavigateToPoseInFrame>(
        this, "navigate_to_pose_in_frame",
        std::bind(&DummyNavigateToPoseServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&DummyNavigateToPoseServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&DummyNavigateToPoseServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Dummy NavigateToPoseInFrame action server ready");
  }

 private:
  rclcpp_action::Server<NavigateToPoseInFrame>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const NavigateToPoseInFrame::Goal> goal) {
    std::string uuid_str = "Received goal request with UUID: ";
    for (auto const &id : uuid) uuid_str += std::to_string(id) + " ";

    RCLCPP_DEBUG(this->get_logger(), "%s", uuid_str.c_str());

    double yaw = goal->yaw;

    RCLCPP_INFO(this->get_logger(),
                "Received navigation goal to pose: (%.2f, %.2f) "
                "with yaw: %.2f rad (%.1f deg)",
                goal->x, goal->y, yaw, yaw * 180.0 / M_PI);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<NavigateToPoseInFrame>>) {
    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<
                       rclcpp_action::ServerGoalHandle<NavigateToPoseInFrame>>
                           goal_handle) {
    std::thread([this, goal_handle]() {
      auto feedback = std::make_shared<NavigateToPoseInFrame::Feedback>();
      auto result = std::make_shared<NavigateToPoseInFrame::Result>();

      const auto goal = goal_handle->get_goal();

      double target_yaw = goal->yaw;
      double target_x = goal->x;
      double target_y = goal->y;

      RCLCPP_INFO(this->get_logger(), "Executing navigation goal...");
      RCLCPP_INFO(this->get_logger(),
                  "Target: Position=(%.2f, %.2f), Yaw=%.2f rad", target_x,
                  target_y, target_yaw);

      const int steps = 10;
      for (int i = 1; i <= steps; ++i) {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal was canceled");
          return;
        }

        double progress = static_cast<double>(i) / steps;

        double current_yaw = target_yaw * progress;

        double remaining_x = target_x - target_x * progress;
        double remaining_y = target_y - target_y * progress;
        feedback->distance_remaining =
            std::sqrt(remaining_x * remaining_x + remaining_y * remaining_y);

        double remaining_yaw = std::abs(target_yaw - current_yaw);
        feedback->yaw_remaining = remaining_yaw;

        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(),
                    "Progress: %.1f%%, Dist "
                    "remaining: %.2f m, Angle remaining: %.2f rad",
                    progress * 100.0, feedback->distance_remaining,
                    remaining_yaw);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(),
                  "Goal succeeded! Final position: (%.2f, %.2f), yaw: %.2f rad",
                  target_x, target_y, target_yaw);
    }).detach();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyNavigateToPoseServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
