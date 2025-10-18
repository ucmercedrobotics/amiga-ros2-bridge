#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amiga_interfaces/action/navigate_to_pose_in_frame.hpp"

using NavigateToPoseInFrame = amiga_interfaces::action::NavigateToPoseInFrame;

class DummyNavigateToPoseInFrameServer : public rclcpp::Node {
 public:
  DummyNavigateToPoseInFrameServer()
      : Node("dummy_navigate_to_pose_in_frame_server") {
    server_ = rclcpp_action::create_server<NavigateToPoseInFrame>(
        this, "navigate_to_pose_in_frame",
        std::bind(&DummyNavigateToPoseInFrameServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&DummyNavigateToPoseInFrameServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&DummyNavigateToPoseInFrameServer::handle_accepted, this,
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
                "Received navigation goal in to pose: (%.2f, %.2f) "
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
                  "Target: Absolute='%i', Position=(%.2f, %.2f), Yaw=%.2f rad",
                  goal->absolute, target_x, target_y, target_yaw);

      const int steps = 10;
      for (int i = 1; i <= steps; ++i) {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal was canceled");
          return;
        }

        double progress = static_cast<double>(i) / steps;

        double current_yaw = target_yaw * progress;

        feedback->distance_remaining =
            std::sqrt(target_x * target_x + target_y * target_y) *
            (1.0 - progress);

        double yaw_remaining = std::abs(target_yaw - current_yaw);
        feedback->yaw_remaining = yaw_remaining;

        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(
            this->get_logger(),
            "Progress: %.1f%%, Dist remaining: %.2f m, Yaw remaining: %.2f rad",
            progress * 100.0, feedback->distance_remaining,
            feedback->yaw_remaining);

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
  auto node = std::make_shared<DummyNavigateToPoseInFrameServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
