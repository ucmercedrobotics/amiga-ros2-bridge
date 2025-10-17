#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class DummyNavigateToPoseServer : public rclcpp::Node {
 public:
  DummyNavigateToPoseServer() : Node("dummy_navigate_to_pose_server") {
    server_ = rclcpp_action::create_server<NavigateToPose>(
        this, "navigate_to_pose",
        std::bind(&DummyNavigateToPoseServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&DummyNavigateToPoseServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&DummyNavigateToPoseServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Dummy NavigateToPose action server ready");
  }

 private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;

  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const NavigateToPose::Goal> goal) {
    std::string uuid_str = "Received goal request with UUID: ";
    for (auto const &id : uuid) uuid_str += std::to_string(id) + " ";

    RCLCPP_DEBUG(this->get_logger(), "%s", uuid_str.c_str());

    double yaw = getYawFromQuaternion(goal->pose.pose.orientation);

    RCLCPP_INFO(this->get_logger(),
                "Received navigation goal in frame '%s' to pose: (%.2f, %.2f) "
                "with yaw: %.2f rad (%.1f deg)",
                goal->pose.header.frame_id.c_str(), goal->pose.pose.position.x,
                goal->pose.pose.position.y, yaw, yaw * 180.0 / M_PI);
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

      double target_yaw = getYawFromQuaternion(goal->pose.pose.orientation);
      double target_x = goal->pose.pose.position.x;
      double target_y = goal->pose.pose.position.y;

      RCLCPP_INFO(this->get_logger(), "Executing navigation goal...");
      RCLCPP_INFO(
          this->get_logger(),
          "Target: Frame='%s', Position=(%.2f, %.2f, %.2f), Yaw=%.2f rad",
          goal->pose.header.frame_id.c_str(), target_x, target_y,
          goal->pose.pose.position.z, target_yaw);

      const int steps = 10;
      for (int i = 1; i <= steps; ++i) {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal was canceled");
          return;
        }

        double progress = static_cast<double>(i) / steps;

        feedback->current_pose.pose.position.x = target_x * progress;
        feedback->current_pose.pose.position.y = target_y * progress;
        feedback->current_pose.pose.position.z = 0.0;

        double current_yaw = target_yaw * progress;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_yaw);
        feedback->current_pose.pose.orientation.x = q.x();
        feedback->current_pose.pose.orientation.y = q.y();
        feedback->current_pose.pose.orientation.z = q.z();
        feedback->current_pose.pose.orientation.w = q.w();

        double remaining_x = target_x - feedback->current_pose.pose.position.x;
        double remaining_y = target_y - feedback->current_pose.pose.position.y;
        feedback->distance_remaining =
            std::sqrt(remaining_x * remaining_x + remaining_y * remaining_y);

        double remaining_yaw = std::abs(target_yaw - current_yaw);

        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(),
                    "Progress: %.1f%%, Pos: (%.2f, %.2f), Yaw: %.2f rad, Dist "
                    "remaining: %.2f m, Angle remaining: %.2f rad",
                    progress * 100.0, feedback->current_pose.pose.position.x,
                    feedback->current_pose.pose.position.y, current_yaw,
                    feedback->distance_remaining, remaining_yaw);

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
