#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "amiga_ros2_behavior_tree/mocks/failure_modes.hpp"
#include "kortex_interfaces/action/segment_leaves.hpp"

using SegmentLeaves = kortex_interfaces::action::SegmentLeaves;

class DummySegmentLeavesServer : public rclcpp::Node {
 public:
  explicit DummySegmentLeavesServer(const std::string &name) : Node(name) {
    server_ = rclcpp_action::create_server<SegmentLeaves>(
        this, "segment_leaves",
        std::bind(&DummySegmentLeavesServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&DummySegmentLeavesServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&DummySegmentLeavesServer::handle_accepted, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "Dummy SegmentLeaves action server started on 'segment_leaves'");
  }

 private:
  rclcpp_action::Server<SegmentLeaves>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const SegmentLeaves::Goal> goal) {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received SegmentLeaves goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SegmentLeaves>>) {
    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SegmentLeaves>>
          goal_handle) {
    std::thread([this, goal_handle]() {
      auto feedback = std::make_shared<SegmentLeaves::Feedback>();
      auto result = std::make_shared<SegmentLeaves::Result>();

      RCLCPP_INFO(this->get_logger(), "Executing SegmentLeaves action...");

      // Simulate leaf segmentation with feedback
      std::vector<std::string> states = {
          "initializing", "capturing_image", "processing", "segmenting", "complete"};

      for (const auto &state : states) {
        feedback->current_state = state;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "State: %s", state.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      // The failure path, after the feedback states, because the real node
      // fails on the *result* of processing a point cloud rather than on
      // being asked. A goal that failed before any feedback would be
      // distinguishable from a real one by its shape alone.
      if (policy_ &&
          policy_->shouldFail(amiga_bt::mocks::kUnkeyedGoal)) {
        const auto mode = policy_->mode();
        if (amiga_bt::mocks::logsAsWarning(mode)) {
          RCLCPP_WARN(this->get_logger(), "%s",
                      amiga_bt::mocks::failureLog(mode));
        } else {
          RCLCPP_ERROR(this->get_logger(), "%s",
                       amiga_bt::mocks::failureLog(mode));
        }
        result->success = false;
        result->message = amiga_bt::mocks::failureResultMessage(mode);
        goal_handle->abort(result);
        return;
      }

      result->success = true;
      result->message = "Successfully segmented 5 leaves";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "SegmentLeaves action succeeded!");
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
  auto node = std::make_shared<DummySegmentLeavesServer>(
      amiga_bt::mocks::nodeName(argc, argv, "dummy_segment_leaves_server"));
  node->configureFailures();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
