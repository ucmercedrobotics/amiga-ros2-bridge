#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "kortex_interfaces/action/gripper_control.hpp"
#include "kortex_interfaces/action/move_to.hpp"
#include "kortex_interfaces/action/segment_leaves.hpp"

using SegmentLeaves = kortex_interfaces::action::SegmentLeaves;
using MoveTo = kortex_interfaces::action::MoveTo;
using GripperControl = kortex_interfaces::action::GripperControl;
using GoalHandleSegmentLeaves = rclcpp_action::ServerGoalHandle<SegmentLeaves>;

namespace {
// Same joint order and pose as the Makefile's `kortex-home` target and
// sim_arm.launch.py's post-bringup home publish -- the one place this arm's
// "home" is actually defined.
const std::array<std::string, 6> kJointNames = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
const std::array<double, 6> kHomePositions = {0.0,      -0.785398, -2.0,
                                              0.0,      -0.436332, 1.5708};
}  // namespace

// A stand-in for pistachio_leaf_segmentation.py's `segment_leaves` action,
// for use where there is no depth camera, YOLO model or real leaf to find --
// Gazebo sim, in particular. Rather than segmenting anything, it drives the
// arm through the motion a real sample takes: step forward along the
// gripper's own approach axis, close the gripper, hold for a beat, open the
// gripper, then return to the joint-space home position. It talks to the
// same `move_to` / `gripper_control` action servers (kortex_move's `moveto`
// node, already sim-compatible) that a real sampling pipeline ends up
// commanding, so a mission sees the same class of arm motion in sim as on
// hardware -- just with no leaf underneath it.
class SimSegmentLeavesServer : public rclcpp::Node {
 public:
  SimSegmentLeavesServer() : Node("segment_leaves_sim") {
    approach_distance_ = declare_parameter<double>("approach_distance", 0.08);
    gripper_close_position_ = declare_parameter<double>(
        "gripper_close_position", GripperControl::Goal::CLOSE);
    gripper_open_position_ = declare_parameter<double>(
        "gripper_open_position", GripperControl::Goal::OPEN);
    hold_time_sec_ = declare_parameter<double>("hold_time_sec", 5.0);
    home_time_sec_ = declare_parameter<double>("home_time_sec", 3.0);
    action_server_timeout_sec_ =
        declare_parameter<double>("action_server_timeout_sec", 10.0);

    move_to_client_ = rclcpp_action::create_client<MoveTo>(this, "move_to");
    gripper_client_ =
        rclcpp_action::create_client<GripperControl>(this, "gripper_control");
    home_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 1);

    server_ = rclcpp_action::create_server<SegmentLeaves>(
        this, "segment_leaves",
        std::bind(&SimSegmentLeavesServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&SimSegmentLeavesServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&SimSegmentLeavesServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Simulated leaf sampler ready on 'segment_leaves': steps "
                "%.2fm forward, closes the gripper to %.2f, holds for %.1fs, "
                "opens the gripper to %.2f, returns home. No camera or model "
                "required.",
                approach_distance_, gripper_close_position_, hold_time_sec_,
                gripper_open_position_);
  }

 private:
  rclcpp_action::Server<SegmentLeaves>::SharedPtr server_;
  rclcpp_action::Client<MoveTo>::SharedPtr move_to_client_;
  rclcpp_action::Client<GripperControl>::SharedPtr gripper_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr home_pub_;

  double approach_distance_;
  double gripper_close_position_;
  double gripper_open_position_;
  double hold_time_sec_;
  double home_time_sec_;
  double action_server_timeout_sec_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const SegmentLeaves::Goal>) {
    RCLCPP_INFO(get_logger(), "Received SegmentLeaves goal (simulated)");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleSegmentLeaves>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSegmentLeaves> goal_handle) {
    std::thread(std::bind(&SimSegmentLeavesServer::execute, this, goal_handle))
        .detach();
  }

  void publish_state(const std::shared_ptr<GoalHandleSegmentLeaves> &goal_handle,
                      const std::string &state) {
    auto feedback = std::make_shared<SegmentLeaves::Feedback>();
    feedback->current_state = state;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "State: %s", state.c_str());
  }

  void fail(const std::shared_ptr<GoalHandleSegmentLeaves> &goal_handle,
            const std::shared_ptr<SegmentLeaves::Result> &result,
            const std::string &message) {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    result->success = false;
    result->message = message;
    goal_handle->abort(result);
  }

  void execute(const std::shared_ptr<GoalHandleSegmentLeaves> goal_handle) {
    auto result = std::make_shared<SegmentLeaves::Result>();
    std::string error;

    publish_state(goal_handle, "approaching_leaf");
    if (!move_relative(0.0, 0.0, approach_distance_, error)) {
      fail(goal_handle, result, "Failed to approach leaf: " + error);
      return;
    }

    publish_state(goal_handle, "grasping_leaf");
    if (!set_gripper(gripper_close_position_, error)) {
      fail(goal_handle, result, "Failed to close gripper on leaf: " + error);
      return;
    }

    publish_state(goal_handle, "holding_leaf");
    std::this_thread::sleep_for(std::chrono::duration<double>(hold_time_sec_));

    publish_state(goal_handle, "releasing_leaf");
    if (!set_gripper(gripper_open_position_, error)) {
      fail(goal_handle, result, "Failed to open gripper: " + error);
      return;
    }

    publish_state(goal_handle, "returning_home");
    go_home();

    publish_state(goal_handle, "complete");
    result->success = true;
    result->message = "Simulated leaf sample complete";
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "SegmentLeaves action succeeded (simulated)");
  }

  // Relative move in the end-effector's own frame; +z is its approach axis
  // (see kortex_description's end_effector_link joint -- its local +z
  // continues outward past the bracelet, away from the arm).
  bool move_relative(double dx, double dy, double dz, std::string &error) {
    if (!move_to_client_->wait_for_action_server(
            std::chrono::duration<double>(action_server_timeout_sec_))) {
      error =
          "'move_to' action server not available -- is sim_arm.launch.py "
          "(launch_arm:=true) running for this robot?";
      return false;
    }

    MoveTo::Goal goal;
    goal.movement_link = goal.END_EFFECTOR_LINK;
    goal.pose.position.x = dx;
    goal.pose.position.y = dy;
    goal.pose.position.z = dz;
    goal.pose.orientation.w = 1.0;  // identity: no rotation requested

    auto done = std::make_shared<std::promise<std::pair<bool, std::string>>>();
    auto done_future = done->get_future();

    rclcpp_action::Client<MoveTo>::SendGoalOptions options;
    options.result_callback =
        [done](const rclcpp_action::ClientGoalHandle<MoveTo>::WrappedResult &wrapped) {
          if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
              wrapped.result->success) {
            done->set_value({true, ""});
          } else {
            done->set_value({false, "move_to did not succeed"});
          }
        };
    options.goal_response_callback =
        [done](const rclcpp_action::ClientGoalHandle<MoveTo>::SharedPtr &handle) {
          if (!handle) {
            done->set_value({false, "move_to goal was rejected"});
          }
        };

    move_to_client_->async_send_goal(goal, options);
    auto outcome = done_future.get();
    error = outcome.second;
    return outcome.first;
  }

  bool set_gripper(double position, std::string &error) {
    if (!gripper_client_->wait_for_action_server(
            std::chrono::duration<double>(action_server_timeout_sec_))) {
      error = "'gripper_control' action server not available";
      return false;
    }

    GripperControl::Goal goal;
    goal.position = position;

    auto done = std::make_shared<std::promise<std::pair<bool, std::string>>>();
    auto done_future = done->get_future();

    rclcpp_action::Client<GripperControl>::SendGoalOptions options;
    options.result_callback =
        [done](const rclcpp_action::ClientGoalHandle<GripperControl>::WrappedResult
                   &wrapped) {
          if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
              wrapped.result->success) {
            done->set_value({true, ""});
          } else {
            done->set_value({false, "gripper_control did not succeed"});
          }
        };
    options.goal_response_callback =
        [done](const rclcpp_action::ClientGoalHandle<GripperControl>::SharedPtr
                   &handle) {
          if (!handle) {
            done->set_value({false, "gripper_control goal was rejected"});
          }
        };

    gripper_client_->async_send_goal(goal, options);
    auto outcome = done_future.get();
    error = outcome.second;
    return outcome.first;
  }

  // Raw joint-space home, the same way the Makefile's `kortex-home` target
  // and sim_arm.launch.py's own startup homing do -- deterministic, unlike
  // reversing the Cartesian approach step, and it needs no MoveIt plan.
  void go_home() {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names.assign(kJointNames.begin(), kJointNames.end());

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(kHomePositions.begin(), kHomePositions.end());
    point.time_from_start = rclcpp::Duration::from_seconds(home_time_sec_);
    traj.points.push_back(point);

    home_pub_->publish(traj);
    std::this_thread::sleep_for(
        std::chrono::duration<double>(home_time_sec_ + 0.5));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSegmentLeavesServer>());
  rclcpp::shutdown();
  return 0;
}
