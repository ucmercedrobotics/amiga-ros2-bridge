// A stand-in for the fruit harvester, for a sim with no fruit on the trees.
//
// Like sim_segment_leaves_server, it drives the arm through the motion the
// real thing takes rather than only reporting on one: reach along the
// gripper's approach axis, close on the fruit, hold, come home, open. It
// commands the same `move_to` / `gripper_control` servers a real harvesting
// pipeline would, so a mission sees the same class of arm motion in sim as on
// hardware -- just with no fruit under the gripper.
//
// It reports what a harvester reports: whether it got anything, and a sentence
// saying so. Whether it got anything depends on the tree it is standing at,
// because that is what it depends on in an orchard -- some trees have fruit on
// them and some do not, and the ones that do not are named in the world rather
// than chosen here. See `barren_trees` below.
//
// **Why this reports a failure and not an empty success.** Nothing in this
// system reasons until something fails: triage is woken by a FAILURE on
// /bt/status_change, and a harvest that returned "success, zero fruit" would
// pass through the whole stack unread. A robot that picked nothing has not
// done the work the mission asked for, so saying so is also just accurate.
//
// **What it deliberately does not say.** Only that it picked nothing -- never
// why. "No fruit harvested" is as true of a bare tree as of fruit the arm
// could not reach, and those want opposite answers: an empty tree is empty for
// every robot, so the task should be dropped, while fruit that this robot
// could not reach is worth offering to one that can. Separating them is the
// camera's job, and a server that announced the reason would be handing the
// agents a conclusion they are supposed to draw. The same rule the rest of
// mocks/ follows: see failure_modes.hpp.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "kortex_interfaces/action/gripper_control.hpp"
#include "kortex_interfaces/action/move_to.hpp"
#include "kortex_interfaces/action/segment_leaves.hpp"

using json = nlohmann::json;
using SegmentLeaves = kortex_interfaces::action::SegmentLeaves;
using MoveTo = kortex_interfaces::action::MoveTo;
using GripperControl = kortex_interfaces::action::GripperControl;
using GoalHandle = rclcpp_action::ServerGoalHandle<SegmentLeaves>;

namespace {
// Same joint order and pose as sim_segment_leaves_server, which is the same
// one the Makefile's `kortex-home` target and sim_arm.launch.py's post-bringup
// publish use -- the one place this arm's "home" is defined.
const std::array<std::string, 6> kJointNames = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
const std::array<double, 6> kHomePositions = {0.0, -0.785398, -2.0,
                                              0.0, -0.436332, 1.5708};
// Metres per degree of latitude. Good to a few parts in a thousand over one
// orchard, which is three orders finer than the four-metre tree spacing it is
// used to resolve.
constexpr double kMetresPerDegree = 111320.0;
}  // namespace

class SimHarvestFruitServer : public rclcpp::Node {
 public:
  SimHarvestFruitServer() : Node("harvest_fruit_sim") {
    pick_time_sec_ = declare_parameter<double>("pick_time_sec", 4.0);
    approach_distance_ = declare_parameter<double>("approach_distance", 0.08);
    gripper_close_position_ = declare_parameter<double>(
        "gripper_close_position", GripperControl::Goal::CLOSE);
    gripper_open_position_ = declare_parameter<double>(
        "gripper_open_position", GripperControl::Goal::OPEN);
    home_time_sec_ = declare_parameter<double>("home_time_sec", 3.0);
    action_server_timeout_sec_ =
        declare_parameter<double>("action_server_timeout_sec", 10.0);

    move_to_client_ = rclcpp_action::create_client<MoveTo>(this, "move_to");
    gripper_client_ =
        rclcpp_action::create_client<GripperControl>(this, "gripper_control");
    home_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory_controller/joint_trajectory", 1);

    // Trees the world has put no fruit on. A parameter rather than a constant
    // because it is a property of the world file, and the launch that resolves
    // that file is the only thing in a position to know it.
    const auto barren = declare_parameter<std::vector<int64_t>>(
        "barren_trees", std::vector<int64_t>{});
    barren_.assign(barren.begin(), barren.end());

    // Plain volatile, matching tcp_demux_node's publisher and the agents'
    // subscriptions. Asking for transient_local here is an incompatible QoS
    // against a volatile publisher, so nothing arrives at all -- which looks
    // exactly like an orchard with no barren trees in it.
    orchard_sub_ = create_subscription<std_msgs::msg::String>(
        "orchard/tree_info_json", 10,
        [this](const std_msgs::msg::String &msg) { load_orchard(msg.data); });
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/filtered", 10, [this](const sensor_msgs::msg::NavSatFix &msg) {
          have_fix_ = true;
          lat_ = msg.latitude;
          lon_ = msg.longitude;
        });
    // The tree the running plan says it is working on -- see tree_here().
    // Transient-local to match MoveToTreeID's publisher, and because the
    // current target is a fact this server needs on startup rather than at the
    // next move.
    target_sub_ = create_subscription<std_msgs::msg::UInt32>(
        "tree/target", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::UInt32 &msg) { target_tree_ = msg.data; });
    // Which way the robot is pointing, the fallback for telling the two rows
    // flanking a lane apart when no plan has named a tree. The global solution
    // rather than the local one because its yaw is ENU, the same frame the
    // tree bearings are computed in.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered/global", 10,
        [this](const nav_msgs::msg::Odometry &msg) {
          const auto &q = msg.pose.pose.orientation;
          have_heading_ = true;
          yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        });

    server_ = rclcpp_action::create_server<SegmentLeaves>(
        this, "harvest_fruit",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const SegmentLeaves::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> gh) {
          std::thread{[this, gh]() { execute(gh); }}.detach();
        });

    RCLCPP_INFO(get_logger(),
                "Fruit harvester ready on 'harvest_fruit': reaches %.2fm, "
                "closes the gripper to %.2f, holds for %.1fs, returns home. "
                "%zu tree(s) in this block carry no fruit",
                approach_distance_, gripper_close_position_, pick_time_sec_,
                barren_.size());
  }

 private:
  void load_orchard(const std::string &payload) {
    json doc;
    try {
      doc = json::parse(payload);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Could not read the orchard: %s", e.what());
      return;
    }
    if (!doc.contains("trees") || !doc["trees"].is_array()) return;
    trees_.clear();
    for (const auto &t : doc["trees"]) {
      if (!t.contains("tree_index") || !t.contains("lat") || !t.contains("lon")) continue;
      trees_.push_back({t["tree_index"].get<int>(), t["lat"].get<double>(),
                        t["lon"].get<double>()});
    }
    RCLCPP_INFO(get_logger(), "Orchard received — %zu trees placed", trees_.size());
  }

  /// The tree the robot is working on, or 0 if that cannot be said.
  ///
  /// Position cannot answer this. A row waypoint sits on the lane's
  /// centre-line and a lane runs *between* two rows, so the tree the mission
  /// asked for and the tree directly opposite it are equidistant -- to the
  /// centimetre, on every approach, not just for a robot that happens to be
  /// badly parked. Measured at tree 26's own waypoint both candidates come out
  /// 4.51 m, and picking the smaller of those is picking GPS noise.
  ///
  /// So the plan is asked instead: MoveToTreeID announces the tree it is
  /// navigating to, which is the only account of the target that survives the
  /// robot turning round. Heading was tried first and is not enough on its own
  /// -- it reads correctly on the approach, then a repair backs the robot out
  /// and re-enters, and a retry at the same spot finds the far row in front of
  /// it and harvests a tree the mission never asked about.
  ///
  /// Heading is kept as the fallback for a robot that reached its tree some
  /// other way, and plain nearest behind that. Distances are metres: a degree
  /// of longitude is cos(lat) of a degree of latitude, so comparing raw degrees
  /// stretches the orchard by 1.26 east-west here.
  int tree_here() const {
    if (target_tree_ != 0) return target_tree_;
    if (!have_fix_ || trees_.empty()) return 0;

    const double lat_scale = kMetresPerDegree;
    const double lon_scale = kMetresPerDegree * std::cos(lat_ * M_PI / 180.0);

    int best = 0, best_ahead = 0;
    double best_d2 = std::numeric_limits<double>::max();
    double best_ahead_d2 = std::numeric_limits<double>::max();

    for (const auto &t : trees_) {
      // ENU: +x east, +y north, to match the yaw taken from odometry.
      const double east = (t.lon - lon_) * lon_scale;
      const double north = (t.lat - lat_) * lat_scale;
      const double d2 = east * east + north * north;

      if (d2 < best_d2) { best_d2 = d2; best = t.index; }

      // In front of the robot: the tree's bearing is within 90 degrees of the
      // heading. Written as a dot product so the wrap-around at +/-180 needs
      // no special case.
      if (have_heading_ &&
          east * std::cos(yaw_) + north * std::sin(yaw_) > 0.0 &&
          d2 < best_ahead_d2) {
        best_ahead_d2 = d2;
        best_ahead = t.index;
      }
    }

    return best_ahead != 0 ? best_ahead : best;
  }

  void publish_state(const std::shared_ptr<GoalHandle> &gh, const std::string &state) {
    auto feedback = std::make_shared<SegmentLeaves::Feedback>();
    feedback->current_state = state;
    gh->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "State: %s", state.c_str());
  }

  void execute(const std::shared_ptr<GoalHandle> gh) {
    auto result = std::make_shared<SegmentLeaves::Result>();
    const int tree = tree_here();

    // Named in the log because it is the one input that decides the outcome,
    // and a harvest that reports nothing about where it was standing cannot be
    // told apart from one that never knew.
    if (tree == 0) {
      RCLCPP_WARN(get_logger(),
                  "Cannot tell which tree this is (orchard: %zu trees, fix: %s)",
                  trees_.size(), have_fix_ ? "yes" : "no");
    } else {
      RCLCPP_INFO(get_logger(), "Harvesting at tree %d", tree);
    }

    std::string error;

    publish_state(gh, "reaching_for_fruit");
    if (!move_relative(0.0, 0.0, approach_distance_, error)) {
      fail(gh, result, "Failed to reach for fruit: " + error);
      return;
    }

    // Checked with the arm already out, because that is when a harvester finds
    // out. Reaching into a bare canopy and withdrawing empty is the motion this
    // failure actually looks like, and it is what the camera sees when triage
    // asks what happened.
    if (std::find(barren_.begin(), barren_.end(), tree) != barren_.end()) {
      publish_state(gh, "returning_home");
      go_home();  // before the abort, or the arm is left hanging in the tree
      result->success = false;
      result->message = "No fruit harvested";
      RCLCPP_ERROR(get_logger(), "No fruit harvested");
      gh->abort(result);
      return;
    }

    publish_state(gh, "picking_fruit");
    if (!set_gripper(gripper_close_position_, error)) {
      fail(gh, result, "Failed to close gripper on fruit: " + error);
      return;
    }

    publish_state(gh, "holding_fruit");
    std::this_thread::sleep_for(std::chrono::duration<double>(pick_time_sec_));

    publish_state(gh, "returning_home");
    go_home();

    // Opened at home rather than at the tree: the fruit is being put down
    // where it is collected, which is also what leaves the gripper ready for
    // the next tree.
    publish_state(gh, "stowing_fruit");
    if (!set_gripper(gripper_open_position_, error)) {
      fail(gh, result, "Failed to open gripper: " + error);
      return;
    }

    publish_state(gh, "complete");
    result->success = true;
    result->message = "Fruit harvested";
    RCLCPP_INFO(get_logger(), "Fruit harvested");
    gh->succeed(result);
  }

  void fail(const std::shared_ptr<GoalHandle> &gh,
            const std::shared_ptr<SegmentLeaves::Result> &result,
            const std::string &message) {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    result->success = false;
    result->message = message;
    gh->abort(result);
  }

  // Relative move in the end-effector's own frame; +z is its approach axis.
  // Same shape as sim_segment_leaves_server's, and talking to the same
  // `move_to` server, so a harvest moves the arm the way a sample does.
  bool move_relative(double dx, double dy, double dz, std::string &error) {
    if (!move_to_client_->wait_for_action_server(
            std::chrono::duration<double>(action_server_timeout_sec_))) {
      error =
          "'move_to' action server not available -- is sim_arm.launch.py "
          "(launch_moveto:=true) running for this robot?";
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
          if (!handle) done->set_value({false, "move_to goal was rejected"});
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
          if (!handle) done->set_value({false, "gripper_control goal was rejected"});
        };

    gripper_client_->async_send_goal(goal, options);
    auto outcome = done_future.get();
    error = outcome.second;
    return outcome.first;
  }

  // Raw joint-space home, deterministic and needing no MoveIt plan -- the same
  // route sim_segment_leaves_server takes back.
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

  struct Tree { int index; double lat; double lon; };

  double pick_time_sec_{4.0};
  double approach_distance_{0.08};
  double gripper_close_position_{0.0};
  double gripper_open_position_{0.0};
  double home_time_sec_{3.0};
  double action_server_timeout_sec_{10.0};

  std::vector<int> barren_;
  std::vector<Tree> trees_;
  bool have_fix_{false};
  double lat_{0.0}, lon_{0.0};
  bool have_heading_{false};
  double yaw_{0.0};
  // 0 until a plan names one; see tree_here().
  int target_tree_{0};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr orchard_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr target_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr home_pub_;
  rclcpp_action::Client<MoveTo>::SharedPtr move_to_client_;
  rclcpp_action::Client<GripperControl>::SharedPtr gripper_client_;
  rclcpp_action::Server<SegmentLeaves>::SharedPtr server_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimHarvestFruitServer>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
