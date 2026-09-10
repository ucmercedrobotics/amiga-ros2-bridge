// Making a mock action server fail the way the real one does.
//
// The chain below a behaviour-tree fault ends in a language model reading
// /rosout. That constrains how a test may inject a failure far more than it
// first appears: a mock that logs "simulating navigation failure" hands the
// model the answer, and every interpretation downstream is then a measurement
// of the test harness rather than of the system.
//
// So the rules here are:
//
//   * The words on the failing path are copied verbatim from the real node --
//     amiga-ros2-nav/amiga_navigation/scripts/waypoint_follower.py -- and each
//     is followed by the same goal abort the real node performs.
//   * No "inject", "mock", "simulate", "fake" or "test" appears anywhere a
//     failing goal can put it.
//   * Which goals fail is a launch *parameter*, never a topic. A
//     /mock/inject_fault message would land in the same 30-second /rosout
//     window the triage agent reads, which is the whole problem again.
//   * The node's own name is a parameter too, so the "[node]" prefix on those
//     lines matches the real one.
//
// What this can and cannot do is worth being plain about. It reproduces the
// *shape* of a failure: the result code, the log text, the timing, the
// behaviour-tree transition. It does not reproduce the substance -- there is no
// blocked row, no lost GPS fix, no empty tank behind it. That makes it sound
// evidence for testing the *wiring* (does a fault reach an auction and come
// back as a plan edit?) and no evidence at all for the quality of the model's
// reasoning, which can only be measured in Gazebo or on hardware. See
// test/scenarios/README.md.

#ifndef AMIGA_ROS2_BEHAVIOR_TREE_MOCKS_FAILURE_MODES_HPP
#define AMIGA_ROS2_BEHAVIOR_TREE_MOCKS_FAILURE_MODES_HPP

#include <algorithm>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace amiga_bt {
namespace mocks {

// Each of these is a real failure path in waypoint_follower.py, named after the
// condition that triggers it there.
enum class FailureMode {
  // "Orchard GetTreeInfo service unavailable" -- the orchard model is down.
  kTreeInfoUnavailable,
  // "GetTreeInfo returned empty result" -- the orchard has no such tree. This
  // is the permanent one: no robot can reach a tree that does not exist, so a
  // correct interpretation drops the task rather than offering it around.
  kTreeInfoEmpty,
  // "No valid target waypoint available from orchard data" -- the tree is
  // known but has no row waypoint to approach it from.
  kNoWaypoint,
  // "NavigateViaLidar action failed" -- the row is blocked, or localisation
  // gave out. Transient in principle, which is what makes it worth offering to
  // a peer rather than dropping.
  kNavFailed,

  // The arm's two, from kortex_vision/pistachio_leaf_segmentation.py.
  //
  // "No point cloud available." -- the depth camera has produced nothing. A
  // fault in this robot, so another robot with a working camera is exactly the
  // right answer.
  kNoPointCloud,
  // "No leaves detected in the point cloud." -- the camera worked and there
  // was nothing to sample. Nobody else would find leaves there either, which
  // makes this the arm's permanent failure.
  kNoLeaves,
  // "Leaf segmentation returned no masks." -- the detector ran, saw, and found
  // nothing it recognised. The third answer, and the only one the other two
  // cannot express: the sensor is fine and the branch is not bare, but the
  // conditions defeated the model -- low sun straight into the lens, the
  // canopy backlit to silhouette. A different position or a later hour finds
  // the leaves, so this belongs to the plan, not to the fleet and not to the
  // scrap heap.
  //
  // The message deliberately reports the outcome and not a cause. The real
  // node's own wording names the point cloud, and the reasoning model reads
  // that as the depth sensor every time -- which is a diagnosis the code has
  // not earned, since zero masks arise from an empty point cloud and from a
  // blinded detector alike.
  kNoMasks,
};

inline FailureMode failureModeFromString(const std::string &name) {
  static const std::map<std::string, FailureMode> kByName = {
      {"tree_info_unavailable", FailureMode::kTreeInfoUnavailable},
      {"tree_info_empty", FailureMode::kTreeInfoEmpty},
      {"no_waypoint", FailureMode::kNoWaypoint},
      {"nav_failed", FailureMode::kNavFailed},
      {"no_point_cloud", FailureMode::kNoPointCloud},
      {"no_leaves", FailureMode::kNoLeaves},
      {"no_masks", FailureMode::kNoMasks},
  };
  auto it = kByName.find(name);
  // Defaults to the transient one. A test that misspells the mode gets a
  // plausible failure rather than a silently succeeding server, which is the
  // direction that fails loudly.
  return it == kByName.end() ? FailureMode::kNavFailed : it->second;
}

// The line the real node logs before aborting. Verbatim, deliberately.
inline const char *failureLog(FailureMode mode) {
  switch (mode) {
    case FailureMode::kTreeInfoUnavailable:
      return "Orchard GetTreeInfo service unavailable";
    case FailureMode::kTreeInfoEmpty:
      return "GetTreeInfo returned empty result";
    case FailureMode::kNoWaypoint:
      return "No valid target waypoint available from orchard data";
    case FailureMode::kNoPointCloud:
      return "No point cloud available.";
    case FailureMode::kNoLeaves:
      return "No leaves detected in the point cloud.";
    case FailureMode::kNoMasks:
      return "Leaf segmentation returned no masks.";
    case FailureMode::kNavFailed:
    default:
      return "NavigateViaLidar action failed";
  }
}

// The real node logs its two arm failures at WARN and its navigation ones at
// ERROR. The level is part of the evidence too -- the triage agent's log slice
// keeps severe lines when it has to truncate, so getting this wrong changes
// what a model sees under load.
inline bool logsAsWarning(FailureMode mode) {
  return mode == FailureMode::kNoPointCloud || mode == FailureMode::kNoLeaves ||
         mode == FailureMode::kNoMasks;
}

// What the real node puts in Result.message for a failure. Empty where the
// real one leaves it empty.
inline const char *failureResultMessage(FailureMode mode) {
  switch (mode) {
    case FailureMode::kNoPointCloud:
      return "No point cloud received yet.";
    case FailureMode::kNoLeaves:
      return "No leaves detected in the point cloud.";
    case FailureMode::kNoMasks:
      return "Leaf segmentation returned no masks.";
    default:
      return "";
  }
}

//: The key a server with no per-goal identity uses. SegmentLeaves takes no
//: arguments -- the arm samples whatever it is in front of -- so "fail this
//: goal" cannot be keyed on anything, and the scenario says `fail_goals: [0]`
//: to mean "this server's goals".
constexpr int64_t kUnkeyedGoal = 0;

// Decides which goals fail. Constructed from parameters, consulted per goal.
class FailurePolicy {
 public:
  // Declares the parameters on `node` and reads them once. They are read once
  // rather than watched, so nothing about the failure can change mid-mission
  // in response to what the robot did -- which would be a feedback loop the
  // scenario did not describe.
  explicit FailurePolicy(const rclcpp::Node::SharedPtr &node) {
    // Spelled out rather than braced: a bare {} is ambiguous between the empty
    // default and a default-constructed ParameterDescriptor, and the overload
    // it picks is not the one this means.
    node->declare_parameter<std::vector<int64_t>>(
        "fail_goals", std::vector<int64_t>{});
    node->declare_parameter<int>("fail_after_n", 0);
    node->declare_parameter<std::string>("failure_mode", "nav_failed");

    fail_goals_ = node->get_parameter("fail_goals").as_integer_array();
    fail_after_n_ = node->get_parameter("fail_after_n").as_int();
    mode_ = failureModeFromString(
        node->get_parameter("failure_mode").as_string());
  }

  // Whether the goal identified by `goal_id` should fail this time.
  //
  // `fail_after_n` counts attempts *at that goal*, not globally, so a
  // RetryUntilSuccessful wrapper behaves the way a scenario means it to: "fail
  // twice, then succeed" is a transient fault the tree can recover from, and
  // that has to be per-goal or the retries of one tree would exhaust the budget
  // of the next.
  bool shouldFail(int64_t goal_id) {
    if (std::find(fail_goals_.begin(), fail_goals_.end(), goal_id) ==
        fail_goals_.end()) {
      return false;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    const int seen = ++attempts_[goal_id];
    return seen > fail_after_n_;
  }

  FailureMode mode() const { return mode_; }

  bool enabled() const { return !fail_goals_.empty(); }

 private:
  std::vector<int64_t> fail_goals_;
  int fail_after_n_ = 0;
  FailureMode mode_ = FailureMode::kNavFailed;
  std::mutex mtx_;
  std::map<int64_t, int> attempts_;
};

// The node name a mock runs under.
//
// A parameter because the "[node]" prefix is part of the evidence: the triage
// agent's log window shows who logged each line, and a failure attributed to
// "dummy_tree_id_action_server" is a failure that announces what it is.
inline std::string nodeName(int argc, char **argv, const std::string &fallback) {
  const std::string flag = "--mock-node-name";
  for (int i = 1; i + 1 < argc; ++i) {
    if (flag == argv[i]) {
      return argv[i + 1];
    }
  }
  return fallback;
}

}  // namespace mocks
}  // namespace amiga_bt

#endif  // AMIGA_ROS2_BEHAVIOR_TREE_MOCKS_FAILURE_MODES_HPP
