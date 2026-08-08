// Publishes behaviour-tree faults so something can reason about them.
//
// The tree is the catalyst for the whole self-correction and coordination
// pipeline: nothing downstream -- the mission planner, the arbiter, the
// coordinator's triage step -- has anything to do until a node fails. Until
// this existed, that trigger was published by hand from a terminal, so every
// stage below it had only ever run against a mock.
//
// What is reported, and what is not:
//
//   * Transitions *to* FAILURE, on leaves (ACTION, CONDITION). A failing leaf
//     makes its ancestors fail too, and reporting each of them would fire the
//     planner once per level for a single fault.
//   * Transitions *to* SUCCESS, on ACTION leaves only -- not CONDITION, which
//     succeeds on nearly every tick and would flood this topic for no
//     consumer that needs it. This is the only place in the stack that ever
//     sees an individual leaf finish, so it is also the only source a
//     replanner has for "this objective is already done" -- see
//     MissionPlannerNode's completed-objectives ledger.
//   * One event per node per `min_interval_sec`, tracked separately for
//     FAILURE and SUCCESS: a ReactiveSequence re-ticks a failing condition at
//     the tick rate, so without throttling one stuck condition is a fault
//     storm -- but a leaf that fails and is retried into SUCCESS within the
//     same window must still report both, or the ledger never learns it
//     finished.
//   * Separately, the whole tree returning FAILURE or SUCCESS, which is the
//     mission-level fact and is not a leaf event at all.
//
// The payload deliberately does not try to say *why* the node failed. This
// callback is handed a status transition and nothing else -- the actual reason
// is in whatever the action node logged to /rosout on its way out. Publishing
// an invented reason here would be worse than publishing none, because the
// planner would prefer it to the log line that actually explains the fault.
// What this does provide is a timestamp the log window can be centred on,
// which is exactly how MissionPlannerNode already correlates the two.

#ifndef AMIGA_ROS2_BEHAVIOR_TREE_FAULT_REPORTER_HPP
#define AMIGA_ROS2_BEHAVIOR_TREE_FAULT_REPORTER_HPP

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace amiga_bt {

class FaultReporter : public BT::StatusChangeLogger {
public:
  // The publisher is created once, by makeFaultPublisher, and outlives every
  // tree. A reporter subscribes to one tree's nodes and must be rebuilt when
  // the tree is replaced -- but if it owned the publisher, destroying it would
  // take the latched fault with it, and the agent stack that is *starting up
  // because of that fault* would find an empty topic.
  FaultReporter(BT::Tree &tree, rclcpp::Node::SharedPtr node,
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
                double min_interval_sec);

  ~FaultReporter() override = default;

  // BT::StatusChangeLogger
  void callback(BT::Duration timestamp, const BT::TreeNode &node,
                BT::NodeStatus prev_status, BT::NodeStatus status) override;
  void flush() override {}

  // The tree finished. Reports FAILURE as a mission-level fault; SUCCESS is
  // published too, because "the mission completed" is what tells the
  // coordinator this robot is free to bid.
  void reportTreeOutcome(BT::NodeStatus status);

private:
  void publish(const std::string &payload);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  double min_interval_sec_;
  // UID -> seconds (node clock) of that node's last reported event, kept
  // separately per status so a FAILURE does not throttle the SUCCESS that
  // follows it (or vice versa) within the same window.
  std::unordered_map<uint16_t, double> last_failure_reported_;
  std::unordered_map<uint16_t, double> last_success_reported_;
};

// One publisher for the life of the process. TRANSIENT_LOCAL, so an agent that
// starts after the fault -- which is the normal case, since the fault is what
// brings the pipeline to life -- still receives it.
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
makeFaultPublisher(const rclcpp::Node::SharedPtr &node, const std::string &topic);

} // namespace amiga_bt

#endif // AMIGA_ROS2_BEHAVIOR_TREE_FAULT_REPORTER_HPP
