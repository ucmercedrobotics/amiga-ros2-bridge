#include "amiga_ros2_behavior_tree/fault_reporter.hpp"

#include <nlohmann/json.hpp>

namespace amiga_bt {

using json = nlohmann::json;

namespace {

// By value, not a const char*: BT::toStr returns a temporary, and handing back
// a pointer into it is a dangling read the moment the call ends.
std::string statusName(BT::NodeStatus status) {
  return BT::toStr(status, false);
}

} // namespace

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
makeFaultPublisher(const rclcpp::Node::SharedPtr &node,
                   const std::string &topic) {
  // TRANSIENT_LOCAL so an agent that starts after the fault still sees it. The
  // pipeline downstream is a chain of LLM-backed nodes that take seconds to
  // come up, and the fault is often what brings them up at all; publishing it
  // into a topic with no subscribers yet would strand the mission with nothing
  // to explain why.
  rclcpp::QoS qos(10);
  qos.transient_local();
  return node->create_publisher<std_msgs::msg::String>(topic, qos);
}

FaultReporter::FaultReporter(
    BT::Tree &tree, rclcpp::Node::SharedPtr node,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
    double min_interval_sec)
    : StatusChangeLogger(tree.rootNode()), node_(std::move(node)),
      pub_(std::move(pub)), min_interval_sec_(min_interval_sec) {
  // Transitions to IDLE are how BT.CPP reports a node being reset between
  // ticks. They are bookkeeping, not outcomes, and they arrive constantly.
  enableTransitionToIdle(false);
}

void FaultReporter::callback(BT::Duration /*timestamp*/,
                             const BT::TreeNode &node,
                             BT::NodeStatus prev_status,
                             BT::NodeStatus status) {
  // Leaves only. A control node reporting FAILURE is reporting its child's.
  const BT::NodeType type = node.type();
  const bool is_leaf =
      type == BT::NodeType::ACTION || type == BT::NodeType::CONDITION;
  const bool is_failure = is_leaf && status == BT::NodeStatus::FAILURE;
  // SUCCESS only from actions: a condition succeeds on nearly every tick and
  // would flood this topic for no consumer that needs it.
  const bool is_success =
      type == BT::NodeType::ACTION && status == BT::NodeStatus::SUCCESS;
  if (!is_failure && !is_success) {
    return;
  }

  const double now = node_->now().seconds();
  const uint16_t uid = node.UID();
  auto &throttle_map = is_failure ? last_failure_reported_ : last_success_reported_;
  auto it = throttle_map.find(uid);
  if (it != throttle_map.end() && (now - it->second) < min_interval_sec_) {
    return;
  }
  throttle_map[uid] = now;

  json event;
  // These three keys are the contract MissionPlannerNode and ArbiterNode
  // already read. Everything below them is additive.
  event["node"] = node.name();
  event["reason"] = "";
  event["timestamp_ms"] = static_cast<int64_t>(now * 1000.0);

  event["node_type"] = BT::toStr(type);
  event["registration_name"] = node.registrationName();
  event["path"] = node.config().path;
  event["uid"] = uid;
  event["prev_status"] = statusName(prev_status);
  event["status"] = statusName(status);
  event["source"] = "leaf";

  if (is_failure) {
    RCLCPP_WARN(node_->get_logger(), "BT fault: %s (%s) failed",
                node.name().c_str(), node.registrationName().c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "BT leaf succeeded: %s (%s)",
                node.name().c_str(), node.registrationName().c_str());
  }
  publish(event.dump());
}

void FaultReporter::reportTreeOutcome(BT::NodeStatus status) {
  const double now = node_->now().seconds();

  json event;
  event["node"] = "<tree>";
  event["reason"] = status == BT::NodeStatus::FAILURE
                        ? "the behaviour tree returned FAILURE"
                        : "the behaviour tree returned SUCCESS";
  event["timestamp_ms"] = static_cast<int64_t>(now * 1000.0);

  event["node_type"] = "TREE";
  event["status"] = statusName(status);
  event["source"] = "tree";

  publish(event.dump());
}

void FaultReporter::publish(const std::string &payload) {
  std_msgs::msg::String msg;
  msg.data = payload;
  pub_->publish(msg);
}

} // namespace amiga_bt
