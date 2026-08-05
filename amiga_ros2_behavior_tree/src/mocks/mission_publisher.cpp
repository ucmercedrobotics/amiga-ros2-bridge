// Publishes one mission, once, after a delay. For scenarios only.
//
// /mission/xml has exactly one writer in production -- the arbiter -- and that
// is load-bearing: it is what makes "the plan currently running" a thing with
// a single source. This node exists so a scenario can start a mission without
// a model endpoint, and it is deliberately the dumbest possible way to do it:
// one string from a parameter, one publish, then nothing.
//
// The delay is not cosmetic. bt_runner subscribes on a plain (non-latched)
// topic, so a mission published before it is up is a mission nobody receives,
// and the scenario would sit there looking like the tree had rejected it.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MissionPublisher : public rclcpp::Node {
 public:
  MissionPublisher() : Node("mission_publisher") {
    this->declare_parameter<std::string>("mission_xml", "");
    this->declare_parameter<std::string>("mission_topic", "/mission/xml");
    this->declare_parameter<double>("delay_sec", 3.0);

    const auto xml = this->get_parameter("mission_xml").as_string();
    const auto topic = this->get_parameter("mission_topic").as_string();
    const auto delay = this->get_parameter("delay_sec").as_double();

    if (xml.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "mission_xml is empty; nothing to publish");
      return;
    }

    pub_ = this->create_publisher<std_msgs::msg::String>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(delay * 1000)), [this, xml,
                                                                   topic]() {
          timer_->cancel();
          std_msgs::msg::String msg;
          msg.data = xml;
          pub_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "published %zu bytes to %s",
                      xml.size(), topic.c_str());
        });
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPublisher>());
  rclcpp::shutdown();
  return 0;
}
