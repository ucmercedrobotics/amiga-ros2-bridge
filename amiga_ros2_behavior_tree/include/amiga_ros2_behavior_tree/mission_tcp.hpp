#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#define MISSION_TCP_DEFAULT_PORT 12346
#define MISSION_TCP_DEFAULT_PAYLOAD_LENGTH 4096

namespace mission_tcp {
std::string wait_for_mission_tcp(int port, const rclcpp::Logger &logger,
                                 bool payload_length_included);
}
