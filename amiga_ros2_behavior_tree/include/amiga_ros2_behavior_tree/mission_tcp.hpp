#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace mission_tcp
{
std::string wait_for_mission_tcp(int port, const rclcpp::Logger &logger);
}
