#include "amiga_ros2_behavior_tree/mission_tcp.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace mission_tcp {

std::string wait_for_mission_tcp(int port, const rclcpp::Logger &logger,
                                 bool payload_length_included) {
  int server_fd = -1;
  int client_fd = -1;
  std::string mission;

  server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    RCLCPP_FATAL(logger, "socket() failed: %s", std::strerror(errno));
    throw std::runtime_error("socket failed");
  }

  int opt = 1;
  if (::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt)) < 0) {
    RCLCPP_FATAL(logger, "setsockopt() failed: %s", std::strerror(errno));
    ::close(server_fd);
    throw std::runtime_error("setsockopt failed");
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(static_cast<uint16_t>(port));

  if (::bind(server_fd, reinterpret_cast<sockaddr *>(&address),
             sizeof(address)) < 0) {
    RCLCPP_FATAL(logger, "bind() failed on port %d: %s", port,
                 std::strerror(errno));
    ::close(server_fd);
    throw std::runtime_error("bind failed");
  }

  if (::listen(server_fd, 1) < 0) {
    RCLCPP_FATAL(logger, "listen() failed: %s", std::strerror(errno));
    ::close(server_fd);
    throw std::runtime_error("listen failed");
  }

  RCLCPP_INFO(logger, "Waiting for mission on TCP port %d...", port);
  socklen_t addrlen = sizeof(address);
  client_fd =
      ::accept(server_fd, reinterpret_cast<sockaddr *>(&address), &addrlen);
  if (client_fd < 0) {
    RCLCPP_FATAL(logger, "accept() failed: %s", std::strerror(errno));
    ::close(server_fd);
    throw std::runtime_error("accept failed");
  }

  auto read_exact = [&](void *dst, size_t len) -> bool {
    size_t total = 0;
    char *ptr = static_cast<char *>(dst);
    while (total < len) {
      ssize_t n = ::read(client_fd, ptr + total, len - total);
      if (n > 0) {
        total += static_cast<size_t>(n);
      } else if (n == 0) {
        RCLCPP_ERROR(logger, "Connection closed while reading %zu bytes", len);
        return false;
      } else {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(logger, "read() failed: %s", std::strerror(errno));
        return false;
      }
    }
    return true;
  };

  size_t payload_size = MISSION_TCP_DEFAULT_PAYLOAD_LENGTH;

  if (payload_length_included) {
    uint32_t payload_size_network = 0;
    if (!read_exact(&payload_size_network, sizeof(payload_size_network))) {
      ::close(client_fd);
      ::close(server_fd);
      return mission;
    }
    payload_size = static_cast<size_t>(ntohl(payload_size_network));
  }

  if (payload_size > 0) {
    mission.resize(payload_size);
    if (!read_exact(&mission[0], payload_size)) {
      ::close(client_fd);
      ::close(server_fd);
      return mission;
    }
  }

  ::close(client_fd);
  ::close(server_fd);

  RCLCPP_INFO(logger, "Received mission (%zu bytes)", mission.size());
  return mission;
}

}  // namespace mission_tcp
