#include "amiga_ros2_behavior_tree/mission_tcp.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace mission_tcp
{

std::string wait_for_mission_tcp(int port, const rclcpp::Logger &logger)
{
    int server_fd = -1;
    int client_fd = -1;
    std::string mission;

    server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
        RCLCPP_FATAL(logger, "socket() failed: %s", std::strerror(errno));
        throw std::runtime_error("socket failed");
    }

    int opt = 1;
    (void)::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(server_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
    {
        RCLCPP_FATAL(logger, "bind() failed on port %d: %s", port, std::strerror(errno));
        ::close(server_fd);
        throw std::runtime_error("bind failed");
    }

    if (::listen(server_fd, 1) < 0)
    {
        RCLCPP_FATAL(logger, "listen() failed: %s", std::strerror(errno));
        ::close(server_fd);
        throw std::runtime_error("listen failed");
    }

    RCLCPP_INFO(logger, "Waiting for mission on TCP port %d...", port);
    socklen_t addrlen = sizeof(address);
    client_fd = ::accept(server_fd, reinterpret_cast<sockaddr *>(&address), &addrlen);
    if (client_fd < 0)
    {
        RCLCPP_FATAL(logger, "accept() failed: %s", std::strerror(errno));
        ::close(server_fd);
        throw std::runtime_error("accept failed");
    }

    char buf[4096];
    while (true)
    {
        ssize_t n = ::read(client_fd, buf, sizeof(buf));
        if (n > 0)
        {
            mission.append(buf, static_cast<size_t>(n));
        }
        else if (n == 0)
        {
            break;
        }
        else
        {
            if (errno == EINTR)
            {
                continue;
            }
            RCLCPP_ERROR(logger, "read() failed: %s", std::strerror(errno));
            break;
        }
    }

    ::close(client_fd);
    ::close(server_fd);

    RCLCPP_INFO(logger, "Received mission (%zu bytes)", mission.size());
    return mission;
}

} // namespace mission_tcp
