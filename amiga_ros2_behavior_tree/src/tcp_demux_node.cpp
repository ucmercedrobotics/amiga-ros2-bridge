#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TcpDemuxNode : public rclcpp::Node {
 public:
  TcpDemuxNode() : rclcpp::Node("tcp_demux_node") {
    this->declare_parameter<int>("port", 12346);
    this->declare_parameter<bool>("payload_length_included", true);
    this->declare_parameter<std::string>("order", std::string("xml_then_json"));
    this->declare_parameter<std::string>("mission_topic", std::string("/mission/xml"));
    this->declare_parameter<std::string>("orchard_topic", std::string("/orchard/tree_info_json"));

    mission_pub_ = this->create_publisher<std_msgs::msg::String>(
        this->get_parameter("mission_topic").as_string(), 1);
    orchard_pub_ = this->create_publisher<std_msgs::msg::String>(
        this->get_parameter("orchard_topic").as_string(), 1);

    server_thread_ = std::thread([this]() { this->server_loop(); });
  }

  ~TcpDemuxNode() override {
    running_ = false;
    if (server_thread_.joinable()) server_thread_.join();
  }

 private:
  void server_loop() {
    int port = this->get_parameter("port").as_int();
    bool length_included = this->get_parameter("payload_length_included").as_bool();
    std::string order = this->get_parameter("order").as_string();

    while (rclcpp::ok() && running_) {
      int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
      if (server_fd < 0) {
        RCLCPP_FATAL(this->get_logger(), "socket() failed: %s", std::strerror(errno));
        return;
      }
      int opt = 1;
      if (::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
        RCLCPP_FATAL(this->get_logger(), "setsockopt() failed: %s", std::strerror(errno));
        ::close(server_fd);
        return;
      }
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = INADDR_ANY;
      address.sin_port = htons(static_cast<uint16_t>(port));
      if (::bind(server_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
        RCLCPP_FATAL(this->get_logger(), "bind() failed on port %d: %s", port, std::strerror(errno));
        ::close(server_fd);
        return;
      }
      if (::listen(server_fd, 1) < 0) {
        RCLCPP_FATAL(this->get_logger(), "listen() failed: %s", std::strerror(errno));
        ::close(server_fd);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "tcp_demux: Waiting on port %d...", port);
      socklen_t addrlen = sizeof(address);
      int client_fd = ::accept(server_fd, reinterpret_cast<sockaddr *>(&address), &addrlen);
      if (client_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "accept() failed: %s", std::strerror(errno));
        ::close(server_fd);
        continue;
      }

      auto read_exact = [&](void *dst, size_t len) -> bool {
        size_t total = 0;
        char *ptr = static_cast<char *>(dst);
        while (total < len) {
          ssize_t n = ::read(client_fd, ptr + total, len - total);
          if (n > 0) {
            total += static_cast<size_t>(n);
          } else if (n == 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection closed while reading %zu bytes", len);
            return false;
          } else {
            if (errno == EINTR) continue;
            RCLCPP_ERROR(this->get_logger(), "read() failed: %s", std::strerror(errno));
            return false;
          }
        }
        return true;
      };

      auto read_frame = [&](std::string &out) -> bool {
        if (!length_included) {
          RCLCPP_ERROR(this->get_logger(), "payload_length_included=false unsupported for demux");
          return false;
        }
        uint32_t sz_n = 0;
        if (!read_exact(&sz_n, sizeof(sz_n))) return false;
        size_t sz = static_cast<size_t>(ntohl(sz_n));
        out.resize(sz);
        if (sz > 0 && !read_exact(&out[0], sz)) return false;
        return true;
      };

      // Expect two frames: XML then JSON (or vice versa)
      std::string first, second;
      bool ok1 = read_frame(first);
      bool ok2 = ok1 && read_frame(second);

      if (ok2) {
        std_msgs::msg::String msg1, msg2;
        msg1.data = first;
        msg2.data = second;
        mission_pub_->publish(msg1);
        orchard_pub_->publish(msg2);
        RCLCPP_INFO(this->get_logger(), "Published XML (%zu) and JSON (%zu)", first.size(), second.size());
      }

      ::close(client_fd);
      ::close(server_fd);
    }
  }

  std::atomic<bool> running_{true};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr orchard_pub_;
  std::thread server_thread_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpDemuxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
