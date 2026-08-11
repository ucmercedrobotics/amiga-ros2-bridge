#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "amiga_ros2_behavior_tree/planner_client.hpp"

using namespace std::chrono_literals;

class TcpDemuxNode : public rclcpp::Node {
 public:
  TcpDemuxNode() : rclcpp::Node("tcp_demux_node") {
    this->declare_parameter<int>("port", 12346);
    this->declare_parameter<bool>("payload_length_included", true);
    this->declare_parameter<std::string>("order", std::string("xml_then_json"));
    this->declare_parameter<bool>("expect_json", true);
    this->declare_parameter<int>("default_frame_size", 65536);
    this->declare_parameter<std::string>("mission_topic", std::string("/mission/xml"));
    this->declare_parameter<std::string>("orchard_topic", std::string("/orchard/tree_info_json"));

    // Planner discovery/heartbeat contract. planner_host empty (the default)
    // disables it entirely, so a robot with no planner configured -- e.g. a
    // dev box using netcat, per the README -- behaves exactly as before.
    this->declare_parameter<std::string>("planner_host", std::string(""));
    this->declare_parameter<int>("planner_port", 8003);
    this->declare_parameter<std::string>("robot_id", std::string(""));
    this->declare_parameter<std::string>("robot_name", std::string(""));
    this->declare_parameter<std::string>("schema", std::string("amiga_btcpp"));
    this->declare_parameter<std::string>("schema_sha256", std::string(""));
    this->declare_parameter<std::vector<std::string>>("capability_actions",
                                                        std::vector<std::string>{});
    this->declare_parameter<bool>("has_manipulator", false);
    this->declare_parameter<int>("heartbeat_interval_s", 5);
    this->declare_parameter<double>("battery_pct", -1.0);
    this->declare_parameter<double>("min_battery_pct_for_mission", -1.0);

    mission_pub_ = this->create_publisher<std_msgs::msg::String>(
        this->get_parameter("mission_topic").as_string(), 1);
    orchard_pub_ = this->create_publisher<std_msgs::msg::String>(
        this->get_parameter("orchard_topic").as_string(), 1);

    std::string robot_id_param = this->get_parameter("robot_id").as_string();
    if (!robot_id_param.empty()) {
      robot_id_ = robot_id_param;
    } else {
      char hostname[256] = {0};
      robot_id_ = (::gethostname(hostname, sizeof(hostname) - 1) == 0) ? hostname
                                                                        : "amiga-unknown";
    }
    battery_pct_ = this->get_parameter("battery_pct").as_double();
    min_battery_pct_for_mission_ = this->get_parameter("min_battery_pct_for_mission").as_double();

    std::string planner_host = this->get_parameter("planner_host").as_string();
    if (!planner_host.empty()) {
      int planner_port = this->get_parameter("planner_port").as_int();
      planner_client_ =
          std::make_unique<amiga_bt::planner::PlannerClient>(planner_host, planner_port);
      discovery_running_ = true;
      discovery_thread_ = std::thread([this]() { this->discovery_loop(); });
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "planner_host not set; skipping planner registration/heartbeat");
    }

    server_thread_ = std::thread([this]() { this->server_loop(); });
  }

  ~TcpDemuxNode() override {
    running_ = false;
    discovery_running_ = false;
    // discovery_thread_ wakes up within one sleep slice; server_thread_ can
    // be sitting in a blocking accept() with no client pending, which may
    // take longer to unwind. Deregister as soon as discovery_thread_ is
    // down rather than after both threads join, so a slow/blocked accept()
    // can't sit between "we're shutting down" and the planner finding out.
    if (discovery_thread_.joinable()) discovery_thread_.join();
    if (planner_client_ && registered_) {
      planner_client_->deregister(robot_id_);
    }
    if (server_thread_.joinable()) server_thread_.join();
  }

 private:
  // Registers once, then heartbeats on heartbeat_interval_s until shutdown.
  // Runs on its own thread since both calls block on network I/O and must
  // not stall mission intake on server_thread_.
  void discovery_loop() {
    amiga_bt::planner::RegisterInfo reg;
    reg.robot_id = robot_id_;
    std::string name = this->get_parameter("robot_name").as_string();
    if (!name.empty()) reg.name = name;
    reg.schema = this->get_parameter("schema").as_string();
    std::string schema_sha256 = this->get_parameter("schema_sha256").as_string();
    if (!schema_sha256.empty()) reg.schema_sha256 = schema_sha256;
    reg.bt_port = this->get_parameter("port").as_int();
    reg.capability_actions = this->get_parameter("capability_actions").as_string_array();
    bool has_manipulator = this->get_parameter("has_manipulator").as_bool();
    if (has_manipulator) reg.has_manipulator = true;
    if (battery_pct_ >= 0.0) reg.battery_pct = battery_pct_;
    int heartbeat_interval_s = this->get_parameter("heartbeat_interval_s").as_int();
    reg.heartbeat_interval_s = heartbeat_interval_s;

    while (rclcpp::ok() && discovery_running_) {
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        reg.status = current_status_;
      }
      auto result = planner_client_->register_robot(reg);
      if (result.ok) {
        registered_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Registered with planner as '%s' (ttl_s=%d, schema_known=%s)",
                    robot_id_.c_str(), result.ttl_s, result.schema_known ? "true" : "false");
        if (!result.schema_known) {
          RCLCPP_WARN(this->get_logger(),
                      "Planner does not know schema '%s'; this robot will not receive missions "
                      "until it does",
                      reg.schema.c_str());
        }
        break;
      }
      RCLCPP_ERROR(this->get_logger(), "Planner registration failed: %s -- retrying in 5s",
                   result.error.c_str());
      sleep_while_running(5s);
    }

    while (rclcpp::ok() && discovery_running_) {
      sleep_while_running(std::chrono::seconds(std::max(heartbeat_interval_s, 1)));
      if (!discovery_running_ || !rclcpp::ok()) break;

      amiga_bt::planner::HeartbeatInfo hb;
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        hb.status = current_status_;
        hb.current_mission_id = current_mission_id_;
      }
      if (battery_pct_ >= 0.0) hb.battery_pct = battery_pct_;

      bool not_found = false;
      bool ok = planner_client_->heartbeat(robot_id_, hb, &not_found);
      if (!ok && not_found) {
        RCLCPP_WARN(this->get_logger(),
                    "Planner lost our registration (404); re-registering");
        auto result = planner_client_->register_robot(reg);
        if (result.ok) {
          registered_ = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Re-registration failed: %s", result.error.c_str());
        }
      } else if (!ok) {
        RCLCPP_WARN(this->get_logger(), "Heartbeat to planner failed");
      }
    }
  }

  // Sleeps in short slices so shutdown (discovery_running_ = false) doesn't
  // have to wait out a multi-second retry/heartbeat interval.
  void sleep_while_running(std::chrono::milliseconds total) {
    const auto slice = 100ms;
    for (auto elapsed = 0ms; elapsed < total && discovery_running_ && rclcpp::ok();
         elapsed += slice) {
      std::this_thread::sleep_for(slice);
    }
  }

  void server_loop() {
    int port = this->get_parameter("port").as_int();
    bool length_included = this->get_parameter("payload_length_included").as_bool();
    std::string order = this->get_parameter("order").as_string();
    bool expect_json = this->get_parameter("expect_json").as_bool();
    int default_frame_size = this->get_parameter("default_frame_size").as_int();

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
          // Read up to a default maximum number of bytes (no explicit length prefix)
          if (default_frame_size <= 0) default_frame_size = 65536;
          out.clear();
          out.reserve(static_cast<size_t>(default_frame_size));

          const size_t chunk = 4096;
          std::vector<char> buf(std::min(static_cast<size_t>(default_frame_size), chunk));
          size_t total = 0;
          while (total < static_cast<size_t>(default_frame_size)) {
            size_t to_read = std::min(buf.size(), static_cast<size_t>(default_frame_size) - total);
            ssize_t n = ::read(client_fd, buf.data(), to_read);
            if (n > 0) {
              out.append(buf.data(), static_cast<size_t>(n));
              total += static_cast<size_t>(n);
              // If we read less than requested and no more data is immediately available,
              // continue loop to try to read remaining until max or EOF.
              continue;
            } else if (n == 0) {
              // Connection closed; stop reading this frame.
              break;
            } else {
              if (errno == EINTR) continue;
              RCLCPP_ERROR(this->get_logger(), "read() failed: %s", std::strerror(errno));
              return false;
            }
          }
          return !out.empty();
        }

        // Length-prefixed frame: 4-byte network-endian size followed by payload
        uint32_t sz_n = 0;
        if (!read_exact(&sz_n, sizeof(sz_n))) return false;
        size_t sz = static_cast<size_t>(ntohl(sz_n));
        out.resize(sz);
        if (sz > 0 && !read_exact(&out[0], sz)) return false;
        return true;
      };

      // Frame 3: length-prefixed JSON ack, written back to the planner on
      // the same connection. A robot that never writes one is still
      // compatible -- the planner just records "sent, unacknowledged" -- so
      // a send failure here only logs, it doesn't unwind the mission.
      auto write_frame = [&](const std::string &payload) -> bool {
        uint32_t sz_n = htonl(static_cast<uint32_t>(payload.size()));
        if (::send(client_fd, &sz_n, sizeof(sz_n), 0) != static_cast<ssize_t>(sizeof(sz_n))) {
          return false;
        }
        size_t total = 0;
        while (total < payload.size()) {
          ssize_t n = ::send(client_fd, payload.data() + total, payload.size() - total, 0);
          if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return false;
          }
          total += static_cast<size_t>(n);
        }
        return true;
      };

      auto accept_and_ack = [&](const std::optional<std::string> &mission_id) {
        bool accepted = true;
        std::optional<std::string> reject_reason;
        if (min_battery_pct_for_mission_ >= 0.0 && battery_pct_ >= 0.0 &&
            battery_pct_ < min_battery_pct_for_mission_) {
          accepted = false;
          reject_reason = "battery too low";
        }
        if (accepted) {
          std::lock_guard<std::mutex> lock(status_mutex_);
          current_status_ = "busy";
          current_mission_id_ = mission_id;
        }
        amiga_bt::planner::AckInfo ack{accepted, robot_id_, mission_id, reject_reason};
        if (!write_frame(amiga_bt::planner::build_ack(ack))) {
          RCLCPP_WARN(this->get_logger(), "Failed to write mission ack");
        }
      };

      if (!expect_json) {
        // Only a single XML frame is expected
        std::string xml;
        bool ok = read_frame(xml);
        if (ok) {
          std_msgs::msg::String xml_msg;
          xml_msg.data = xml;
          mission_pub_->publish(xml_msg);
          RCLCPP_INFO(this->get_logger(), "Published XML (%zu)", xml_msg.data.size());
          accept_and_ack(std::nullopt);
        }
      } else {
        // JSON payload requires length header
        if (!length_included) {
          RCLCPP_ERROR(this->get_logger(),
                       "expect_json=true requires payload_length_included=true (JSON must be length-prefixed)");
        } else {
          // Expect two frames respecting order
          std::string first, second;
          bool ok1 = read_frame(first);
          bool ok2 = ok1 && read_frame(second);
          if (ok2) {
            std_msgs::msg::String xml_msg, json_msg;
            if (order == "json_then_xml") {
              json_msg.data = first;
              xml_msg.data = second;
            } else {  // default: xml_then_json
              xml_msg.data = first;
              json_msg.data = second;
            }
            mission_pub_->publish(xml_msg);
            orchard_pub_->publish(json_msg);
            RCLCPP_INFO(this->get_logger(), "Published XML (%zu) and JSON (%zu)", xml_msg.data.size(),
                        json_msg.data.size());
            accept_and_ack(amiga_bt::planner::extract_mission_id(json_msg.data));
          }
        }
      }

      ::close(client_fd);
      ::close(server_fd);
    }
  }

  std::atomic<bool> running_{true};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr orchard_pub_;
  std::thread server_thread_;

  std::string robot_id_;
  std::unique_ptr<amiga_bt::planner::PlannerClient> planner_client_;
  std::thread discovery_thread_;
  std::atomic<bool> discovery_running_{false};
  std::atomic<bool> registered_{false};
  std::atomic<double> battery_pct_{-1.0};
  double min_battery_pct_for_mission_ = -1.0;

  // Shared between server_thread_ (writes, on mission receipt) and
  // discovery_thread_ (reads, for the periodic heartbeat body).
  std::mutex status_mutex_;
  std::string current_status_{"idle"};
  std::optional<std::string> current_mission_id_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpDemuxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
