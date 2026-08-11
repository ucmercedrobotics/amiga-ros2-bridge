#include "amiga_ros2_behavior_tree/planner_client.hpp"

#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <memory>
#include <sstream>

#include <nlohmann/json.hpp>

namespace amiga_bt::planner {

namespace {

using json = nlohmann::json;

bool set_nonblocking(int fd) {
  int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) return false;
  return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool set_blocking(int fd) {
  int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) return false;
  return ::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) == 0;
}

std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

}  // namespace

ParsedHeaders parse_http_headers(const std::string &raw) {
  ParsedHeaders headers;
  auto terminator = raw.find("\r\n\r\n");
  if (terminator == std::string::npos) return headers;

  headers.complete = true;
  headers.header_end = terminator + 4;

  auto status_line_end = raw.find("\r\n");
  if (status_line_end != std::string::npos) {
    std::istringstream status_line(raw.substr(0, status_line_end));
    std::string http_version;
    status_line >> http_version >> headers.status_code;
  }

  std::string header_block = raw.substr(0, terminator);
  std::string lower_block = to_lower(header_block);
  auto cl_pos = lower_block.find("content-length:");
  if (cl_pos != std::string::npos) {
    auto value_start = cl_pos + std::string("content-length:").size();
    auto value_end = header_block.find("\r\n", value_start);
    std::string value = header_block.substr(value_start, value_end - value_start);
    auto first = value.find_first_not_of(" \t");
    auto last = value.find_last_not_of(" \t");
    if (first != std::string::npos) {
      try {
        headers.content_length = std::stoul(value.substr(first, last - first + 1));
      } catch (const std::exception &) {
        // Malformed Content-Length: fall back to reading until EOF.
      }
    }
  }
  return headers;
}

HttpResult http_request(const std::string &host, int port, const std::string &method,
                         const std::string &path, const std::string &body,
                         std::chrono::milliseconds timeout) {
  HttpResult result;

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo *resolved = nullptr;
  int gai = ::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &resolved);
  if (gai != 0 || resolved == nullptr) {
    result.error = std::string("getaddrinfo failed: ") + gai_strerror(gai);
    return result;
  }
  std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> resolved_guard(resolved, &freeaddrinfo);

  int fd = ::socket(resolved->ai_family, resolved->ai_socktype, resolved->ai_protocol);
  if (fd < 0) {
    result.error = std::string("socket() failed: ") + std::strerror(errno);
    return result;
  }

  set_nonblocking(fd);
  int rc = ::connect(fd, resolved->ai_addr, resolved->ai_addrlen);
  if (rc < 0 && errno != EINPROGRESS) {
    result.error = std::string("connect() failed: ") + std::strerror(errno);
    ::close(fd);
    return result;
  }
  if (rc < 0) {
    pollfd pfd{fd, POLLOUT, 0};
    int pr = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
    if (pr <= 0) {
      result.error = "connect() timed out";
      ::close(fd);
      return result;
    }
    int so_error = 0;
    socklen_t len = sizeof(so_error);
    ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len);
    if (so_error != 0) {
      result.error = std::string("connect() failed: ") + std::strerror(so_error);
      ::close(fd);
      return result;
    }
  }
  set_blocking(fd);

  timeval tv{};
  tv.tv_sec = timeout.count() / 1000;
  tv.tv_usec = (timeout.count() % 1000) * 1000;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  std::ostringstream request;
  request << method << " " << path << " HTTP/1.1\r\n"
          << "Host: " << host << "\r\n"
          << "Content-Type: application/json\r\n"
          << "Content-Length: " << body.size() << "\r\n"
          << "Connection: close\r\n"
          << "\r\n"
          << body;
  std::string request_str = request.str();

  size_t sent = 0;
  while (sent < request_str.size()) {
    ssize_t n = ::send(fd, request_str.data() + sent, request_str.size() - sent, 0);
    if (n <= 0) {
      if (n < 0 && errno == EINTR) continue;
      result.error = std::string("send() failed: ") + std::strerror(errno);
      ::close(fd);
      return result;
    }
    sent += static_cast<size_t>(n);
  }

  std::string raw;
  ParsedHeaders headers;
  char buf[4096];
  while (true) {
    ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n < 0) {
      if (errno == EINTR) continue;
      result.error = std::string("recv() failed: ") + std::strerror(errno);
      ::close(fd);
      return result;
    }
    if (n == 0) break;  // peer closed the connection
    raw.append(buf, static_cast<size_t>(n));
    headers = parse_http_headers(raw);
    if (headers.complete && headers.content_length) {
      size_t have = raw.size() - headers.header_end;
      if (have >= *headers.content_length) break;
    }
    // No Content-Length: keep reading until the peer closes (we sent
    // "Connection: close", so a compliant server will).
  }
  ::close(fd);

  if (!headers.complete) {
    result.error = "malformed HTTP response (no header terminator)";
    return result;
  }
  result.connected = true;
  result.status_code = headers.status_code;
  result.body = headers.content_length
                    ? raw.substr(headers.header_end, *headers.content_length)
                    : raw.substr(headers.header_end);
  return result;
}

std::string build_register_body(const RegisterInfo &info) {
  json j;
  j["robot_id"] = info.robot_id;
  if (info.name) j["name"] = *info.name;
  j["schema"] = info.schema;
  if (info.schema_sha256) j["schema_sha256"] = *info.schema_sha256;

  json bt_endpoint;
  bt_endpoint["host"] = info.bt_host ? json(*info.bt_host) : json(nullptr);
  bt_endpoint["port"] = info.bt_port;
  bt_endpoint["protocol"] = "tcp-lenprefix-v1";
  j["bt_endpoint"] = bt_endpoint;

  if (!info.capability_actions.empty() || info.has_manipulator) {
    json capabilities;
    if (!info.capability_actions.empty()) capabilities["actions"] = info.capability_actions;
    if (info.has_manipulator) capabilities["has_manipulator"] = *info.has_manipulator;
    j["capabilities"] = capabilities;
  }

  j["status"] = info.status;
  if (info.battery_pct) j["battery_pct"] = *info.battery_pct;
  if (info.lat && info.lon) j["position"] = {{"lat", *info.lat}, {"lon", *info.lon}};
  j["heartbeat_interval_s"] = info.heartbeat_interval_s;
  return j.dump();
}

std::string build_heartbeat_body(const HeartbeatInfo &info) {
  json j;
  j["status"] = info.status;
  if (info.battery_pct) j["battery_pct"] = *info.battery_pct;
  if (info.lat && info.lon) j["position"] = {{"lat", *info.lat}, {"lon", *info.lon}};
  if (info.current_mission_id) j["current_mission_id"] = *info.current_mission_id;
  return j.dump();
}

std::string build_ack(const AckInfo &info) {
  json j;
  j["accepted"] = info.accepted;
  j["robot_id"] = info.robot_id;
  j["mission_id"] = info.mission_id ? json(*info.mission_id) : json(nullptr);
  j["error"] = info.error ? json(*info.error) : json(nullptr);
  return j.dump();
}

std::optional<std::string> extract_mission_id(const std::string &frame2_json) {
  if (frame2_json.empty()) return std::nullopt;
  try {
    json j = json::parse(frame2_json);
    if (j.contains("mission") && j["mission"].is_object()) {
      const auto &mission = j["mission"];
      if (mission.contains("mission_id") && mission["mission_id"].is_string()) {
        return mission["mission_id"].get<std::string>();
      }
    }
  } catch (const json::parse_error &) {
    // Not JSON, or not the shape we expect -- treat like "no mission key".
  }
  return std::nullopt;
}

PlannerClient::PlannerClient(std::string host, int port) : host_(std::move(host)), port_(port) {}

RegisterResult PlannerClient::register_robot(const RegisterInfo &info) {
  RegisterResult result;
  HttpResult response =
      http_request(host_, port_, "POST", "/robots/register", build_register_body(info));
  if (!response.connected) {
    result.error = response.error;
    return result;
  }
  if (response.status_code != 200) {
    result.error = "planner returned HTTP " + std::to_string(response.status_code) + ": " +
                    response.body;
    return result;
  }
  try {
    json j = json::parse(response.body);
    result.ok = j.value("ok", false);
    result.ttl_s = j.value("ttl_s", 15);
    result.schema_known = j.value("schema_known", true);
  } catch (const json::parse_error &e) {
    result.error = std::string("failed to parse register response: ") + e.what();
  }
  return result;
}

bool PlannerClient::heartbeat(const std::string &robot_id, const HeartbeatInfo &info,
                               bool *not_found_out) {
  if (not_found_out) *not_found_out = false;
  HttpResult response = http_request(host_, port_, "POST", "/robots/" + robot_id + "/heartbeat",
                                      build_heartbeat_body(info));
  if (!response.connected) return false;
  if (response.status_code == 404) {
    if (not_found_out) *not_found_out = true;
    return false;
  }
  return response.status_code == 200;
}

void PlannerClient::deregister(const std::string &robot_id) {
  http_request(host_, port_, "DELETE", "/robots/" + robot_id, "");
}

}  // namespace amiga_bt::planner
