// The robot side of the fleet planner's discovery contract: register once,
// heartbeat on an interval, deregister on clean shutdown, and ack the
// mission frame tcp_demux_node already receives over TCP. Three HTTP calls,
// tiny JSON bodies, on the local network -- not enough to justify a real
// HTTP client library, so this talks HTTP/1.1 over a plain socket by hand.
#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace amiga_bt::planner {

// Result of one HTTP round trip. `connected` false means the request never
// got a response at all (DNS/connect/timeout/send/recv failure) -- as
// opposed to a response that arrived with a non-200 status, which is a
// successful round trip the caller still has to check status_code for.
struct HttpResult {
  bool connected = false;
  int status_code = 0;
  std::string body;
  std::string error;
};

// Parsed HTTP response headers, exposed for testing independent of sockets.
struct ParsedHeaders {
  bool complete = false;  // saw the blank line terminating the header block
  int status_code = 0;
  std::optional<size_t> content_length;
  size_t header_end = 0;  // index into the raw buffer right after "\r\n\r\n"
};

ParsedHeaders parse_http_headers(const std::string &raw);

HttpResult http_request(const std::string &host, int port, const std::string &method,
                         const std::string &path, const std::string &body,
                         std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));

struct RegisterInfo {
  std::string robot_id;
  std::optional<std::string> name;
  std::string schema;
  std::optional<std::string> schema_sha256;
  int bt_port = 0;
  // Unset => "host": null in the request, which is the contract's
  // recommended value: the planner substitutes the request's source IP.
  std::optional<std::string> bt_host;
  std::vector<std::string> capability_actions;
  std::optional<bool> has_manipulator;
  std::string status = "idle";
  std::optional<double> battery_pct;
  std::optional<double> lat;
  std::optional<double> lon;
  int heartbeat_interval_s = 5;
};

struct HeartbeatInfo {
  std::string status = "idle";
  std::optional<double> battery_pct;
  std::optional<double> lat;
  std::optional<double> lon;
  std::optional<std::string> current_mission_id;
};

struct RegisterResult {
  bool ok = false;
  int ttl_s = 15;
  bool schema_known = false;
  std::string error;  // set when ok is false
};

struct AckInfo {
  bool accepted;
  std::string robot_id;
  std::optional<std::string> mission_id;
  std::optional<std::string> error;
};

std::string build_register_body(const RegisterInfo &info);
std::string build_heartbeat_body(const HeartbeatInfo &info);
std::string build_ack(const AckInfo &info);

// Pulls "mission.mission_id" out of the frame-2 orchard JSON. Returns
// nullopt if frame 2 was omitted, isn't valid JSON, or carries no mission
// key -- all cases where the ack still has to go out with mission_id: null.
std::optional<std::string> extract_mission_id(const std::string &frame2_json);

class PlannerClient {
 public:
  PlannerClient(std::string host, int port);

  RegisterResult register_robot(const RegisterInfo &info);

  // Returns false on any failure to reach or be acknowledged by the
  // planner. `not_found_out`, if given, is set to true specifically for a
  // 404 -- the contract's signal that the planner restarted and lost the
  // roster, which the caller should answer by registering again.
  bool heartbeat(const std::string &robot_id, const HeartbeatInfo &info,
                 bool *not_found_out = nullptr);

  // Best-effort; the contract says skipping this is fine, the planner's TTL
  // handles it 15s slower.
  void deregister(const std::string &robot_id);

 private:
  std::string host_;
  int port_;
};

}  // namespace amiga_bt::planner
