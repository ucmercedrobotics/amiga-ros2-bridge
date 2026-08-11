#include "amiga_ros2_behavior_tree/planner_client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using amiga_bt::planner::AckInfo;
using amiga_bt::planner::HeartbeatInfo;
using amiga_bt::planner::RegisterInfo;
using json = nlohmann::json;

TEST(BuildRegisterBody, IncludesRequiredFieldsAndNullHostByDefault) {
  RegisterInfo info;
  info.robot_id = "amiga-01";
  info.schema = "amiga_btcpp";
  info.bt_port = 12346;
  info.capability_actions = {"MoveToTreeID", "SampleLeaf"};
  info.status = "idle";
  info.heartbeat_interval_s = 5;

  json body = json::parse(amiga_bt::planner::build_register_body(info));

  EXPECT_EQ(body["robot_id"], "amiga-01");
  EXPECT_EQ(body["schema"], "amiga_btcpp");
  EXPECT_EQ(body["bt_endpoint"]["port"], 12346);
  EXPECT_EQ(body["bt_endpoint"]["protocol"], "tcp-lenprefix-v1");
  EXPECT_TRUE(body["bt_endpoint"]["host"].is_null());
  EXPECT_EQ(body["status"], "idle");
  EXPECT_EQ(body["heartbeat_interval_s"], 5);
  ASSERT_EQ(body["capabilities"]["actions"].size(), 2u);
  EXPECT_EQ(body["capabilities"]["actions"][0], "MoveToTreeID");
  EXPECT_FALSE(body.contains("name"));
  EXPECT_FALSE(body.contains("schema_sha256"));
  EXPECT_FALSE(body.contains("battery_pct"));
  EXPECT_FALSE(body.contains("position"));
}

TEST(BuildRegisterBody, OptionalFieldsRoundTripWhenSet) {
  RegisterInfo info;
  info.robot_id = "amiga-02";
  info.name = "Amiga #2";
  info.schema = "amiga_btcpp";
  info.schema_sha256 = "9f2c...";
  info.bt_port = 12347;
  info.bt_host = "10.0.0.5";
  info.has_manipulator = true;
  info.battery_pct = 0.87;
  info.lat = 37.361;
  info.lon = -120.4318;

  json body = json::parse(amiga_bt::planner::build_register_body(info));

  EXPECT_EQ(body["name"], "Amiga #2");
  EXPECT_EQ(body["schema_sha256"], "9f2c...");
  EXPECT_EQ(body["bt_endpoint"]["host"], "10.0.0.5");
  EXPECT_EQ(body["capabilities"]["has_manipulator"], true);
  EXPECT_DOUBLE_EQ(body["battery_pct"].get<double>(), 0.87);
  EXPECT_DOUBLE_EQ(body["position"]["lat"].get<double>(), 37.361);
  EXPECT_DOUBLE_EQ(body["position"]["lon"].get<double>(), -120.4318);
}

TEST(BuildHeartbeatBody, OmitsUnsetOptionalFields) {
  HeartbeatInfo info;
  info.status = "busy";
  info.current_mission_id = "a3f1c9";

  json body = json::parse(amiga_bt::planner::build_heartbeat_body(info));

  EXPECT_EQ(body["status"], "busy");
  EXPECT_EQ(body["current_mission_id"], "a3f1c9");
  EXPECT_FALSE(body.contains("battery_pct"));
  EXPECT_FALSE(body.contains("position"));
}

TEST(BuildAck, RejectedIncludesErrorAndNullMissionId) {
  AckInfo ack{false, "amiga-01", std::nullopt, std::string("battery too low")};
  json body = json::parse(amiga_bt::planner::build_ack(ack));

  EXPECT_EQ(body["accepted"], false);
  EXPECT_EQ(body["robot_id"], "amiga-01");
  EXPECT_TRUE(body["mission_id"].is_null());
  EXPECT_EQ(body["error"], "battery too low");
}

TEST(BuildAck, AcceptedIncludesMissionIdAndNullError) {
  AckInfo ack{true, "amiga-01", std::string("a3f1c9"), std::nullopt};
  json body = json::parse(amiga_bt::planner::build_ack(ack));

  EXPECT_EQ(body["accepted"], true);
  EXPECT_EQ(body["mission_id"], "a3f1c9");
  EXPECT_TRUE(body["error"].is_null());
}

TEST(ExtractMissionId, ParsesNestedMissionId) {
  std::string frame2 = R"({"trees": [], "mission": {"mission_id": "a3f1c9", "robot_id": "amiga-01"}})";
  auto id = amiga_bt::planner::extract_mission_id(frame2);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(*id, "a3f1c9");
}

TEST(ExtractMissionId, ReturnsNulloptWhenMissionKeyAbsent) {
  EXPECT_FALSE(amiga_bt::planner::extract_mission_id(R"({"trees": []})").has_value());
}

TEST(ExtractMissionId, ReturnsNulloptForEmptyOrMalformedInput) {
  EXPECT_FALSE(amiga_bt::planner::extract_mission_id("").has_value());
  EXPECT_FALSE(amiga_bt::planner::extract_mission_id("not json").has_value());
}

TEST(ParseHttpHeaders, ReadsStatusCodeAndContentLength) {
  std::string raw = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 13\r\n\r\n{\"ok\": true}";
  auto headers = amiga_bt::planner::parse_http_headers(raw);
  ASSERT_TRUE(headers.complete);
  EXPECT_EQ(headers.status_code, 200);
  ASSERT_TRUE(headers.content_length.has_value());
  EXPECT_EQ(*headers.content_length, 13u);
}

TEST(ParseHttpHeaders, IncompleteWithoutBlankLine) {
  auto headers = amiga_bt::planner::parse_http_headers("HTTP/1.1 200 OK\r\nContent-Length: 2");
  EXPECT_FALSE(headers.complete);
}

namespace {

// A minimal one-shot HTTP server on loopback: accepts a single connection,
// captures the raw request bytes it was sent, and writes back a fixed
// response. Enough to exercise http_request()'s request framing and
// Content-Length-bounded read without a real HTTP library on either side.
class OneShotHttpServer {
 public:
  OneShotHttpServer(int status_code, const std::string &response_body) {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    ::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    ::getsockname(fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    ::listen(fd_, 1);

    thread_ = std::thread([this, status_code, response_body]() {
      int client = ::accept(fd_, nullptr, nullptr);
      if (client < 0) return;
      char buf[4096];
      ssize_t n;
      while ((n = ::recv(client, buf, sizeof(buf), 0)) > 0) {
        received_.append(buf, static_cast<size_t>(n));
        if (received_.find("\r\n\r\n") != std::string::npos) {
          // Assume the small test bodies arrive in one read after headers.
          break;
        }
      }
      std::string response = "HTTP/1.1 " + std::to_string(status_code) + " X\r\n" +
                              "Content-Type: application/json\r\n" +
                              "Content-Length: " + std::to_string(response_body.size()) +
                              "\r\n\r\n" + response_body;
      ::send(client, response.data(), response.size(), 0);
      ::close(client);
    });
  }

  ~OneShotHttpServer() {
    if (thread_.joinable()) thread_.join();
    ::close(fd_);
  }

  int port() const { return port_; }
  const std::string &received() const { return received_; }

 private:
  int fd_ = -1;
  int port_ = 0;
  std::thread thread_;
  std::string received_;
};

}  // namespace

TEST(HttpRequest, RoundTripsRequestAndResponse) {
  OneShotHttpServer server(200, R"({"ok": true, "ttl_s": 15})");

  auto result = amiga_bt::planner::http_request("127.0.0.1", server.port(), "POST",
                                                  "/robots/register", R"({"robot_id": "amiga-01"})");

  ASSERT_TRUE(result.connected) << result.error;
  EXPECT_EQ(result.status_code, 200);
  json body = json::parse(result.body);
  EXPECT_EQ(body["ok"], true);
  EXPECT_EQ(body["ttl_s"], 15);

  EXPECT_NE(server.received().find("POST /robots/register HTTP/1.1"), std::string::npos);
  EXPECT_NE(server.received().find("Content-Length: 24"), std::string::npos);
  EXPECT_NE(server.received().find(R"({"robot_id": "amiga-01"})"), std::string::npos);
}

TEST(HttpRequest, ConnectFailureReportsNotConnected) {
  // Port 1 is a reserved low port nothing will be listening on; connect()
  // should fail (refused) well within the default timeout.
  auto result = amiga_bt::planner::http_request("127.0.0.1", 1, "GET", "/", "",
                                                  std::chrono::milliseconds(500));
  EXPECT_FALSE(result.connected);
  EXPECT_FALSE(result.error.empty());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
