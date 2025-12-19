#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <amiga_interfaces/srv/get_tree_info.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <algorithm>

using GetTreeInfo = amiga_interfaces::srv::GetTreeInfo;
using json = nlohmann::json;

class OrchardManagementServiceNode : public rclcpp::Node {
 public:
  OrchardManagementServiceNode()
      : rclcpp::Node("orchard_management_service_node") {
    this->declare_parameter<std::string>("json_topic", std::string("/orchard/tree_info_json"));

    service_ = this->create_service<GetTreeInfo>(
        "/orchard/get_tree_info",
        std::bind(&OrchardManagementServiceNode::handle_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    json_sub_ = this->create_subscription<std_msgs::msg::String>(
        this->get_parameter("json_topic").as_string(), 10,
        [this](const std_msgs::msg::String &msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          last_json_ = msg.data;
          RCLCPP_INFO(this->get_logger(), "Received tree info JSON over topic (%zu bytes)", msg.data.size());
        });

    RCLCPP_INFO(this->get_logger(),
                "Orchard service ready on /orchard/get_tree_info (consuming JSON topic)");
  }

 private:
  void handle_request(const std::shared_ptr<GetTreeInfo::Request> request,
                      std::shared_ptr<GetTreeInfo::Response> response) {
    try {
      std::string payload;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        payload = last_json_;
      }
      if (payload.empty()) {
        RCLCPP_WARN(this->get_logger(), "No JSON cached from topic");
        response->json = std::string("[]");
        return;
      }

      json data;
      try {
        data = json::parse(payload);
      } catch (const std::exception &e) {
        response->json = std::string("[]");
        return;
      }

      auto include_field = [&](const std::string &k) {
        const auto &fields = request->fields;
        return fields.empty() || std::find(fields.begin(), fields.end(), k) != fields.end();
      };

      auto project_obj = [&](const json &obj) {
        json out = json::object();
        if (include_field("tree_index") && obj.contains("tree_index")) out["tree_index"] = obj["tree_index"];
        if (include_field("row") && obj.contains("row")) out["row"] = obj["row"];
        if (include_field("col") && obj.contains("col")) out["col"] = obj["col"];
        if (include_field("lat") && obj.contains("lat")) out["lat"] = obj["lat"];
        if (include_field("lon") && obj.contains("lon")) out["lon"] = obj["lon"];
        if (include_field("row_waypoints") && obj.contains("row_waypoints")) out["row_waypoints"] = obj["row_waypoints"];
        return out;
      };
      if (!data.is_array()) {
        response->json = std::string("[]");
        return;
      }

      // Determine filter key based on index_type
      std::string key;
      switch (request->index_type) {
        case amiga_interfaces::srv::GetTreeInfo::Request::TREE_INDEX:
          key = "tree_index"; break;
        case amiga_interfaces::srv::GetTreeInfo::Request::ROW_INDEX:
          key = "row"; break;
        case amiga_interfaces::srv::GetTreeInfo::Request::COL_INDEX:
          key = "col"; break;
        default:
          key.clear();
      }

      std::vector<int> filters;
      filters.reserve(request->indicies.size());
      for (auto v : request->indicies) filters.push_back(static_cast<int>(v));

      // Log the query
      if (!key.empty() && !filters.empty()) {
        std::string filter_str = key + " in [";
        for (size_t i = 0; i < filters.size(); ++i) {
          filter_str += std::to_string(filters[i]);
          if (i < filters.size() - 1) filter_str += ", ";
        }
        filter_str += "]";
        RCLCPP_INFO(this->get_logger(), "Tree query: %s", filter_str.c_str());
      } else if (key.empty()) {
        RCLCPP_INFO(this->get_logger(), "Tree query: return all trees");
      }

      json arr = json::array();
      for (const auto &obj : data) {
        if (!key.empty()) {
          if (!obj.contains(key) || !obj[key].is_number()) continue;
          int val = obj[key].get<int>();
          if (!filters.empty() && std::find(filters.begin(), filters.end(), val) == filters.end()) {
            continue;
          }
        }
        arr.push_back(project_obj(obj));
      }
      response->json = arr.dump();
      RCLCPP_INFO(this->get_logger(), "Returning %zu trees", arr.size());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service error: %s", e.what());
      response->json = std::string("[]");
    }
  }

  rclcpp::Service<GetTreeInfo>::SharedPtr service_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr json_sub_;
  std::mutex mtx_;
  std::string last_json_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrchardManagementServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
