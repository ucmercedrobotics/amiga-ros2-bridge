#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <amiga_interfaces/srv/get_tree_info.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <algorithm>
#include <unordered_map>

using GetTreeInfo = amiga_interfaces::srv::GetTreeInfo;
using json = nlohmann::json;

constexpr double EARTH_RADIUS_M = 6378137.0;
constexpr uint8_t ROW_WAYPOINT_LAT = 0;
constexpr uint8_t ROW_WAYPOINT_LON = 1;

class OrchardManagementServiceNode : public rclcpp::Node {
 public:
  OrchardManagementServiceNode()
      : rclcpp::Node("orchard_management_service_node") {
    this->declare_parameter<std::string>("json_topic", std::string("/orchard/tree_info_json"));
    this->declare_parameter<double>("x_offset", double(0.0));
    this->declare_parameter<double>("y_offset", double(0.0));

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

      double x_offset = this->get_parameter("x_offset").as_double();
      double y_offset = this->get_parameter("y_offset").as_double();
      auto apply_offset_to_keys = [&](json &obj, const char *lat_key, const char *lon_key) {
        if (!obj.contains(lat_key) || !obj.contains(lon_key) ||
            !obj[lat_key].is_number() || !obj[lon_key].is_number()) {
          return;
        }
        double adj_lat = 0.0;
        double adj_lon = 0.0;
        this->add_meters_to_gps(
            obj[lat_key].get<double>(), obj[lon_key].get<double>(),
            x_offset, y_offset,
            adj_lat, adj_lon);
        obj[lat_key] = adj_lat;
        obj[lon_key] = adj_lon;
      };

      auto apply_offset_to_row_waypoints = [&](json &waypoints) {
        if (!waypoints.is_array()) {
          return;
        }
        for (auto &wp : waypoints) {
          if (!wp.is_array() || wp.size() <= ROW_WAYPOINT_LON ||
              !wp[ROW_WAYPOINT_LAT].is_number() || !wp[ROW_WAYPOINT_LON].is_number()) {
            continue;
          }
          double adj_lat = 0.0;
          double adj_lon = 0.0;
          this->add_meters_to_gps(
              wp[ROW_WAYPOINT_LAT].get<double>(), wp[ROW_WAYPOINT_LON].get<double>(),
              x_offset, y_offset,
              adj_lat, adj_lon);
          wp[ROW_WAYPOINT_LAT] = adj_lat;
          wp[ROW_WAYPOINT_LON] = adj_lon;
        }
      };

      auto project_obj = [&](const json &obj) {
        json out = json::object();
        for (const char *field : {"tree_index", "row", "col", "lat", "lon", "row_waypoints", "aisle_entrances"}) {
          if (include_field(field) && obj.contains(field)) {
            out[field] = obj[field];
          }
        }
        apply_offset_to_keys(out, "lat", "lon");

        if (out.contains("row_waypoints")) {
          apply_offset_to_row_waypoints(out["row_waypoints"]);
        }

        if (out.contains("aisle_entrances") && out["aisle_entrances"].is_array()) {
          for (auto &entrance : out["aisle_entrances"]) {
            if (!entrance.is_object()) {
              continue;
            }
            apply_offset_to_keys(entrance, "lat", "lon");
          }
        }
        return out;
      };

      // New JSON format is a wrapped object with at least:
      // - "trees": array of tree records
      // - "aisle_entrances": array of entrance records
      // - "aisle_to_entrance_indices": object mapping aisle id -> [entrance_index...]
      if (!data.is_object()) {
        response->json = std::string("[]");
        return;
      }

      const auto trees_it = data.find("trees");
      if (trees_it == data.end() || !trees_it->is_array()) {
        response->json = std::string("[]");
        return;
      }
      const json &trees = *trees_it;

      // Determine filter key based on index_type
      std::string key;
      switch (request->index_type) {
        case amiga_interfaces::srv::GetTreeInfo::Request::TREE_INDEX:
          key = "tree_index"; break;
        case amiga_interfaces::srv::GetTreeInfo::Request::ROW_INDEX:
          key = "row"; break;
        case amiga_interfaces::srv::GetTreeInfo::Request::COL_INDEX:
          key = "col"; break;
        case amiga_interfaces::srv::GetTreeInfo::Request::AISLE_INDEX:
          key = "aisle_index"; break;
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

      if (request->index_type == amiga_interfaces::srv::GetTreeInfo::Request::AISLE_INDEX) {
        if (!data.contains("aisle_to_entrance_indices") || !data["aisle_to_entrance_indices"].is_object() ||
            !data.contains("aisle_entrances") || !data["aisle_entrances"].is_array()) {
          response->json = std::string("[]");
          return;
        }

        std::unordered_map<int, json> entrance_by_index;
        for (const auto &entrance : data["aisle_entrances"]) {
          if (!entrance.is_object() || !entrance.contains("entrance_index") || !entrance["entrance_index"].is_number_integer()) {
            continue;
          }
          json projected = entrance;
          apply_offset_to_keys(projected, "lat", "lon");
          entrance_by_index[projected["entrance_index"].get<int>()] = projected;
        }

        std::vector<int> aisle_ids = filters;
        if (aisle_ids.empty()) {
          for (auto it = data["aisle_to_entrance_indices"].begin(); it != data["aisle_to_entrance_indices"].end(); ++it) {
            try {
              aisle_ids.push_back(std::stoi(it.key()));
            } catch (...) {
            }
          }
        }

        for (int aisle_id : aisle_ids) {
          const std::string aisle_key = std::to_string(aisle_id);
          if (!data["aisle_to_entrance_indices"].contains(aisle_key) ||
              !data["aisle_to_entrance_indices"][aisle_key].is_array()) {
            continue;
          }

          json out = json::object();
          out["aisle_index"] = aisle_id;
          if (include_field("aisle_entrances")) {
            out["aisle_entrances"] = json::array();
            for (const auto &idx : data["aisle_to_entrance_indices"][aisle_key]) {
              if (!idx.is_number_integer()) {
                continue;
              }
              const int entrance_idx = idx.get<int>();
              auto it = entrance_by_index.find(entrance_idx);
              if (it != entrance_by_index.end()) {
                out["aisle_entrances"].push_back(it->second);
              }
            }
          }
          arr.push_back(out);
        }
      } else {
        for (const auto &obj : trees) {
          if (!key.empty()) {
            if (!obj.contains(key) || !obj[key].is_number()) continue;
            int val = obj[key].get<int>();
            if (!filters.empty() && std::find(filters.begin(), filters.end(), val) == filters.end()) {
              continue;
            }
          }
          arr.push_back(project_obj(obj));
        }
      }

      response->json = arr.dump();
      RCLCPP_INFO(this->get_logger(), "Returning %zu records", arr.size());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service error: %s", e.what());
      response->json = std::string("[]");
    }
  }

  void add_meters_to_gps(
      double lat_deg, double lon_deg,
      double dx_east_m, double dy_north_m,
      double &out_lat_deg, double &out_lon_deg)
  {
    double lat_rad = lat_deg * M_PI / 180.0;

    // Meters → degrees
    double dlat = dy_north_m / EARTH_RADIUS_M;
    double dlon = dx_east_m / (EARTH_RADIUS_M * std::cos(lat_rad));

    out_lat_deg = lat_deg + (dlat * 180.0 / M_PI);
    out_lon_deg = lon_deg + (dlon * 180.0 / M_PI);
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
