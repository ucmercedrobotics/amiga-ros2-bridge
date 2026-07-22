#include "amiga_navigation/lidar_object_navigator.hpp"

#include <cmath>
#include <limits>
#include <thread>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::placeholders;

namespace amiga_navigation {

LidarObjectNavigator::LidarObjectNavigator(const rclcpp::NodeOptions& options)
    : Node("lidar_object_navigator", options) {
  this->declare_parameter<std::string>("lidar_topic", "/ouster/points");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("lidar_link", "lidar_link");
  this->declare_parameter<double>("safety_distance", 1.25);
  this->declare_parameter<double>("azimuth_tolerance", 0.5);
  this->declare_parameter<double>("min_object_height", 0.5);
  this->declare_parameter<double>("max_object_height", 1.5);
  this->declare_parameter<double>("min_object_distance", 1.0);
  this->declare_parameter<double>("max_object_distance", 5.0);
  std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  lidar_link_ = this->get_parameter("lidar_link").as_string();
  safety_distance_ = this->get_parameter("safety_distance").as_double();
  azimuth_tolerance_ = this->get_parameter("azimuth_tolerance").as_double();
  min_object_height_ = this->get_parameter("min_object_height").as_double();
  max_object_height_ = this->get_parameter("max_object_height").as_double();
  min_object_distance_ = this->get_parameter("min_object_distance").as_double();
  max_object_distance_ = this->get_parameter("max_object_distance").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, qos,
    std::bind(&LidarObjectNavigator::lidar_callback, this, _1));

  move_in_frame_client_ =
      rclcpp_action::create_client<MoveInFrameAction>(
          this, "move_in_frame");
  rotate_in_frame_client_ =
      rclcpp_action::create_client<RotateInFrameAction>(
          this, "rotate_in_frame");

  action_server_ = rclcpp_action::create_server<NavigateViaLidar>(
      this, "navigate_via_lidar",
      std::bind(&LidarObjectNavigator::handle_goal, this, _1, _2),
      std::bind(&LidarObjectNavigator::handle_cancel, this, _1),
      std::bind(&LidarObjectNavigator::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "LidarObjectNavigator initialized");
}

void LidarObjectNavigator::lidar_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  size_t point_count = static_cast<size_t>(msg->width) * msg->height;
  RCLCPP_DEBUG(this->get_logger(), "Received PointCloud2 with %zu points",
               point_count);
  latest_scan_ = msg;
}

rclcpp_action::GoalResponse LidarObjectNavigator::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const NavigateViaLidar::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(this->get_logger(),
              "Received goal to navigate to object angle %f",
              goal->object_angle);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LidarObjectNavigator::handle_cancel(
    const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LidarObjectNavigator::handle_accepted(
    const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle) {
  std::thread{std::bind(&LidarObjectNavigator::execute, this, _1), goal_handle}
      .detach();
}

void LidarObjectNavigator::execute(
    const std::shared_ptr<GoalHandleNavigateViaLidar> goal_handle) {
  if (!latest_scan_) {
    RCLCPP_WARN(this->get_logger(), "No LiDAR data available yet.");
    return;
  }
  active_goal_handle_ = goal_handle;

  auto result = std::make_shared<NavigateViaLidar::Result>();
  double theta_target = goal_handle->get_goal()->object_angle;
  double azimuth_tolerance = azimuth_tolerance_;

  const auto &pc = *latest_scan_;

  // Get transform from LiDAR frame to base_link
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_link_, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    result->success = false;
    result->message = "TF transform failed";
    goal_handle->abort(result);
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
              "Transform from %s to %s: translation=(%.2f, %.2f, %.2f), rotation=(%.2f, %.2f, %.2f, %.2f)",
              lidar_link_.c_str(), base_frame_.c_str(),
              transform_stamped.transform.translation.x,
              transform_stamped.transform.translation.y,
              transform_stamped.transform.translation.z,
              transform_stamped.transform.rotation.x,
              transform_stamped.transform.rotation.y,
              transform_stamped.transform.rotation.z,
              transform_stamped.transform.rotation.w);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc, "z");

  std::vector<Eigen::Vector3f> selected_points;
  float x, y, z;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    x = *iter_x;
    y = *iter_y;
    z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      continue;

    // Transform point to base_link frame
    geometry_msgs::msg::PointStamped lidar_point, base_point;
    lidar_point.header = pc.header;
    lidar_point.point.x = x;
    lidar_point.point.y = y;
    lidar_point.point.z = z;
    
    tf2::doTransform(lidar_point, base_point, transform_stamped);
    
    float bx = base_point.point.x;
    float by = base_point.point.y;
    float bz = base_point.point.z;

    double azimuth = std::atan2(by, bx);
    double diff = std::atan2(std::sin(azimuth - theta_target),
                            std::cos(azimuth - theta_target));

    if (std::fabs(diff) < azimuth_tolerance) {
      if (bz > min_object_height_ && bz < max_object_height_) {
        selected_points.emplace_back(bx, by, bz);
      }
    }
  }

  if (selected_points.empty()) {
    RCLCPP_WARN(this->get_logger(), "No points found at orientation %.2f rad", theta_target);
    result->success = true;
    result->message = "No points found";
    goal_handle->succeed(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Found %zu points near %.2f rad. Height range: [%.2f, %.2f] m",
              selected_points.size(), theta_target, min_object_height_, max_object_height_);

  bool found_point = false;
  size_t closest_idx = 0;
  float min_dist = std::numeric_limits<float>::max();
  for (size_t i = 0; i < selected_points.size(); ++i) {
    float dist = std::sqrt(selected_points[i](0) * selected_points[i](0) +
                          selected_points[i](1) * selected_points[i](1));
    if (dist < max_object_distance_ && dist > min_object_distance_ && dist < min_dist) {
      closest_idx = i;
      min_dist = dist;
      found_point = true;
    }
  }

  if (!found_point) {
    RCLCPP_WARN(this->get_logger(), 
                "No points found within maximum distance of %.2f m", max_object_distance_);
    result->success = true;
    result->message = "No points within range";
    goal_handle->succeed(result);
    return;
  }

  float bx = selected_points[closest_idx](0);
  float by = selected_points[closest_idx](1);
  float bz = selected_points[closest_idx](2);

  float ground_distance = std::sqrt(bx * bx + by * by);
  float object_angle = std::atan2(by, bx);

  RCLCPP_INFO(this->get_logger(),
              "Object at: (x=%.3f,y=%.3f,z=%.3f) ground_distance=%.2f m, angle=%.2f rad",
              bx, by, bz, ground_distance, object_angle);

  if (!move_in_frame_client_->wait_for_action_server(
          std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
                 "MoveInFrame action server not available!");
    result->success = false;
    result->message = "MoveInFrame action server not available";
    goal_handle->abort(result);
    return;
  }

  // TODO: this isn't ideal
  float target_distance = std::max(0.0f, ground_distance - safety_distance_);
  float target_x = target_distance * std::cos(object_angle);
  float target_y = target_distance * std::sin(object_angle);

  RCLCPP_INFO(this->get_logger(),
              "Sending relative move goal: (%.2f, %.2f) yaw=%.2f",
              target_x, target_y, object_angle);

  if (target_x == 0.0f && target_y == 0.0f) {
    auto goal_msg = RotateInFrameAction::Goal();
    goal_msg.yaw = object_angle;
    
    auto send_goal_options =
        rclcpp_action::Client<RotateInFrameAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&LidarObjectNavigator::rotate_goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&LidarObjectNavigator::rotate_feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&LidarObjectNavigator::rotate_result_callback, this, _1);

    rotate_in_frame_client_->async_send_goal(goal_msg, send_goal_options);
  }
  else {
    auto goal_msg = MoveInFrameAction::Goal();
    goal_msg.x = target_x;
    goal_msg.y = target_y;

    auto send_goal_options =
        rclcpp_action::Client<MoveInFrameAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&LidarObjectNavigator::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&LidarObjectNavigator::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&LidarObjectNavigator::result_callback, this, _1);

    move_in_frame_client_->async_send_goal(goal_msg, send_goal_options);
  }
}


void LidarObjectNavigator::rotate_goal_response_callback(
    const GoalHandleRotateInFrame::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(),
                 "Goal was rejected by RotateInFrameAction server");
    if (active_goal_handle_) {
      auto result = std::make_shared<NavigateViaLidar::Result>();
      result->success = false;
      result->message = "RotateInFrameAction goal rejected";
      active_goal_handle_->abort(result);
    }
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "Goal accepted by RotateInFrameAction server, navigating...");
  }
}

void LidarObjectNavigator::goal_response_callback(
    const GoalHandleMoveInFrame::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(),
                 "Goal was rejected by MoveInFrameAction server");
    if (active_goal_handle_) {
      auto result = std::make_shared<NavigateViaLidar::Result>();
      result->success = false;
      result->message = "MoveInFrameAction goal rejected";
      active_goal_handle_->abort(result);
    }
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "Goal accepted by MoveInFrameAction server, navigating...");
  }
}

void LidarObjectNavigator::rotate_result_callback(
    const GoalHandleRotateInFrame::WrappedResult& result) {
  auto action_result = std::make_shared<NavigateViaLidar::Result>();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
    action_result->success = true;
    action_result->message = "Navigation succeeded";
    active_goal_handle_->succeed(action_result);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Navigation failed");
    action_result->success = false;
    action_result->message = "Navigation failed";
    active_goal_handle_->abort(action_result);
  }
  active_goal_handle_.reset();
}

void LidarObjectNavigator::feedback_callback(
    GoalHandleMoveInFrame::SharedPtr,
    const std::shared_ptr<const MoveInFrameAction::Feedback>
        feedback) {
  RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m",
              feedback->distance_remaining);
  if (active_goal_handle_) {
    auto fb = std::make_shared<NavigateViaLidar::Feedback>();
    fb->distance_remaining = feedback->distance_remaining;
    active_goal_handle_->publish_feedback(fb);
  }
}

void LidarObjectNavigator::result_callback(
    const GoalHandleMoveInFrame::WrappedResult& result) {
  auto action_result = std::make_shared<NavigateViaLidar::Result>();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
    action_result->success = true;
    action_result->message = "Navigation succeeded";
    active_goal_handle_->succeed(action_result);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Navigation failed");
    action_result->success = false;
    action_result->message = "Navigation failed";
    active_goal_handle_->abort(action_result);
  }
  active_goal_handle_.reset();
}

}  // namespace amiga_navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<amiga_navigation::LidarObjectNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
