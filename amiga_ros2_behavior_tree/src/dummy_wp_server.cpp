#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>

#include "amiga_interfaces/action/wpfollow.hpp"

using WpFollow = amiga_interfaces::action::Wpfollow;

class DummyGPSActionServer : public rclcpp::Node
{
public:
    DummyGPSActionServer()
        : Node("dummy_gps_action_server")
    {
        server_ = rclcpp_action::create_server<WpFollow>(
            this,
            "follow_gps_waypoints",
            std::bind(&DummyGPSActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DummyGPSActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&DummyGPSActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<WpFollow>::SharedPtr server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const WpFollow::Goal> goal)
    {
        std::string uuid_str = "Received goal request with UUID: ";
        for (auto const &id : uuid)
            uuid_str += std::to_string(id) + " ";

        RCLCPP_DEBUG(this->get_logger(), "%s", uuid_str.c_str());
        RCLCPP_INFO(this->get_logger(), "Received navigation goal to pose: (%.2f, %.2f)",
                    goal->lat, goal->lon);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<WpFollow>>)
    {
        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<WpFollow>> goal_handle)
    {
        // Run asynchronously
        std::thread([this, goal_handle]()
                    {
            auto feedback = std::make_shared<WpFollow::Feedback>();
            auto result = std::make_shared<WpFollow::Result>();

            RCLCPP_INFO(this->get_logger(), "Executing goal...");
            RCLCPP_INFO(this->get_logger(), "Navigating to (%.8f, %.8f)",
                        goal_handle->get_goal()->lat, goal_handle->get_goal()->lon);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!"); })
            .detach();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummyGPSActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
