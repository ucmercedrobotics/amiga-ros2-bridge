#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_gps_waypoints.hpp> // for NavigateToGPS example

using FollowGPSWaypoints = nav2_msgs::action::FollowGPSWaypoints;

class DummyGPSActionServer : public rclcpp::Node
{
public:
    DummyGPSActionServer()
        : Node("dummy_gps_action_server")
    {
        server_ = rclcpp_action::create_server<FollowGPSWaypoints>(
            this,
            "follow_gps_waypoints",
            std::bind(&DummyGPSActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DummyGPSActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&DummyGPSActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<FollowGPSWaypoints>::SharedPtr server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowGPSWaypoints::Goal> goal)
    {
        std::string uuid_str = "Received goal request with UUID: ";
        for (auto const &id : uuid)
            uuid_str += std::to_string(id) + " ";

        RCLCPP_DEBUG(this->get_logger(), "%s", uuid_str.c_str());
        RCLCPP_INFO(this->get_logger(), "Received goal with %zu waypoint(s)", goal->gps_poses.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowGPSWaypoints>>)
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowGPSWaypoints>> goal_handle)
    {
        // Run asynchronously
        std::thread([this, goal_handle]()
                    {
            auto feedback = std::make_shared<FollowGPSWaypoints::Feedback>();
            auto result = std::make_shared<FollowGPSWaypoints::Result>();

            for (size_t i = 0; i < goal_handle->get_goal()->gps_poses.size(); i++)
            {
                if (goal_handle->is_canceling())
                {
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled at waypoint %zu", i);
                    return;
                }

                feedback->current_waypoint = i;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Processing waypoint (%f, %f)",
                    goal_handle->get_goal()->gps_poses[i].position.latitude,
                    goal_handle->get_goal()->gps_poses[i].position.longitude);

                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            result->error_code = FollowGPSWaypoints::Result::NONE;
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
