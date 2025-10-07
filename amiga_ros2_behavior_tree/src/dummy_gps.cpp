#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class DummyGPSActionServer : public rclcpp::Node
{
public:
    DummyGPSActionServer()
        : Node("dummy_gps_action_server")
    {
        server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "follow_gps_waypoints",
            std::bind(&DummyGPSActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DummyGPSActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&DummyGPSActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        std::string uuid_str = "Received goal request with UUID: ";
        for (auto const &id : uuid)
            uuid_str += std::to_string(id) + " ";

        RCLCPP_DEBUG(this->get_logger(), "%s", uuid_str.c_str());
        RCLCPP_INFO(this->get_logger(), "Received navigation goal to pose: (%.2f, %.2f)",
                    goal->pose.pose.position.x, goal->pose.pose.position.y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToPose>>)
    {
        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToPose>> goal_handle)
    {
        // Run asynchronously
        std::thread([this, goal_handle]()
                    {
            auto feedback = std::make_shared<NavigateToPose::Feedback>();
            auto result = std::make_shared<NavigateToPose::Result>();

            // Simulate navigation progress
            const int simulation_steps = 2;
            for (int i = 0; i < simulation_steps; i++)
            {
                if (goal_handle->is_canceling())
                {
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled during navigation");
                    return;
                }

                // Update feedback with current position (not working)
                feedback->current_pose.header.stamp = this->get_clock()->now();
                feedback->current_pose.header.frame_id = "map";
                feedback->current_pose.pose.position.x = i * 0.1;
                feedback->current_pose.pose.position.y = i * 0.1;
                feedback->current_pose.pose.orientation.w = 1.0;

                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Navigation progress: %d/%d", i+1, simulation_steps);

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

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
