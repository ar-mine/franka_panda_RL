//
// Created by armine on 2/23/22.
//
#include "franka_control/ros2_ReachClient.h"

namespace franka_control
{
    ReachActionClient::ReachActionClient(const std::shared_ptr<rclcpp::Node>& node_)
    {
        this->node_ = node_;
        this->reach_action_client = rclcpp_action::create_client<Reach>(node_, "Reach");

    }

    void ReachActionClient::send_goal()
    {
        using namespace std::placeholders;

        if (!this->reach_action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->node_->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = Reach::Goal();
        goal_msg.current_states = {0.1, 0.1, 0.1};

        RCLCPP_INFO(this->node_->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Reach>::SendGoalOptions();
        send_goal_options.goal_response_callback =
                std::bind(&ReachActionClient::goal_response_callback, this, _1);
//            send_goal_options.feedback_callback =
//                    std::bind(&ReachActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
                std::bind(&ReachActionClient::result_callback, this, _1);
        this->reach_action_client->async_send_goal(goal_msg, send_goal_options);
    }

    void ReachActionClient::goal_response_callback(std::shared_future<GoalHandleReach::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->node_->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void ReachActionClient::feedback_callback(
            GoalHandleReach::SharedPtr,
            const std::shared_ptr<const Reach::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto number : feedback->partial_waypoints) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->node_->get_logger(), ss.str().c_str());
    }

    void ReachActionClient::result_callback(const GoalHandleReach::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->node_->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->node_->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->node_->get_logger(), "Unknown result code");
                return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->waypoints) {
            ss << number << " ";
            output.push_back(number);
        }

//        RCLCPP_INFO(this->node_->get_logger(), ss.str().c_str());
    }

}
