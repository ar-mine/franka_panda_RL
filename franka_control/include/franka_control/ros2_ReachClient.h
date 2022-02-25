//
// Created by armine on 2/23/22.
//

#ifndef BUILD_ROS2_REACHCLIENT_H
#define BUILD_ROS2_REACHCLIENT_H

#endif //BUILD_ROS2_REACHCLIENT_H

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "franka_action_interface/action/reach.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace franka_control
{
    using Reach = franka_action_interface::action::Reach;
    using GoalHandleReach = rclcpp_action::ClientGoalHandle<Reach>;

    class ReachActionClient
    {
    public:
        explicit ReachActionClient(const std::shared_ptr<rclcpp::Node> &node_);

        std::vector<double> output;
        bool finish_flag = false;

        void send_goal();
    private:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp_action::Client<Reach>::SharedPtr reach_action_client;

        // Callback for action
        void goal_response_callback(std::shared_future<GoalHandleReach::SharedPtr> future);

        void result_callback(const GoalHandleReach::WrappedResult & result);

        void feedback_callback(
                GoalHandleReach::SharedPtr,
                const std::shared_ptr<const Reach::Feedback> feedback);

    };
}