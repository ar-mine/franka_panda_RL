//
// Created by armine on 10/31/22.
//

#ifndef BUILD_CARTESIAN_POSITION_CONTROLLER_H
#define BUILD_CARTESIAN_POSITION_CONTROLLER_H

#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <kdl/frames.hpp>

#include "am_franka_controllers/controller_base.h"
#include "franka_interface/action/cart_pose_set.hpp"

namespace am_franka_controllers{

    class CartesianPositionController : public ControllerBase{
        public:
        using CartPoseSet = franka_interface::action::CartPoseSet;
        using GoalHandleCartPoseSet = rclcpp_action::ServerGoalHandle<CartPoseSet>;

        explicit CartesianPositionController(const std::string& node_name, const rclcpp::NodeOptions& node_options,
        const std::string& robot_ip);

        void timer_callback();
        void target_pose_callback(const geometry_msgs::msg::Pose::ConstSharedPtr msg);
        void gripper_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;

        private:
        rclcpp_action::Server<CartPoseSet>::SharedPtr cart_pose_server_;
        rclcpp_action::GoalResponse cart_pose_goal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const CartPoseSet::Goal>& goal);
        rclcpp_action::CancelResponse cart_pose_cancel(const std::shared_ptr<GoalHandleCartPoseSet>& goal_handle);
        void cart_pose_accepted(const std::shared_ptr<GoalHandleCartPoseSet>& goal_handle);
        void execute(const std::shared_ptr<GoalHandleCartPoseSet>& goal_handle);

        int pose_error_compute();

        const double position_eps = 0.01;
        const double orientation_eps = 0.15;
        const double position_max_vel = 0.10;
        const double position_max_acc = 4.0;
        const double orientation_max_vel = 0.10;
        const double orientation_max_acc = 4.0;

        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;

        int gripper_status = -1;
    };
}

#endif //BUILD_CARTESIAN_POSITION_CONTROLLER_H
