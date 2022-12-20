//
// Created by armine on 12/12/22.
//

#ifndef BUILD_MOVEIT_CPP_BASE_H
#define BUILD_MOVEIT_CPP_BASE_H

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <franka/vacuum_gripper.h>
#include <std_msgs/msg/bool.hpp>

#include "franka_interface/action/cart_pose_set.hpp"

namespace am_franka_controllers{

class MoveitCppBase {
    using CartPoseSet = franka_interface::action::CartPoseSet;
    using GoalHandleCartPoseSet = rclcpp_action::ServerGoalHandle<CartPoseSet>;
public:
    explicit MoveitCppBase(const rclcpp::Node::SharedPtr& node);

    bool move_from_current(const geometry_msgs::msg::Pose& target_pose);

    void run();

    static volatile bool keepRunning_;

private:
    const rclcpp::Node::SharedPtr& node_;

    std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_;
    moveit::core::RobotModelConstPtr robot_model_ptr_;
    moveit::core::RobotStatePtr robot_start_state_;
    const moveit::core::JointModelGroup * joint_model_group_ptr_;

    rclcpp_action::Server<CartPoseSet>::SharedPtr move_server_;

    std::unique_ptr<franka::VacuumGripper> vacuum_gripper_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
    int gripper_status;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    static void signalHandler(int);
};

}

#endif //BUILD_MOVEIT_CPP_BASE_H
