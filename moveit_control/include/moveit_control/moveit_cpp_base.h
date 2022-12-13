//
// Created by armine on 12/12/22.
//

#ifndef BUILD_MOVEIT_CPP_BASE_H
#define BUILD_MOVEIT_CPP_BASE_H

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

namespace am_franka_controllers{

class MoveitCppBase {
public:
    explicit MoveitCppBase(const rclcpp::Node::SharedPtr& node);

    bool MoveFromCurrent(const geometry_msgs::msg::Pose& target_pose);

    void run();

    static volatile bool keepRunning_;

private:
    const rclcpp::Node::SharedPtr& node_;

    std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_;
    moveit::core::RobotModelConstPtr robot_model_ptr_;
    moveit::core::RobotStatePtr robot_start_state_;
    const moveit::core::JointModelGroup * joint_model_group_ptr_;

    static void signalHandler(int);
};

}

#endif //BUILD_MOVEIT_CPP_BASE_H
