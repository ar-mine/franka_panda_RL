//
// Created by armine on 12/12/22.
//

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "moveit_control/moveit_cpp_base.h"

namespace am_franka_controllers{
    volatile bool MoveitCppBase::keepRunning_ = true;

MoveitCppBase::MoveitCppBase(const rclcpp::Node::SharedPtr& node) : node_(node){
    
    static const std::string PLANNING_GROUP = "panda_arm";

    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    robot_model_ptr_ = moveit_cpp_ptr->getRobotModel();
    robot_start_state_ = planning_components_->getStartState();
    joint_model_group_ptr_ = robot_model_ptr_->getJointModelGroup(PLANNING_GROUP);

    RCLCPP_INFO(node->get_logger(), "Initialize MoveitCppBase");

}

bool MoveitCppBase::MoveFromCurrent(const geometry_msgs::msg::Pose& target_pose){
    geometry_msgs::msg::PoseStamped target;
    target.pose = target_pose;
    target.header.frame_id = "panda_link0";
    planning_components_->setGoal(target, "panda_link8");
    auto plan_solution = planning_components_->plan();
    if (plan_solution){
        planning_components_->execute();
        return true;
    }
    else{
        return false;
    }
}

void MoveitCppBase::run(){
    signal(SIGINT, MoveitCppBase::signalHandler);

    while(MoveitCppBase::keepRunning_){
        ;
    }
}

void MoveitCppBase::signalHandler(int){
    MoveitCppBase::keepRunning_ = false;
}

}


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_cpp_base", "", node_options);
    RCLCPP_INFO(node->get_logger(), "Initialize node");

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto move_cpp_node = am_franka_controllers::MoveitCppBase(node);
//    geometry_msgs::msg::Pose target_pose;
//    target_pose.orientation.w = 1.0;
//    target_pose.position.x = 0.28;
//    target_pose.position.y = -0.2;
//    target_pose.position.z = 0.5;
//    move_cpp_node.MoveFromCurrent(target_pose);
    move_cpp_node.run();


    RCLCPP_INFO(node->get_logger(), "Shutting down.");
    rclcpp::shutdown();

    return 0;
}
