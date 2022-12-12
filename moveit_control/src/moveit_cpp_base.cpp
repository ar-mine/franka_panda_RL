//
// Created by armine on 12/12/22.
//

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "moveit_control/moveit_cpp_base.h"

namespace am_franka_controllers{

MoveitCppBase::MoveitCppBase(const std::string& node_name, const rclcpp::NodeOptions& node_options)
                            : Node(node_name, node_options){
    RCLCPP_INFO(this->get_logger(), "Initialize node");

    static const std::string PLANNING_GROUP = "panda_arm";
    static const std::string LOGNAME = "moveit_cpp_tutorial";

    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(this);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto robot_start_state = planning_components->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
}

}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);
    auto moveit_cpp_node = std::make_shared<am_franka_controllers::MoveitCppBase>("run_moveit_cpp", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(moveit_cpp_node);
    executor.spin();

    return 0;
}
