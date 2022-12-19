//
// Created by armine on 12/12/22.
//

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "moveit_control/moveit_cpp_base.h"
#include "std_msgs/msg/bool.hpp"
#include "franka_interface/srv/stack_pick.hpp"

namespace am_franka_controllers{

    volatile bool MoveitCppBase::keepRunning_ = true;

MoveitCppBase::MoveitCppBase(const rclcpp::Node::SharedPtr& node) : node_(node){
    
    static const std::string PLANNING_GROUP = "panda_arm";

    // MoveitCpp init
    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    // MoveitCpp interface init
    planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    robot_model_ptr_ = moveit_cpp_ptr->getRobotModel();
    robot_start_state_ = planning_components_->getStartState();
    joint_model_group_ptr_ = robot_model_ptr_->getJointModelGroup(PLANNING_GROUP);

    // Controller action server
//    auto action_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    move_server_ = rclcpp_action::create_server<CartPoseSet>(
        node_, "~/move",
        [this](const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const CartPoseSet::Goal>& goal){
            RCLCPP_INFO(node_->get_logger(), "Received goal and target pose: %.2f %.2f %.2f, %.2f %.2f %.2f %.2f",
                       goal->target.position.x, goal->target.position.y, goal->target.position.z,
                       goal->target.orientation.x, goal->target.orientation.y, goal->target.orientation.z, goal->target.orientation.w);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
        [this](const std::shared_ptr<GoalHandleCartPoseSet>& goal_handle){
            RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal, stop the robot.");
            // TODO: write robot stop interface

            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
            },
        [this](const std::shared_ptr<GoalHandleCartPoseSet>& goal_handle){
            geometry_msgs::msg::Pose goal = goal_handle->get_goal()->target;
            auto result = std::make_shared<CartPoseSet::Result>();

            bool err_code = move_from_current(goal);
            result->success = err_code;
            if (err_code){
                RCLCPP_INFO(node_->get_logger(), "Move to target successfully!");
                goal_handle->succeed(result);
            }
            else{
                RCLCPP_WARN(node_->get_logger(), "Cannot move to target pose!");
                goal_handle->succeed(result);
            }

        }
//        rcl_action_server_get_default_options(), action_callback_group_
    );

    // Gripper loop
    vacuum_gripper_ = std::make_unique<franka::VacuumGripper>("172.16.0.2");

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(node->get_logger(), "Initialize MoveitCppBase");

}

bool MoveitCppBase::move_from_current(const geometry_msgs::msg::Pose& target_pose){
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
//    rclcpp::sleep_for(std::chrono::seconds(5));
//
//    rclcpp::Client<franka_interface::srv::StackPick>::SharedPtr client =
//            node_->create_client<franka_interface::srv::StackPick>("stack_pick");
//    auto request = std::make_shared<franka_interface::srv::StackPick::Request>();
//    request->idx = 2;
//    auto result = client->async_send_request(request);
//    if (rclcpp::spin_until_future_complete(node_, result) ==
//        rclcpp::FutureReturnCode::SUCCESS)
//    {
//        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Switch successfully");
//    } else {
//        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
//    }
//    rclcpp::sleep_for(std::chrono::seconds(1));
//    geometry_msgs::msg::TransformStamped t;
//    try {
//         t = tf_buffer_->lookupTransform(
//                "camera_color_optical_frame", "box_pose",
//                tf2::TimePointZero);
//    } catch (const tf2::TransformException & ex) {
//        RCLCPP_INFO(
//                node_->get_logger(), "Could not transform %s to %s: %s",
//                "camera_color_optical_frame", "box_pose", ex.what());
//        return;
//    }
//    RCLCPP_INFO(node_->get_logger(), "Pose: %.4f, %.4f, %.4f || %.4f, %.4f, %.4f, %.4f",
//                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
//                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
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
    move_cpp_node.run();

    RCLCPP_INFO(node->get_logger(), "Shutting down.");
    rclcpp::shutdown();

    return 0;
}
