//
// Created by armine on 2/21/22.
//
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_moveit_logger");

void posePrint(geometry_msgs::msg::Pose_<std::allocator<void>>& pose)
{
    std::cout<<"position.x:"<<pose.position.x<<std::endl
             <<"position.y:"<<pose.position.y<<std::endl
             <<"position.z:"<<pose.position.z<<std::endl
             <<"orientation.x:"<<pose.orientation.x<<std::endl
             <<"orientation.y:"<<pose.orientation.y<<std::endl
             <<"orientation.z:"<<pose.orientation.z<<std::endl
             <<"orientation.w:"<<pose.orientation.w<<std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("franka_moveit_control", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the *JointModelGroup*. Throughout MoveIt, the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "panda_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Getting Basic Information
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    geometry_msgs::msg::PoseStamped_<std::allocator<void>> current_pose_stamp = move_group.getCurrentPose();
    geometry_msgs::msg::Pose_<std::allocator<void>> current_pose= current_pose_stamp.pose;
    posePrint(current_pose);

    // Planning to a Pose goal
    // We can plan a motion for this group to a desired pose for the end-effector.
    geometry_msgs::msg::Pose home_pose;
    home_pose.position = current_pose.position;
    home_pose.position.x = 0.03844;
    home_pose.position.y = 0.0;
//    home_pose.position.z -= 0.1;
    home_pose.orientation = current_pose.orientation;
//    home_pose.orientation.x = 0.99;
//    home_pose.orientation.y = -0.098;
//    home_pose.orientation.z = 0.045;
//    home_pose.orientation.w = 1.0;

    move_group.setApproximateJointValueTarget(home_pose, "panda_hand_tcp");

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    const moveit::planning_interface::MoveItErrorCode ret = move_group.plan(my_plan);
    std::cout<<"Planning return is:"<<ret.val<<std::endl;
    bool success = (ret == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    if(success)
        move_group.move();

    rclcpp::shutdown();
    return 0;
}
