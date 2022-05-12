//
// Created by armine on 2/21/22.
//
#include "franka_control/moveit_circle.h"

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_moveit_logger");


void home_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<double> home_joint_positions = {0, -45, 0, -135, 0, 90, 45};
//    deg2rad(home_joint_positions);

    move_group.setJointValueTarget(home_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Move to home pose %s", success ? "" : "FAILED");

    if (success)
        move_group.move();
}

void draw_circle(moveit::planning_interface::MoveGroupInterface& move_group,
                 moveit_visual_tools::MoveItVisualTools &visual_tools,
                 const moveit::core::JointModelGroup* joint_model_group)
{
    int factor = 2;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose;
    for (int i = 0; i <= 180; i+=factor)
    {
        target_pose.position.x = sin(i*PI/180)*0.3;
        target_pose.position.y = cos(i*PI/180)*0.3;
        target_pose.position.z = 0.35;
        target_pose.orientation.x = 1.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;

        waypoints.push_back(target_pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Circle path (%.2f%% acheived)", fraction * 100.0);

    visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
    visual_tools.trigger();

    move_group.execute(trajectory);
}

void pose_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    geometry_msgs::msg::Pose home_pose;
    home_pose.position.x = 0;
    home_pose.position.y = 0.3;
    home_pose.position.z = 0.35;
    home_pose.orientation.x = 1.0;
    home_pose.orientation.y = 0;
    home_pose.orientation.z = 0;
    home_pose.orientation.w = 0.0;

    move_group.setApproximateJointValueTarget(home_pose, "panda_hand_tcp");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    const moveit::planning_interface::MoveItErrorCode ret = move_group.plan(my_plan);
    bool success = (ret == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Move to initial pose %s", success ? "" : "FAILED");
    if (success)
        move_group.move();
}

int main(int argc, char **argv) {
    // Node init
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("franka_moveit_circle", node_options);

    //
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the *JointModelGroup*. Throughout MoveIt, the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "panda_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    const moveit::core::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");

    // Getting Basic Information
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());


    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0",
                                                        "rviz_visual_tools", move_group.getRobotModel());

    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script */
    /* via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    RCLCPP_INFO(LOGGER, "Initial finished!");


    for (int i = 0; i < 5; i++) {
    home_move(move_group);

    rclcpp::sleep_for(std::chrono::seconds(2));

    pose_move(move_group);

    rclcpp::sleep_for(std::chrono::seconds(2));

    draw_circle(move_group, visual_tools, joint_model_group);

    visual_tools.trigger();
    }
    rclcpp::shutdown();
    return 0;
}