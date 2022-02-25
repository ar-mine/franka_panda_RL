//
// Created by armine on 2/21/22.
//
#include "franka_control/moveit_control.hpp"

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_moveit_logger");


void home_move(moveit::planning_interface::MoveGroupInterface& move_group)
{
    geometry_msgs::msg::PoseStamped_<std::allocator<void>> current_pose_stamp = move_group.getCurrentPose();
    geometry_msgs::msg::Pose_<std::allocator<void>> current_pose = current_pose_stamp.pose;
    posePrint(current_pose);

    // Planning to a Pose goal
    // We can plan a motion for this group to a desired pose for the end-effector.
    geometry_msgs::msg::Pose home_pose;
    home_pose.position = current_pose.position;
    home_pose.position.x = 0.03844 + 0.6;
    home_pose.position.y = 0.0;
    home_pose.position.z = 0.19740;
//    home_pose.orientation = current_pose.orientation;
    home_pose.orientation.x = 1.0;
    home_pose.orientation.y = 0;
    home_pose.orientation.z = 0;
    home_pose.orientation.w = 0.0;

    move_group.setApproximateJointValueTarget(home_pose, "panda_hand_tcp");

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    const moveit::planning_interface::MoveItErrorCode ret = move_group.plan(my_plan);
    std::cout << "Planning return is:" << ret.val << std::endl;
    bool success = (ret == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    if (success)
        move_group.move();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("franka_moveit_control", node_options);

    // Action init
    auto reach_action_client = franka_control::ReachActionClient(move_group_node);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the *JointModelGroup*. Throughout MoveIt, the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "panda_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");

    // Getting Basic Information
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    reach_action_client.send_goal();
    while(!reach_action_client.finish_flag)
    {
        rclcpp::sleep_for(std::chrono::milliseconds (100));
    }

    std::vector<double> output = reach_action_client.output;
    auto point_sum = output.size() / 6;
    RCLCPP_INFO(LOGGER, "There are total %d points.", point_sum);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose;
    for(unsigned long i=0; i<point_sum; i++)
    {
        target_pose.position.x = 0.6+output[i*6];
        target_pose.position.y = output[i*6+1];
        target_pose.position.z = output[i*6+2];
        target_pose.orientation.x = 1.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;

//        posePrint(target_pose);

        waypoints.push_back(target_pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory);

    rclcpp::shutdown();
    return 0;
}