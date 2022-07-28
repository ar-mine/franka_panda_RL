#include "am_robot_utils/moveit/moveit_base.h"

Am::MoveitBase::MoveitBase(const std::string& node_name, const rclcpp::NodeOptions& node_options, int mode)
: Node(node_name, node_options),
move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
    // Use upper joint velocity and acceleration limits
    this->move_group_.setMaxAccelerationScalingFactor(0.15);
    this->move_group_.setMaxVelocityScalingFactor(0.15);

    // Set Planner
    this->move_group_.setPlannerId("RRTConnectkConfigDefault");

    // Only initialize useful resource
    if(mode == 1)
    {
        // Subscribe to target pose
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose",
                                                                                      rclcpp::QoS(1), std::bind(&MoveitBase::target_pose_callback, this, std::placeholders::_1));
        target_joints_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("~/target_joints",
                                                                                         rclcpp::QoS(1), std::bind(&MoveitBase::target_joints_callback, this, std::placeholders::_1));
    }
    else if(mode == 2)
    {

    }
    grasp_client_ptr_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
            this, "/panda_gripper/grasp");

    // Add collision to planning scene
    add_collision_full(move_group_, planning_scene_interface_);

    // Sleep to ensure system stable
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(this->get_logger(), "Basic function initialization successful.");
}

void Am::MoveitBase::target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    // Return if target pose is unchanged
    if (msg->pose == previous_target_pose_)
    {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Target pose has changed. Planning and executing...");

    // Plan and execute motion
    this->move_group_.setPoseTarget(msg->pose, EEF_LINK);
    this->move_group_.move();

    // Update for next callback
    previous_target_pose_ = msg->pose;
}

void Am::MoveitBase::target_joints_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
    // Return if target pose is unchanged
    std::vector<double> target_pose = msg->data;
    if (target_pose == previous_target_joints_)
    {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Target pose has changed. Planning and executing...");

    // Plan and execute motion
    this->move_group_.setJointValueTarget(target_pose);
    this->move_group_.move();

    // Update for next callback
    previous_target_joints_ = target_pose;
}

bool Am::MoveitBase::moveit_move_ready(const rclcpp::Logger& LOGGER)
{
    this->move_group_.setJointValueTarget(std::vector<double>({0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854}));

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (this->move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Move to ready pose %s", success ? "SUCCESS" : "FAILED");

    if (success)
        this->move_group_.move();

    return success;
}

void Am::MoveitBase::hand_action(bool open_close, const rclcpp::Logger& LOGGER)
{

//        this->timer_->cancel();

    if (!this->grasp_client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = franka_msgs::action::Grasp::Goal();
    if(open_close)
        goal_msg.width = 0.18;
    else
        goal_msg.width = 0.02;
    goal_msg.speed = 0.04;
    goal_msg.force = 50;
    goal_msg.epsilon.inner = 0;
    goal_msg.epsilon.outer = 0;

    RCLCPP_INFO(LOGGER, "Sending hand action goal");

    auto send_goal_options = rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions();
//        send_goal_options.goal_response_callback =
//                std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
//        send_goal_options.feedback_callback =
//                std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
            std::bind(&MoveitBase::result_callback, this, std::placeholders::_1);

    this->grasp_client_ptr_->async_send_goal(goal_msg, send_goal_options);

    while(!this->finish_flag)
    {
        rclcpp::sleep_for(std::chrono::milliseconds (100));
    }
    this->finish_flag = false;
}

using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>;
void Am::MoveitBase::result_callback(const GoalHandleGrasp::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
//            RCLCPP_ERROR(LOGGER, "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
//            RCLCPP_ERROR(LOGGER, "Goal was canceled");
            break;
        default:
//            RCLCPP_ERROR(LOGGER, "Unknown result code");
            break;
    }
    finish_flag = true;
}