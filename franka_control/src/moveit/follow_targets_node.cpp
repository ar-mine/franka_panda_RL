#include "std_msgs/msg/float64_multi_array.hpp"

#include "franka_control/utils/moveit_utils.h"
#include "franka_control/utils/geometry_utils.h"

const std::string MOVE_GROUP = "panda_manipulator";
const std::string EEF_LINK = "panda_hand_tcp";

class MoveItFollowTarget : public rclcpp::Node
{
public:
    /// Constructor
    explicit MoveItFollowTarget(rclcpp::NodeOptions node_options);

    /// Move group interface and planning scene interface for the robot
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    /// Subscriber for target pose and target joint states
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_joints_sub_;
    /// Target pose and joint states that is used to detect changes
    geometry_msgs::msg::Pose previous_target_pose_;
    std::vector<double> previous_target_joints_;
private:
    /// Callback that plans and executes trajectory each time the target pose is changed
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    void target_joints_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
};

MoveItFollowTarget::MoveItFollowTarget(rclcpp::NodeOptions node_options) : Node("follow_targets", node_options),
                                                                           move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
    // Use upper joint velocity and acceleration limits
    this->move_group_.setMaxAccelerationScalingFactor(0.1);
    this->move_group_.setMaxVelocityScalingFactor(0.1);

    // Set Planner
    this->move_group_.setPlannerId("RRTConnectkConfigDefault");

    // Subscribe to target pose
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose",
                                                                                  rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));
    target_joints_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("~/target_joints",
                                                                                     rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_joints_callback, this, std::placeholders::_1));


    // Add collision to planning scene
    add_collision_full(move_group_, planning_scene_interface_);

    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MoveItFollowTarget::target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
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

void MoveItFollowTarget::target_joints_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
//    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_group_node_", node_options);

    const std::shared_ptr<MoveItFollowTarget> target_follower = std::make_shared<MoveItFollowTarget>(node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
//    executor.add_node(node);
    executor.add_node(target_follower);
    executor.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}