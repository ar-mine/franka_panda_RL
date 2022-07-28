#include "cmath"

#include "std_msgs/msg/float32_multi_array.hpp"

#include "franka_control/utils/moveit_utils.h"
#include "franka_control/utils/geometry_utils.h"

const std::string MOVE_GROUP = "panda_manipulator";

class HandProcessor : public rclcpp::Node {
public:
    /// Constructor
    explicit HandProcessor(const std::shared_ptr<rclcpp::Node> &node);

    /// Move group interface and planning scene interface for the robot
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    /// Publisher for target pose
//    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    /// Subscription for hand's relative x_y_z
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr relative_xyz_sub_;
    /// Timer
    rclcpp::TimerBase::SharedPtr timer_;
    /// Tf Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;

    /// Target pose that is used to detect changes
    geometry_msgs::msg::Pose previous_target_pose_;
    /// Tf from effector to end camera
    geometry_msgs::msg::Pose transform_eef2cam;
    /// Tf from camera to base link
    geometry_msgs::msg::Pose transform_cam2base;

    struct Xyz {
        float x, y, z;
    };

    Xyz hand_array;
private:
    /// Callback for Timer
    void TimerCallback();

    /// Callback for xyz array
    void XyzCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);

    bool new_frame = false;

    rclcpp::Logger logger = this->get_logger();
    std::vector<geometry_msgs::msg::Pose> waypoints;

    rclcpp::Time time_begin;
    rclcpp::Time time_now;
};

HandProcessor::HandProcessor(const std::shared_ptr<rclcpp::Node>& node) : Node("hand_processor"),
                                                                          move_group_(node, MOVE_GROUP)
{
    // Use upper joint velocity and acceleration limits
    this->move_group_.setMaxAccelerationScalingFactor(0.5);
    this->move_group_.setMaxVelocityScalingFactor(0.5);
    // Set Planner
    this->move_group_.setPlannerId("RRTConnectkConfigDefault");
    // Add collision to planning scene
    add_collision_full(move_group_, planning_scene_interface_);

    hand_array = {-1, -1, -1};

    // Tf
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Subscribe to target pose
//    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1));
    relative_xyz_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/hand_tracker/array", rclcpp::QoS(1), std::bind(&HandProcessor::XyzCallback, this, std::placeholders::_1));

    // Timer
    timer_  = this->create_wall_timer(std::chrono::milliseconds(100),
                                      std::bind(&HandProcessor::TimerCallback, this));

    rclcpp::sleep_for(std::chrono::seconds(2));

    bool ret = transform_get(tf_buffer, logger, "camera_color_optical_frame", "panda_hand_tcp",
                             transform_eef2cam);

    RCLCPP_INFO(logger, "Initialization successful.");
}

void HandProcessor::TimerCallback()
{
    if(hand_array.z > 0 and new_frame)
    {
        time_begin = this->get_clock()->now();
        RCLCPP_INFO(logger, "x:%.2f, y:%.2f, z:%.2f", hand_array.x, hand_array.y, hand_array.z);

        double theta_x = -atan2(hand_array.y, hand_array.z);
        double theta_x_y = atan2(hand_array.x, sqrt(pow(hand_array.y, 2) + pow(hand_array.z, 2)));
        double distance = sqrt(pow(hand_array.x, 2) + pow(hand_array.y, 2) + pow(hand_array.z, 2));
        RCLCPP_INFO(logger, "theta_x:%.2f, theta_x_y:%.2f, distance:%.2f", theta_x, theta_x_y, distance);

        bool ret = transform_get(tf_buffer, logger, "panda_link0", "camera_color_optical_frame",
                                 transform_cam2base);
        RCLCPP_INFO(logger, "t_x:%.2f, t_y:%.2f, t_z:%.2f",
                    transform_cam2base.position.x, transform_cam2base.position.y, transform_cam2base.position.z);
        RCLCPP_INFO(logger, "r_x:%.2f, r_y:%.2f, r_z:%.2f, r_w:%.2f",
                    transform_cam2base.orientation.x, transform_cam2base.orientation.y, transform_cam2base.orientation.z, transform_cam2base.orientation.w);

        tf2::Quaternion quaternion;
        geometry_msgs::msg::Pose tf_pose;
        quaternion.setRPY(theta_x, theta_x_y,0);
        quaternion = quaternion.normalize();
        quaternion_to_orientation(quaternion, tf_pose);

        pose_multiply(tf_pose, transform_cam2base);

        RCLCPP_INFO(logger, "t_x:%.2f, t_y:%.2f, t_z:%.2f",
                    tf_pose.position.x, tf_pose.position.y, tf_pose.position.z);
        RCLCPP_INFO(logger, "r_x:%.2f, r_y:%.2f, r_z:%.2f, r_w:%.2f",
                    tf_pose.orientation.x, tf_pose.orientation.y, tf_pose.orientation.z, tf_pose.orientation.w);

        auto transform_eef2cam_tmp = transform_eef2cam;
        pose_multiply(transform_eef2cam_tmp, tf_pose);
        RCLCPP_INFO(logger, "t_x:%.2f, t_y:%.2f, t_z:%.2f",
                    transform_eef2cam_tmp.position.x, transform_eef2cam_tmp.position.y, transform_eef2cam_tmp.position.z);
        RCLCPP_INFO(logger, "r_x:%.2f, r_y:%.2f, r_z:%.2f, r_w:%.2f",
                    transform_eef2cam_tmp.orientation.x, transform_eef2cam_tmp.orientation.y, transform_eef2cam_tmp.orientation.z, transform_eef2cam_tmp.orientation.w);

//        geometry_msgs::msg::Pose tmp;
//        tmp.position.x = 0.5; tmp.position.y = 0.00; tmp.position.z = 0.4;
//        tmp.orientation.x = 1.0; tmp.orientation.y = 0.00; tmp.orientation.z = 0.00; tmp.orientation.w = 0.00;

//        geometry_msgs::msg::PoseStamped target;
//        target.pose = transform_eef2cam_tmp;
//        target_pose_pub_->publish(target);


        RCLCPP_INFO(this->get_logger(), "Target pose has changed. Planning and executing...");

        time_now = this->get_clock()->now();
        RCLCPP_INFO(logger, "Time diff: %f:", (time_now-time_begin).seconds());
        // Plan and execute motion
        waypoints.clear();
        waypoints.emplace_back(transform_eef2cam_tmp);
//        this->move_group_.setPoseTarget(transform_eef2cam_tmp, "panda_hand_tcp");
//        this->move_group_.plan(plan);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = this->move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        time_now = this->get_clock()->now();
        RCLCPP_INFO(logger, "Time diff: %f:", (time_now-time_begin).seconds());
        this->move_group_.asyncExecute(trajectory);
//        this->move_group_.setPoseTarget(transform_eef2cam_tmp, "panda_hand_tcp");
//        this->move_group_.asyncMove();
        time_now = this->get_clock()->now();
        RCLCPP_INFO(logger, "Time diff: %f:", (time_now-time_begin).seconds());
        new_frame = false;
    }
    else
    {
        // When hand_array is not updated
        ;
    }
}

void HandProcessor::XyzCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
{
    auto array_data = msg->data;
    if(array_data.size() == 3)
    {
        hand_array.x = array_data[0];
        hand_array.y = array_data[1];
        hand_array.z = array_data[2];
        new_frame = true;
    }
    else
    {
        // When the size of data is not 3
        ;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_group_node_", node_options);
    auto target_follower = std::make_shared<HandProcessor>(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(target_follower);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}