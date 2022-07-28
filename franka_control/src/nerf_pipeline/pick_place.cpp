#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include <rapidjson/writer.h>
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"

#include "geometry_msgs/msg/pose.h"
#include "std_msgs/msg/int32.hpp"

#include <franka/gripper.h>

#include "am_robot_utils/moveit/moveit_base.h"
#include "am_robot_utils/geometry/calculation.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_control.nerf_pipeline.command_executor");

bool json_read(const std::string& file_name, const std::string& key, std::vector<double>& data)
{
    using namespace rapidjson;

    FILE* fp_in = fopen(file_name.c_str(), "r");
    char readBuffer[65536];
    FileReadStream is(fp_in, readBuffer, sizeof(readBuffer));
    Document document;
    document.ParseStream(is);
    fclose(fp_in);

    Value& data_array = document[key.c_str()];
    for(auto &d : data_array.GetArray())
    {
        data.push_back(d.GetDouble());
    }
    return true;
}

bool json_read(const std::string& file_name, const std::string& key, std::vector<std::array<double, 3>>& data)
{
    using namespace rapidjson;

    FILE* fp_in = fopen(file_name.c_str(), "r");
    char readBuffer[65536];
    FileReadStream is(fp_in, readBuffer, sizeof(readBuffer));
    Document document;
    document.ParseStream(is);
    fclose(fp_in);

    Value& data_array = document[key.c_str()];
    for(auto &dd : data_array.GetArray())
    {
        std::array<double, 3> tmp{};
        int count = 0;
        for(auto &d : dd.GetArray())
        {
            tmp.at(count) = d.GetDouble();
            count++;
        }
        data.push_back(tmp);
    }
    return true;
}

class Runner : public Am::MoveitBase
{
public:
    explicit Runner(const rclcpp::NodeOptions& node_options) : MoveitBase("follow_targets", node_options, 1),
                                                               gripper_("172.16.0.2")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        step_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/step_", 3);
        flag_subscription_ = this->create_subscription<std_msgs::msg::Int32>("/flag", rclcpp::QoS(1),
                                                                             std::bind(&Runner::flag_callback, this, std::placeholders::_1));

        json_read("/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_demo_bringup/src/demo_hand/joints_data.json",
                  "joints_1", pick_pose);
        json_read("/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_demo_bringup/src/demo_hand/joints_data.json",
                  "joints_2", place_pose);
        json_read("/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_demo_bringup/src/demo_hand/joints_data.json",
                  "path", path_points);

//        move_group_.setJointValueTarget(pick_pose);
//        move_group_.move();
//        trajectory_load();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&Runner::timer_callback, this));
        gripper_.homing();
        // Sleep to ensure system stable
        rclcpp::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(LOGGER, "Node initialization successful.");
    }

    void run()
    {
        step_ = 0;
        for(size_t i=0; i<3; i++)
        {
            move_group_.setJointValueTarget(pick_pose);
            move_group_.move();
            trajectory_load();

            step_ = 1;
            rclcpp::sleep_for(std::chrono::seconds(1));
            if(flag_==0)
            {
                RCLCPP_INFO(LOGGER, "Executing...");
                move_group_.asyncExecute(trajectory_);
                while(true)
                {
                    if(flag_==1)
                    {
                        RCLCPP_INFO(LOGGER, "Stop...");
                        move_group_.stop();
                        break;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(100));
                }
            }
            rclcpp::sleep_for(std::chrono::seconds(2));
            RCLCPP_INFO(LOGGER, "Try to grasp...");

            geometry_msgs::msg::Pose pose;
            for(size_t j=0; j<10; j++)
            {
                if(Am::transform_get(tf_buffer_, LOGGER, "panda_link0", "hand_pose", pose))
                    break;
            }

            step_ = 0;
            pose.position.z = 0.05;
            move_group_.setPoseTarget(pose, "panda_hand_tcp");
            move_group_.move();
            pose.position.z = 0.007;
            move_group_.setPoseTarget(pose, "panda_hand_tcp");
            move_group_.move();
//            this->hand_action(false, LOGGER);
            gripper_.grasp(0.038, 0.04, 30);
            pose.position.z = 0.05;
            move_group_.setPoseTarget(pose, "panda_hand_tcp");
            move_group_.move();

            move_group_.setJointValueTarget(place_pose);
            move_group_.move();
//            gripper_.grasp(0.18, 0.04, 30);

            step_ = 2;
            rclcpp::sleep_for(std::chrono::seconds(3));

            step_ = 0;
            Am::transform_get(tf_buffer_, LOGGER, "panda_link0", "lattice_"+std::to_string(i), pose);
            pose.position.z = 0.4;
            move_group_.setPoseTarget(pose, "panda_hand_tcp");
            move_group_.move();
            pose.position.z = 0.3;
            move_group_.setPoseTarget(pose, "panda_hand_tcp");
            move_group_.move();
//            this->hand_action(true, LOGGER);
            gripper_.grasp(0.18, 0.04, 50);
            pose.position.z = 0.4;
            move_group_.setPoseTarget(pose, "panda_hand_tcp");
            move_group_.move();

        }
        moveit_move_ready(LOGGER);
    }

    void test()
    {
        moveit_move_ready(LOGGER);
        int count = 0;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = 1.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;
        for(auto point : path_points)
        {
            target_pose.position.x = point.at(0);
            target_pose.position.y = point.at(1);
            target_pose.position.z = point.at(2);
            waypoints.push_back(target_pose);
        }
        for(auto point : waypoints)
        {
            RCLCPP_INFO(LOGGER, "%f, %f, %f", point.position.x, point.position.y, point.position.z);
        }
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        for(auto t : trajectory.joint_trajectory.points)
        {
            std::cout<<t.time_from_start.sec<<" "<<t.time_from_start.nanosec<<std::endl;
        }
        move_group_.execute(trajectory);

    }

    void timer_callback()
    {
        std_msgs::msg::Int32 data;
        data.data = step_;
        step_publisher_->publish(data);
    }

    void flag_callback(std_msgs::msg::Int32::ConstSharedPtr msg)
    {
        flag_ = msg->data;
    }

    void trajectory_load()
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = 1.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;
        for(auto point : path_points)
        {
            target_pose.position.x = point.at(0);
            target_pose.position.y = point.at(1);
            target_pose.position.z = point.at(2);
            waypoints.push_back(target_pose);
        }
//        for(auto point : waypoints)
//        {
//            RCLCPP_INFO(LOGGER, "%f, %f, %f", point.position.x, point.position.y, point.position.z);
//        }
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_);

//        for(auto t : trajectory.joint_trajectory.points)
//        {
//            std::cout<<t.time_from_start.sec<<" "<<t.time_from_start.nanosec<<std::endl;
//        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr step_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flag_subscription_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    int flag_ = 0;
private:
    std::vector<double> pick_pose;
    std::vector<double> place_pose;
    std::vector<std::array<double, 3>> path_points;
    moveit_msgs::msg::RobotTrajectory trajectory_;

    rclcpp::TimerBase::SharedPtr timer_;

    franka::Gripper gripper_;

    int step_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    const std::shared_ptr<Runner> target_follower = std::make_shared<Runner>(node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(target_follower);
    std::thread([&executor]() { executor.spin(); }).detach();

    target_follower->run();
//    target_follower->test();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}