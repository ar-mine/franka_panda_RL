/*
 * This file is to control robot arm picking target objects while executing predefined trajectory.
 * */
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

bool json_read(const rclcpp::Logger& logger, const std::string& file_name, const std::string& key, std::vector<std::array<double, 3>>& data)
{
    using namespace rapidjson;

    FILE* fp_in = fopen(file_name.c_str(), "r");
    char readBuffer[65536];
    FileReadStream is(fp_in, readBuffer, sizeof(readBuffer));
    Document document;
    document.ParseStream(is);
    fclose(fp_in);

    Value& data_array = document[key.c_str()];
    int count = 0;
    for(auto &dd : data_array.GetArray())
    {
        std::array<double, 3> tmp{};
        int i = 0;
        for(auto &d : dd.GetArray())
        {
            tmp.at(i) = d.GetDouble();
            i++;
        }
        data.push_back(tmp);
        count ++;
    }
    RCLCPP_INFO(logger, "Total %d points are loaded to trajectory.", count);
    return true;
}

class Runner : public Am::MoveitBase
{
public:
    explicit Runner(const rclcpp::NodeOptions& node_options) : MoveitBase("follow_targets", node_options, 1, 0.25, 0.25),
                                                               logger_(this->get_logger())
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
        json_read(logger_,
                "/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_demo_bringup/src/demo_hand/joints_data.json",
                  "path", path_points);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                         std::bind(&Runner::timer_callback, this));

        // Sleep to ensure system stable
        rclcpp::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(this->get_logger(), "Node initialization successful.");
    }

    void run()
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan_temp;

        // Set the mode 0 for detection module
        step_ = 0;
        while(true)
        {
            // Indicate whether the trajectory is executed fully
            bool full_loop = true;
            // Indicate whether the trajectory is not executed
            // that is, it detects target before starting to move
            bool no_loop_detect = false;

            // Move to start of the trajectory
            move_group_.setJointValueTarget(pick_pose);
            move_group_.move();

            // Set step flag to 1
            // it will be sent by topic and means that perception module should start to detect
            step_ = 1;
            // Compute trajectories from current pose
            trajectory_load(path_points);
            // Keep it stable
            rclcpp::sleep_for(std::chrono::seconds(1));

            // If detecting target before moving
            if(flag_==1)
            {
                RCLCPP_INFO(this->get_logger(), "Detect target before moving");
                no_loop_detect = true;
            }

            // Define freq and equal epsilon
            static int freq = 50;
            static double epsilon = 0.02;
            // Indicate which point will be passed
            unsigned long path_idx = 0;
            std::array<double, 3> point_move_to = path_points[path_idx];
            // Not detect target in the beginning
            if(!no_loop_detect)
            {
                RCLCPP_INFO(this->get_logger(), "Executing...");
                move_group_.asyncExecute(trajectory_);
                while(true)
                {
                    // Move point ptr when robot is closed to the trajectory point
                    // Indicate which point has been passed
                    auto current_pose = move_group_.getCurrentPose().pose.position;
                    if((abs(current_pose.x-point_move_to.at(0))+abs(current_pose.y-point_move_to.at(1)))<epsilon)
                    {
                        path_idx++;
                        point_move_to = path_points[path_idx];
                        // When reach the last one point
                        if(path_idx == path_points.size()-1)
                            break;
//                        RCLCPP_INFO(LOGGER, "To %d point", path_idx);
                    }

                    // Check whether the target is detected
                    if(flag_==1)
                    {
                        move_group_.stop();
                        RCLCPP_INFO(logger_, "Stop the trajectory");

                        // Wait for robot being stable and image detection being stable
                        rclcpp::sleep_for(std::chrono::seconds(3));
                        // Fake target, filter those cannot be detected stably
                        if(flag_==0)
                        {
                            // Continue from stopped points
                            trajectory_load({path_points.begin()+path_idx, path_points.end()});
                            move_group_.asyncExecute(trajectory_);
                            RCLCPP_INFO(logger_, "Continue to executing...");
                        }
                        else if(flag_==1)
                        {
                            // Not execute the full trajectory
                            // It means that something is detected
                            full_loop = false;
                            break;
                        }
                    }
                    // Sleep for detection frequency
                    rclcpp::sleep_for(std::chrono::milliseconds(1000/freq));
                }
            }

            // If not detecting anything interesting
            if(full_loop && !no_loop_detect)
            {
                step_ = 0;
                RCLCPP_INFO(logger_, "Complete the full trajectory, not detecting anything");
                break;
            }

            /** After detecting anything **/
            // Try to get transform
            geometry_msgs::msg::Pose pose;
            if(!Am::transform_multi_get(tf_buffer_, logger_, "panda_link0", "hand_pose", pose, 5))
            {
                RCLCPP_ERROR(logger_, "Get transform failed and will exit!");
                break;
            }

            // Stop to detect to save computer resource
            step_ = 0;
            const double height = 0.008;
            std::vector<geometry_msgs::msg::Pose> waypoints;
            pose.position.z = 0.05;
            waypoints.push_back(pose);
            pose.position.z = height;
            waypoints.push_back(pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            move_group_.asyncExecute(trajectory);
            geometry_msgs::msg::PoseStamped current_pose;
            while(true)
            {
                current_pose = move_group_.getCurrentPose();
                if(std::abs(current_pose.pose.position.z - height)<0.004)
                    break;
                rclcpp::sleep_for(std::chrono::milliseconds(1000/freq));
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
//            // Start to try to grasp
//            RCLCPP_INFO(logger_, "Move to target location 1");
//            pose.position.z = 0.05;
//            move_group_.setPoseTarget(pose, "panda_hand_tcp");
//            bool success = (move_group_.plan(plan_temp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//            if(success) {
//                RCLCPP_INFO(logger_, "Plan successfully");
//                move_group_.execute(plan_temp);
//            }
//            else
//                RCLCPP_INFO(logger_, "Plan fail");
//            RCLCPP_INFO(logger_, "Plan to target location 2");
//            pose.position.z = 0.008;
//            move_group_.setPoseTarget(pose, "panda_hand_tcp");
//            success = (move_group_.plan(plan_temp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//
//            // If the target pose for robot can be reached
//            if(success)
//            {
//                RCLCPP_INFO(logger_, "Plan successfully");
//                move_group_.execute(plan_temp);
                if(this->hand_action(logger_,0.03, 0.04, 30, 0.02, 0.02))
                    RCLCPP_INFO(logger_, "Grasp successfully!");
                else
                    RCLCPP_INFO(logger_, "Grasp failed, try again!");
                pose.position.z = 0.2;
                move_group_.setPoseTarget(pose, "panda_hand_tcp");
                move_group_.move();

                // PLace object into box
                move_group_.setJointValueTarget(place_pose);
                move_group_.move();
                current_pose = move_group_.getCurrentPose();
                current_pose.pose.position.z = 0.25;
                move_group_.setPoseTarget(current_pose, "panda_hand_tcp");
                move_group_.move();
                this->hand_action(logger_,0.18, 0.04, 30, 0.02, 0.02);
//            }
//            else
//            {
//                RCLCPP_INFO(logger_, "Cannot reach target pose!");
//            }
        }

        // All tasks are finished, move to home configuration
        moveit_move_ready(logger_);
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

    void trajectory_load(const std::vector<std::array<double, 3>>& path)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = 1.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;
        for(auto point : path)
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
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_);
        RCLCPP_INFO(logger_, "Load trajectory with a fraction: %.2f.", fraction);
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
    rclcpp::Logger logger_;
//    franka::Gripper gripper_;

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