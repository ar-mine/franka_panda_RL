#include <queue>
#include <cstdio>
#include <cmath>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include <rapidjson/writer.h>
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"

#include <franka/robot.h>
#include <franka/gripper.h>

#include "std_msgs/msg/float32_multi_array.hpp"

#include "franka_control/motion_generator.hpp"
#include "franka_control/utils/geometry_utils.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_control.hand_follow_node.cpp");

// Catch ctrl+c to interpret
bool stop_flag = false;
void signal_handler(int sig)
{
    if(sig == SIGINT)
    {
        stop_flag = true;
    }
}

// Ensure the velocity command to meet the limits
double vel_refine(double v_goal, double v_current, double a_current, double a_max, double j_max, double t)
{
    double v = v_current;
    double a = a_current;
    double a_goal = 0.0;

    if(v_goal-v_current > 0)
        a_goal = a_max;
    else if(v_goal-v_current < 0)
        a_goal = -a_max;

    double delta_acc_max = j_max * t;
    if(a_goal-a_current > delta_acc_max)
        a += delta_acc_max;
    else if(a_goal-a_current < -delta_acc_max)
        a -= delta_acc_max;
    else
        a = a_goal;

    double delta_vel_max = a * t;
    if(abs(v_goal-v_current) > abs(delta_vel_max))
        v += delta_vel_max;
//    else if(v_goal-v_current < -delta_vel_max)
//        v -= delta_vel_max;
    else
        v = v_goal;
    return v;
}

bool json_save(const std::string& file_name, const std::string& key, const std::array<double, 7>& data)
{
    using namespace rapidjson;

    FILE* fp_in = fopen(file_name.c_str(), "r");
    char readBuffer[65536];
    FileReadStream is(fp_in, readBuffer, sizeof(readBuffer));
    Document document;
    document.ParseStream(is);
    fclose(fp_in);

    Value& data_array = document[key.c_str()];
    data_array.Clear();
    Document::AllocatorType& allocator = document.GetAllocator();
    for(auto d : data)
    {
        data_array.PushBack(d, allocator);
    }

    FILE* fp_out = fopen(file_name.c_str(), "w"); // 非 Windows 平台使用 "w"
    char writeBuffer[65536];
    FileWriteStream os(fp_out, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    document.Accept(writer);
    fclose(fp_out);

    return true;
}

bool json_save(const std::string& file_name, const std::string& key, const std::vector<std::array<double, 3>>& data)
{
    using namespace rapidjson;

    FILE* fp_in = fopen(file_name.c_str(), "r");
    char readBuffer[65536];
    FileReadStream is(fp_in, readBuffer, sizeof(readBuffer));
    Document document;
    document.ParseStream(is);
    fclose(fp_in);

    Value& data_array = document[key.c_str()];
    data_array.Clear();
    Document::AllocatorType& allocator = document.GetAllocator();
    for(auto dd : data)
    {
        Value tmp(kArrayType);
        for(auto d : dd)
            tmp.PushBack(d, allocator);
        data_array.PushBack(tmp, allocator);
    }

    FILE* fp_out = fopen(file_name.c_str(), "w"); // 非 Windows 平台使用 "w"
    char writeBuffer[65536];
    FileWriteStream os(fp_out, writeBuffer, sizeof(writeBuffer));
    Writer<FileWriteStream> writer(os);
    document.Accept(writer);
    fclose(fp_out);

    return true;
}

class HandTracker : public rclcpp::Node
{
public:
    HandTracker(const std::string& node_name, const rclcpp::NodeOptions& node_options, const std::string& fci_ip);

    void run();

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

    /// Member value of x_y_z
    struct Xyz {
        float x, y, z, ux, uy, flag;
    };
    Xyz hand_array{};

private:
    /// Callback for Timer
    void TimerCallback();
    /// Callback for xyz array
    void XyzCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);

    /// Path refine function
    void PathRefine(double step);

    /// The IP address of robot
    std::string fci_ip_;

    /// The handler of robot and gripper
    franka::Gripper gripper_;
    franka::Robot robot_;
    /// Realtime robot states
    std::array<double, 7> current_q{};
    std::array<double, 3> current_t{};
    std::array<double, 3> pre_current_t{0.0, 0.0, 0.0};

    /// Flags
    bool new_frame = false;
    bool record_flag = false;

    /// Counter
    int increase_count = 0;
    int threshold = 7;
    int step_count = 0;
    double previous_depth = 0.0;

    /// Path points
    std::vector<std::array<double, 3>> path_points;
};

HandTracker::HandTracker(const std::string& node_name, const rclcpp::NodeOptions& node_options,
            const std::string& fci_ip) : Node(node_name, node_options),
                                         fci_ip_(fci_ip),
                                         gripper_(fci_ip),
                                         robot_(fci_ip)
{
    /// How to check the exception thrown from here? ///

    // Move to ready pose and open the gripper
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot_.control(motion_generator);
    RCLCPP_INFO(this->get_logger(), "Finished moving to initial joint configuration.");

    // Timer
//    timer_  = this->create_wall_timer(std::chrono::milliseconds(100),
//                                      std::bind(&HandTracker::TimerCallback, this));
    // Subscribe to target pose
    relative_xyz_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>
            ("/hand_tracker/array", rclcpp::QoS(1), std::bind(&HandTracker::XyzCallback, this, std::placeholders::_1));

    // ROS parameters declared here
    this->declare_parameter("shut_down", false);

    // The class has been initialized
    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void HandTracker::XyzCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
{
    auto array_data = msg->data;
    if(array_data.size() == 6)
    {
        hand_array.x = array_data[0];
        hand_array.y = array_data[1];
        hand_array.z = array_data[2];
        hand_array.ux = array_data[3];
        hand_array.uy = array_data[4];
        hand_array.flag = array_data[5];
        new_frame = true;
    }
    else
    {
        // When the size of data is not 3
        ;
    }
}

void HandTracker::TimerCallback()
{
    if(hand_array.flag>0 and new_frame)
    {
        if(hand_array.z - previous_depth >= 0.005)
            increase_count ++;
        else
            increase_count = increase_count>1 ? increase_count-1 : 0;
        if(increase_count >= threshold)
        {
            // Save
            step_count ++;
            increase_count = 0;

            if(step_count <= 2)
            {
                RCLCPP_INFO(LOGGER, "Save");
                json_save("/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_demo_bringup/src/demo_hand/joints_data.json",
                          "joints_"+std::to_string(step_count), current_q);
                record_flag = !record_flag;
            }
        }
        previous_depth = hand_array.z;
        if(record_flag)
        {
            if(abs(current_t.at(0)-pre_current_t.at(0))+abs(current_t.at(1)-pre_current_t.at(1)) >= 0.01)
            {
                path_points.push_back(current_t);
                pre_current_t.at(0) = current_t.at(0);
                pre_current_t.at(1) = current_t.at(1);
                pre_current_t.at(2) = current_t.at(2);
            }
        }
        if(step_count == 2)
        {
            step_count ++;
            PathRefine(0.03);
            json_save("/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_demo_bringup/src/demo_hand/joints_data.json",
                      "path", path_points);
        }
        new_frame = false;
        RCLCPP_INFO(LOGGER, "x:%.2f, y:%.2f, z:%.2f, ux:%.2f, uy:%.2f, counter: %d",
                    hand_array.x, hand_array.y, hand_array.z, hand_array.ux, hand_array.uy, increase_count);
    }
    else
    {
        // When hand_array is not updated
        ;
    }
}

void HandTracker::run()
{
    std::array<double, 6> current_vel{};
    std::array<double, 6> current_acc{};

    std::queue<double> bias_vel_x;
    std::queue<double> bias_vel_y;
    std::queue<double> bias_vel_z;
    const double queue_size_max = 5;
    const double i_factor = 0.2;
    for(size_t i=0; i < queue_size_max; i++)
    {
        bias_vel_x.emplace(0.0);
        bias_vel_y.emplace(0.0);
        bias_vel_z.emplace(0.0);
    }
    double compensation_x = 0.0;
    double compensation_y = 0.0;
    double compensation_z = 0.0;

    double time = 0.0;
    double a_max = 1.0;
    double j_max = 500;
    double factor = 0.7;

    robot_.control([=, &time, &current_vel, &current_acc,
                    &bias_vel_x, &bias_vel_y, &bias_vel_z,
                    &compensation_x, &compensation_y, &compensation_z]
    (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities
    {
        double time_seg = period.toSec();
        time += time_seg;

        current_t = {robot_state.O_T_EE.at(12), robot_state.O_T_EE.at(13), robot_state.O_T_EE.at(14)};
        current_q = robot_state.q;
        current_vel = robot_state.O_dP_EE_c;
        current_acc = robot_state.O_ddP_EE_c;

        // Velocity
        double v_x = current_vel.at(0);
        double v_y = current_vel.at(1);
        double v_z = current_vel.at(2);
        double v_goal_x = 0.0, v_goal_y = 0.0, v_goal_z = 0.0;
        if(hand_array.flag > 0)
        {
            v_goal_x = -hand_array.y * factor + compensation_x/queue_size_max;
            v_goal_y = -hand_array.x * factor + compensation_y/queue_size_max;
            v_goal_z = -(hand_array.z-0.2) * factor + compensation_z/queue_size_max;
        }
        compensation_x -= bias_vel_x.front();
        compensation_y -= bias_vel_y.front();
        compensation_z -= bias_vel_z.front();
        bias_vel_x.pop();
        bias_vel_y.pop();
        bias_vel_z.pop();
        bias_vel_x.emplace(v_goal_x*i_factor);
        bias_vel_y.emplace(v_goal_y*i_factor);
        bias_vel_z.emplace(v_goal_z*i_factor);
        compensation_x += v_goal_x*i_factor;
        compensation_y += v_goal_y*i_factor;
        compensation_z += v_goal_z*i_factor;
//        RCLCPP_INFO(this->get_logger(), "The bias: %f, %f, %f", bias_vel_x.back(), bias_vel_y.back(), bias_vel_z.back());
//        RCLCPP_INFO(this->get_logger(), "The goal vel: %f, %f, %f", v_goal_x, v_goal_y, v_goal_z);

        // Velocity refined for controller
        v_x = vel_refine(v_goal_x, v_x, current_acc.at(0), a_max, j_max, time_seg);
        v_y = vel_refine(v_goal_y, v_y, current_acc.at(1), a_max, j_max, time_seg);
        v_z = vel_refine(v_goal_z, v_z, current_acc.at(2), a_max, j_max, time_seg);
        RCLCPP_INFO(this->get_logger(), "The input: %f, %f, %f", v_x, v_y, v_z);
        franka::CartesianVelocities output = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};

        if (stop_flag) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(output);
        }
        return output;
    });
}

void HandTracker::PathRefine(double step)
{
    std::vector<std::array<double, 3>> tmp;
    double record_x, record_y, record_z;
    record_x = path_points[0].at(0);
    record_y = path_points[0].at(1);
    record_z = path_points[0].at(2);
    tmp.push_back(std::array<double, 3>{record_x, record_y, record_z});
    double distance = 0.0, factor = 0.0;
    for(auto point : path_points)
    {
        while(true)
        {
            distance = sqrt(pow(point.at(0)-record_x, 2) + pow(point.at(1)-record_y, 2));
            if(distance < step)
                break;
            else
            {
                factor = step/distance;
                record_x += (point.at(0)-record_x)*factor;
                record_y += (point.at(1)-record_y)*factor;
                tmp.push_back(std::array<double, 3>{record_x, record_y, record_z});
            }
        }
    }
    record_x = path_points.back().at(0);
    record_y = path_points.back().at(1);
    record_z = path_points.back().at(2);
    tmp.push_back(std::array<double, 3>{record_x, record_y, record_z});
    path_points = tmp;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signal_handler);

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
//    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<HandTracker> node = std::make_shared<HandTracker>("hand_follow_node", node_options, "172.16.0.2");

    // Spin the node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    node->run();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
