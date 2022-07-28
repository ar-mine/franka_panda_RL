/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_cpp_interface_demo.cpp
 *      Project   : moveit2_tutorials
 *      Created   : 07/13/2020
 *      Author    : Adam Pettinger
 */

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
// Servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_tutorials.servo_demo_node.cpp");

// BEGIN_TUTORIAL

// Setup
// ^^^^^
// First we declare pointers to the node and publisher that will publish commands to Servo
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr relative_xyz_sub_;
size_t count_ = 0;
struct Xyz {
    float x, y, z;
};
Xyz hand_array = {-1, -1, -1};
bool new_frame = false;
// BEGIN_SUB_TUTORIAL publishCommands
// Here is the timer callback for publishing commands. The C++ interface sends commands through internal ROS topics,
// just like if Servo was launched using ServoServer.
void publishCommands()
{
    if(hand_array.z > 0 and new_frame)
    {
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg->header.stamp = node_->now();
        msg->header.frame_id = "panda_hand_tcp";
        msg->twist.linear.x = hand_array.x;
        msg->twist.linear.y = -hand_array.y;
        msg->twist.linear.z = 0.0;
        twist_cmd_pub_->publish(std::move(msg));

        new_frame = false;
    }
    else
    {
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg->header.stamp = node_->now();
        msg->header.frame_id = "panda_hand_tcp";
        msg->twist.linear.x = 0;
        msg->twist.linear.y = 0;
        msg->twist.linear.z = 0.0;
        twist_cmd_pub_->publish(std::move(msg));
    }
}
void XyzCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
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
// END_SUB_TUTORIAL

// Next we will set up the node, planning_scene_monitor, and collision object
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(true);
    node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

    // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
    rclcpp::sleep_for(std::chrono::seconds(4));

    // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
    // before initializing any collision objects
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            node_, "robot_description", tf_buffer, "planning_scene_monitor");

    // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
    if (planning_scene_monitor->getPlanningScene())
    {
        planning_scene_monitor->startStateMonitor("/joint_states");
        planning_scene_monitor->setPlanningScenePublishingFrequency(25);
        planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                             "/moveit_servo/publish_planning_scene");
        planning_scene_monitor->startSceneMonitor();
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning scene not configured");
        return EXIT_FAILURE;
    }

    // These are the publishers that will send commands to MoveIt Servo. Two command types are supported: JointJog
    // messages which will directly jog the robot in the joint space, and TwistStamped messages which will move the
    // specified link with the commanded Cartesian velocity. In this demo, we jog the end effector link.
    joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("~/delta_joint_cmds", 10);
    twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("~/delta_twist_cmds", 10);
    relative_xyz_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/hand_tracker/array", rclcpp::QoS(1), XyzCallback);

    // Initializing Servo
    // ^^^^^^^^^^^^^^^^^^
    // Servo requires a number of parameters to dictate its behavoir. These can be read automatically by using the
    // :code:`makeServoParameters` helper function
    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_, LOGGER);
    if (!servo_parameters)
    {
        RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
        return EXIT_FAILURE;
    }

    // Initialize the Servo C++ interface by passing a pointer to the node, the parameters, and the PSM
    auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);

    // You can start Servo directly using the C++ interface. If launched using the alternative ServoServer, a ROS service
    // is used to start Servo. Before it is started, MoveIt Servo will not accept any commands or move the robot
    servo->start();

    // Sending Commands
    // ^^^^^^^^^^^^^^^^
    // For this demo, we will use a simple ROS timer to send joint and twist commands to the robot
    rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(100ms, publishCommands);

    // CALL_SUB_TUTORIAL publishCommands

    // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
    auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_);
    executor->spin();

    // END_TUTORIAL

    rclcpp::shutdown();
    return 0;
}
