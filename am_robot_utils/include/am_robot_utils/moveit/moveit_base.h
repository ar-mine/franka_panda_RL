//
// Created by armine on 6/9/22.
//

#ifndef BUILD_MOVEIT_BASE_H
#define BUILD_MOVEIT_BASE_H

#include "std_msgs/msg/float64_multi_array.hpp"

#include "am_robot_utils/base.h"
#include "am_robot_utils/geometry/geometry_base.h"
#include "am_robot_utils/moveit/moveit_utils.h"
#include "franka_msgs/action/grasp.hpp"

const std::string MOVE_GROUP = "panda_manipulator";
const std::string EEF_LINK = "panda_hand_tcp";

namespace Am {
    class MoveitBase : public rclcpp::Node {
    public:
        /// Constructor
        MoveitBase(const std::string &node_name, const rclcpp::NodeOptions &node_options, int mode,
                   double max_vel=0.15, double max_acc=0.15);

        /// Move group interface and planning scene interface for the robot
        moveit::planning_interface::MoveGroupInterface move_group_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

        /*********************** Control mode 1(topic)  ********************************/
        /// Subscriber for target pose and target joint states
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_joints_sub_;
        /// Target pose and joint states that is used to detect changes
        geometry_msgs::msg::Pose previous_target_pose_;
        std::vector<double> previous_target_joints_;

        /// Callback that plans and executes trajectory each time the target pose is changed
        void target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

        void target_joints_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

        /*********************** Control mode 2(function)  ********************************/
        bool moveit_move_ready(const rclcpp::Logger &LOGGER);

        void hand_action(bool open_close, const rclcpp::Logger &LOGGER, double target_width=0.0);


    private:
        using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>;

        void result_callback(const GoalHandleGrasp::WrappedResult &result);

        rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr grasp_client_ptr_;
        bool finish_flag = false;
    };
}
#endif //BUILD_MOVEIT_BASE_H
