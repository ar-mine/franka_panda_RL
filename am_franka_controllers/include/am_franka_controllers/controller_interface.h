//
// Created by armine on 9/24/22.
//

#ifndef BUILD_POSITION_CONTROLLER_H
#define BUILD_POSITION_CONTROLLER_H

#include <kdl/frames.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/bool.hpp>

#include "am_franka_controllers/cart_vel_controller.h"
#include "franka_action_interface/action/cart_pose_set.hpp"

namespace am_franka_controllers{

    class ControllerInterface : public CartVelController{
    public:
        using CartPoseSet = franka_action_interface::action::CartPoseSet;
        using GoalHandleCartPoseSet = rclcpp_action::ServerGoalHandle<CartPoseSet>;

        explicit ControllerInterface(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                                    const std::string& robot_ip);

        void timer_callback();
        void target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
        void gripper_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;

    private:
        bool target_changed = false;

        rclcpp_action::Server<CartPoseSet>::SharedPtr cart_pose_server_;
        rclcpp_action::GoalResponse cart_pose_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CartPoseSet::Goal> goal);
        rclcpp_action::CancelResponse cart_pose_cancel(const std::shared_ptr<GoalHandleCartPoseSet> goal_handle);
        void cart_pose_accepted(const std::shared_ptr<GoalHandleCartPoseSet> goal_handle);
        void execute(const std::shared_ptr<GoalHandleCartPoseSet> goal_handle);

        KDL::Frame kdl_cart_pos_des_;
        KDL::Frame kdl_cart_pos_current;

        Eigen::Matrix<double, 6, 1> vec_cart_error_last;

        int control_freq = 50; // Unit: hz
        double transl_vel_limit = 0.2; // Unit: m/s
        double rot_vel_limit = 0.4; // Unit: rad/s
        Eigen::Matrix<double, 6, 1> twist_limit = (Eigen::Matrix<double, 6, 1>() << transl_vel_limit/control_freq, transl_vel_limit/control_freq, transl_vel_limit/control_freq,
                                                                                    rot_vel_limit/control_freq, rot_vel_limit/control_freq, rot_vel_limit/control_freq).finished();

        KDL::Vector kdl_frame_upper_limit{0.6, 0.3, 0.5};
        KDL::Vector kdl_frame_lower_limit{0.1, -0.3, -0.1};
    };

    void tr2kdlFrame(const std::array<double, 16> &cart_pos, KDL::Frame &kdl_cart_pos);
    std::array<double, 4> kdlFrame2quat(KDL::Frame &kdl_cart_pos);
    void twist_scale(KDL::Twist& src_kdl_twist, KDL::Twist& limit_kdl_twist, double eps=0);
    bool cmd_scale(Eigen::Matrix<double, 6, 1>& cmd_src, Eigen::Matrix<double, 6, 1>& cmd_limit, double eps=5e-3);

}

#endif //BUILD_POSITION_CONTROLLER_H
