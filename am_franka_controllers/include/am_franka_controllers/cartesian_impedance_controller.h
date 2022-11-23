//
// Created by armine on 9/24/22.
//

#ifndef BUILD_POSITION_CONTROLLER_H
#define BUILD_POSITION_CONTROLLER_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

#include <std_msgs/msg/bool.hpp>

#include <kdl/frames.hpp>

#include "am_franka_controllers/controller_base.h"

namespace am_franka_controllers{

    class CartesianImpedanceController : public ControllerBase{
    public:
        explicit CartesianImpedanceController(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                                              const std::string& robot_ip);

        void timer_callback();
        void gripper_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
    private:
        Eigen::MatrixXd stiffness_, damping_;

        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;

        double last_force_ext_x = 0.0;
        bool sucked = false;
    };
}

#endif //BUILD_POSITION_CONTROLLER_H
