//
// Created by armine on 9/24/22.
//

#include "franka/model.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <franka/exception.h>
#include "am_franka_controllers/cartesian_impedance_controller.h"

namespace am_franka_controllers{
    using namespace std::placeholders;
    CartesianImpedanceController::CartesianImpedanceController(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                                                               const std::string& robot_ip) : ControllerBase(node_name, node_options, robot_ip, ControllerType::JntTorque)
    {
        // Compliance parameters
        const double translational_stiffness{100.0};
        const double rotational_stiffness{10.0};
        Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
        damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
        stiffness_ = stiffness, damping_ = damping;

        // Set timer as signal generator
        timer_  = this->create_wall_timer(std::chrono::milliseconds(10),
                                          std::bind(&CartesianImpedanceController::timer_callback, this));
        // Set gripper subscription
        gripper_sub_ = this->create_subscription<std_msgs::msg::Bool>("~/gripper",
                                                                      rclcpp::QoS(1), std::bind(&CartesianImpedanceController::gripper_callback, this, _1));

        // Delay to keep the system stable
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Equilibrium point is the initial position
        read();

        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(current_state_.O_T_EE.data()));
        Eigen::Vector3d position_d(initial_transform.translation());
        Eigen::Quaterniond orientation_d(initial_transform.linear());
        position_d_ = position_d, orientation_d_ = orientation_d;
    }

    void CartesianImpedanceController::timer_callback() {
        // Get current states
        read();

        // get state variables
        std::array<double, 7> coriolis_array = model_->coriolis(current_state_);
        std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, current_state_);

        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(current_state_.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(current_state_.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(current_state_.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());
        // compute error to desired equilibrium pose
        // position error
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d_;
        // orientation error
        // "difference" quaternion
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.linear() * error.tail(3);
        // compute control
        Eigen::VectorXd tau_task(7), tau_d(7), force_ext(6);
        // Spring damper system with damping ratio=1
        force_ext = -stiffness_ * error - damping_ * (jacobian * dq);
        tau_task << jacobian.transpose() * force_ext;
        tau_d << tau_task + coriolis;
        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

        if(last_force_ext_x == 0.0)
            last_force_ext_x = force_ext[0];
        if(force_ext[0] - last_force_ext_x >= 0.1)
        {
            RCLCPP_INFO(this->get_logger(), "Force detected!");
//            auto vacuum_gripper_state = vacuum_gripper_->readOnce();
//            if (!vacuum_gripper_state.in_control_range) {
//                sucked = true;
//                std::cout << "Object lost." << std::endl;
//            }

            if(not sucked)
            {
                sucked = true;
                std::thread([this]() {
                    try
                    {
                        vacuum_gripper_->vacuum(100, std::chrono::milliseconds(1000));
                        RCLCPP_INFO(this->get_logger(), "Try to suck object");
                    }
                    catch (franka::Exception const& e)
                    {
                        vacuum_gripper_->stop();
                        RCLCPP_FATAL_STREAM(this->get_logger(), e.what());
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(5000));
                    try
                    {
                        vacuum_gripper_->dropOff(std::chrono::milliseconds(1000));
                        RCLCPP_INFO(this->get_logger(), "Try to drop out object");
                    }
                    catch (franka::Exception const& e)
                    {
                        vacuum_gripper_->stop();
                        RCLCPP_FATAL_STREAM(this->get_logger(), e.what());
                    }
                    sucked = false;
                }).detach();
            }
        }
        last_force_ext_x = force_ext[0];

//        RCLCPP_INFO_STREAM(this->get_logger(), force_ext);
        jntWrite(tau_d_array);
    }

    void CartesianImpedanceController::gripper_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        try
        {
            if(msg->data)
            {
                vacuum_gripper_->vacuum(100, std::chrono::milliseconds(1000));
                RCLCPP_INFO(this->get_logger(), "Try to suck object");
            }
            else
            {
//                gripper_->stop();
                vacuum_gripper_->dropOff(std::chrono::milliseconds(1000));
                RCLCPP_INFO(this->get_logger(), "Try to drop out object");
            }
        }
        catch (franka::Exception const& e)
        {
            vacuum_gripper_->stop();
            RCLCPP_FATAL_STREAM(this->get_logger(), e.what());
        }
        catch (...)
        {
            vacuum_gripper_->stop();
            RCLCPP_FATAL_STREAM(this->get_logger(), "Detect exception");
        }
    }
}



