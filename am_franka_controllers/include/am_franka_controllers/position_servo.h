//
// Created by armine on 9/15/22.
//

#ifndef BUILD_POSITION_SERVO_H
#define BUILD_POSITION_SERVO_H

#include <string>
#include <Eigen/Eigen>
#include <boost/scoped_ptr.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"

namespace am_franka_controllers{

    class PositionServoController : public controller_interface::ControllerInterface {
        public:
        using Vector3d = Eigen::Matrix<double, 3, 1>;
        using Vector4d = Eigen::Matrix<double, 4, 1>;
        using Vector7d = Eigen::Matrix<double, 7, 1>;

        controller_interface::return_type init(const std::string& controller_name) override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::return_type update() override;
        void updateJointStates();
        void updateFKStates();
        void XyzCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr relative_xyz_sub_;

    private:
        std::string arm_id_;
        const int num_joints = 7;
        Vector7d q_;
        Vector7d q_goal_;
        Vector7d dq_;
        Vector7d dq_filtered_;
        Vector7d eef_;
        Vector3d eef_t_;
        Vector4d eef_r_;
        Vector7d k_gains_;
        Vector7d d_gains_;

        KDL::Frame current_pose_;
        KDL::Frame reference_pose_;
        KDL::Chain robot_chain_;
        KDL::JntArray jnt_pos_, jnt_effort_;
        KDL::Jacobian jacobian_;
        KDL::Twist error_;
        boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

        double factor_ = 0.05;
        Vector7d dq_max_ = (Vector7d() << 2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26).finished();  // in m/s
        Vector7d ddq_max_start_ = (Vector7d() << 10, 10, 10, 10, 10, 10, 10).finished();         // in m/s^2
        Vector7d tau_max_ = (Vector7d() << 87, 87, 87, 87, 12, 12, 12).finished();          // in Nm
    };

}

#endif //BUILD_POSITION_SERVO_H
