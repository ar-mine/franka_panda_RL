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
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <kdl/chaindynparam.hpp>
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
        rclcpp::Time start_time_;

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
        KDL::JntArray jnt_pos_, jnt_velocity_, jnt_effort_;
        KDL::JntArray jnt_pos_des_, jnt_vel_delta_;
        KDL::JntArray jnt_pos_error_, jnt_pos_last_error_;
        KDL::Jacobian jacobian_;
        KDL::JntSpaceInertiaMatrix cart_mass_;

        KDL::Twist cart_error_;
        boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
        boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> jnt_to_vel_solver_;
        boost::scoped_ptr<KDL::ChainIkSolverPos_NR> cart_to_jnt_solver_;
        boost::scoped_ptr<KDL::ChainDynParam> jnt_dyn_solver_;

        double speed_factor_ = 0.1;
        KDL::Twist extra_gravity_ = KDL::Twist(KDL::Vector(0, 0, 3.7),   // in m/s
                                               KDL::Vector(0, 0, 0));  // in rad/s

        KDL::Twist cart_vel_max_ = KDL::Twist(KDL::Vector(1.7, 1.7, 1.7),   // in m/s
                                              KDL::Vector(2.5, 2.5, 2.5));  // in rad/s

        Vector7d dq_max_ = (Vector7d() << 2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26).finished();  // in m/s
        Vector7d ddq_max_start_ = (Vector7d() << 10, 10, 10, 10, 10, 10, 10).finished();         // in m/s^2
        Vector7d tau_max_ = (Vector7d() << 87, 87, 87, 87, 12, 12, 12).finished();          // in Nm
    };

}

#endif //BUILD_POSITION_SERVO_H
