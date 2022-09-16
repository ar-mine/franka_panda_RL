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
#include <kdl/frames_io.hpp>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

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

        KDL::Tree robot_tree_;
        KDL::Chain robot_chain_;
        KDL::JntArray jnt_pos_, jnt_effort_;
        boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
    };

}

#endif //BUILD_POSITION_SERVO_H
