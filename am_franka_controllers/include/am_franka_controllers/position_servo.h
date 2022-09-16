//
// Created by armine on 9/15/22.
//

#ifndef BUILD_POSITION_SERVO_H
#define BUILD_POSITION_SERVO_H

#include <string>
#include <kdl_parser/kdl_parser.hpp>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace am_franka_controllers{

    class PositionServoController : public controller_interface::ControllerInterface {
        public:
        controller_interface::return_type init(const std::string& controller_name) override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::return_type update() override;

        private:
        std::string arm_id_;
        const int num_joints = 7;
        KDL::Tree robot_tree;
        KDL::Chain robot_chain;
    };

}

#endif //BUILD_POSITION_SERVO_H
