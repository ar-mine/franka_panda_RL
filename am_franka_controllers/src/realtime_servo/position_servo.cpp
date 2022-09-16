//
// Created by armine on 9/15/22.
//
#include "am_franka_controllers/position_servo.h"

namespace am_franka_controllers{
    controller_interface::return_type PositionServoController::init(const std::string& controller_name)
    {
        // best place to initialize the variables, reserve memory,
        // and most importantly, declare node parameters used by the controller.
        auto ret = ControllerInterface::init(controller_name);
        if (ret != controller_interface::return_type::OK) {
            return ret;
        }

        try {
            auto_declare<std::string>("arm_id", "panda");
        } catch (const std::exception& e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::return_type::ERROR;
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::InterfaceConfiguration PositionServoController::command_interface_configuration() const
    {
        /**
         *  ALL and NONE option will ask for access to all available interfaces or none of them.
         *  The INDIVIDUAL configuration needs a detailed list of required interface names.
         * */
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (int i = 1; i <= num_joints; ++i) {
            config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
        }
        return config;
    }

    controller_interface::InterfaceConfiguration PositionServoController::state_interface_configuration() const
    {
        return {};
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    PositionServoController::on_configure(const rclcpp_lifecycle::State& previous_state)
    {
        // Parameters are usually read here, and everything is prepared so that the controller can be started.
        std::string robot_desc_string;
        robot_desc_string = node_->get_parameter("robot_description").as_string();
        if (!kdl_parser::treeFromString(robot_desc_string, robot_tree)){
            fprintf(stderr, "Failed to construct kdl tree\n");
            return CallbackReturn::FAILURE;
        }

        robot_tree.getChain("panda_link0", "panda_hand_tcp", robot_chain);

        arm_id_ = node_->get_parameter("arm_id").as_string();
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    PositionServoController::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        /**
         * Implement the on_activate method with checking, and potentially sorting,
         * the interfaces and assigning members’ initial values.
         * This method is part of the real-time loop, therefore avoid any reservation of memory and,
         * in general, keep it as short as possible.
         * */
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    PositionServoController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        /**
         * Implement the on_deactivate method, which does the opposite of on_activate.
         * In many cases, this method is empty.
         * This method should also be real-time safe as much as possible.*/
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type PositionServoController::update()
    {
        /**
         * Implement the update method as the main entry point.
         * The method should be implemented with real-time constraints in mind.
         * When this method is called, the state interfaces have the most recent values from the hardware,
         * and new commands for the hardware should be written into command interfaces.
         * */
        for (auto& command_interface : command_interfaces_) {
            command_interface.set_value(0);
        }
        return controller_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(am_franka_controllers::PositionServoController,
        controller_interface::ControllerInterface)