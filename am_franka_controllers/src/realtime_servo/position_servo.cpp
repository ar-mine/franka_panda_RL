//
// Created by armine on 9/15/22.
//
#include "am_franka_controllers/position_servo.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("am_franka_controllers/position_servo.cpp");

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
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (int i = 1; i <= num_joints; ++i) {
            config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
            config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
        }
        return config;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    PositionServoController::on_configure(const rclcpp_lifecycle::State& previous_state)
    {
        // Parameters are usually read here, and everything is prepared so that the controller can be started.
        std::string robot_desc_string;
        robot_desc_string = node_->get_parameter("robot_description").as_string();
        if (!kdl_parser::treeFromString(robot_desc_string, robot_tree_)){
            fprintf(stderr, "Failed to construct kdl tree\n");
            return CallbackReturn::FAILURE;
        }

        robot_tree_.getChain("panda_link0", "panda_hand_tcp", robot_chain_);
        // Create solver based on kinematic chain
        jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
        // resizes the joint state vectors in non-realtime
        jnt_pos_.resize(robot_chain_.getNrOfJoints());
//        RCLCPP_INFO(LOGGER, "Total number of joints: %d", robot_chain_.getNrOfJoints());

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
        RCLCPP_INFO(LOGGER, "Now running on_activate.");
        updateJointStates();

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
//        RCLCPP_INFO(LOGGER, "Now running update.");
        updateJointStates();
        updateFKStates();

        for (auto& command_interface : command_interfaces_) {
            command_interface.set_value(0);
        }
        return controller_interface::return_type::OK;
    }

    void PositionServoController::updateJointStates() {
        for (auto i = 0; i < num_joints; ++i) {
            const auto& position_interface = state_interfaces_.at(2 * i);
            const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

            assert(position_interface.get_interface_name() == "position");
            assert(velocity_interface.get_interface_name() == "velocity");

            q_(i) = position_interface.get_value();
            dq_(i) = velocity_interface.get_value();
//            RCLCPP_INFO(LOGGER, "Current q: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                       q_[0], q_[1], q_[2], q_[3], q_[4], q_[5], q_[6]);
        }
    }

    void PositionServoController::updateFKStates()
    {
        // Assign some values to the joint positions
        for(unsigned int i=0;i<7;i++){
            jnt_pos_(i) = q_[i];
        }
        // computes Cartesian pose in realtime
        KDL::Frame current_pose;
        jnt_to_pose_solver_->JntToCart(jnt_pos_, current_pose);

        eef_t_ = {current_pose.p.data[0], current_pose.p.data[1], current_pose.p.data[2]};
        current_pose.M.GetQuaternion(eef_r_[0], eef_r_[1], eef_r_[2], eef_r_[3]);
//        RCLCPP_INFO(LOGGER, "EEF translation: %.2f %.2f %.2f, rotation: %.2f %.2f %.2f %.2f",
//                eef_t_[0], eef_t_[1], eef_t_[2],
//                eef_r_[0], eef_r_[1], eef_r_[2], eef_r_[3]);

    }
}

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(am_franka_controllers::PositionServoController,
        controller_interface::ControllerInterface)
