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

        // Subscribe to target pose
        for(int i = 0; i < 6;i++)
            error_(i) = 0.0;
        relative_xyz_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>
                ("/hand_tracker/array", rclcpp::QoS(1), std::bind(&PositionServoController::XyzCallback, this, std::placeholders::_1));

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
        arm_id_ = node_->get_parameter("arm_id").as_string();
        auto k_gains = node_->get_parameter("k_gains").as_double_array();
        auto d_gains = node_->get_parameter("d_gains").as_double_array();
        if (k_gains.empty()) {
            RCLCPP_FATAL(node_->get_logger(), "k_gains parameter not set");
            return CallbackReturn::FAILURE;
        }
        if (k_gains.size() != static_cast<uint>(num_joints)) {
            RCLCPP_FATAL(node_->get_logger(), "k_gains should be of size %d but is of size %d", num_joints,
                         k_gains.size());
            return CallbackReturn::FAILURE;
        }
        if (d_gains.empty()) {
            RCLCPP_FATAL(node_->get_logger(), "d_gains parameter not set");
            return CallbackReturn::FAILURE;
        }
        if (d_gains.size() != static_cast<uint>(num_joints)) {
            RCLCPP_FATAL(node_->get_logger(), "d_gains should be of size %d but is of size %d", num_joints,
                         d_gains.size());
            return CallbackReturn::FAILURE;
        }
        for (int i = 0; i < num_joints; ++i) {
            d_gains_(i) = d_gains.at(i);
            k_gains_(i) = k_gains.at(i);
        }
        dq_filtered_.setZero();

        // Parse description to KDL tree
        KDL::Tree robot_tree;
        std::string robot_desc_string;
        robot_desc_string = node_->get_parameter("robot_description").as_string();
        if (!kdl_parser::treeFromString(robot_desc_string, robot_tree)){
            fprintf(stderr, "Failed to construct kdl tree\n");
            return CallbackReturn::FAILURE;
        }
        // Convert KDL tree to KDL chain
        robot_tree.getChain("panda_link0", "panda_hand_tcp", robot_chain_);
        // Create solver based on kinematic chain
        jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));
        // resizes the joint state, joint effort and jacobian array vectors in non-realtime
        jnt_pos_.resize(robot_chain_.getNrOfJoints());
        jnt_effort_.resize(robot_chain_.getNrOfJoints());
        jacobian_.resize(robot_chain_.getNrOfJoints());
//        RCLCPP_INFO(LOGGER, "Total number of joints: %d", robot_chain_.getNrOfJoints());

        // sets the desired pose
//        reference_pose_ = KDL::Frame(KDL::Rotation::Quaternion(1, 0, 0, 0),KDL::Vector(0.5, 0, 0.5));

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
        updateFKStates();
        reference_pose_ = current_pose_;
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
        updateJointStates();
        updateFKStates();

        bool finished = KDL::Equal(current_pose_, reference_pose_, 1e-6);
        if(not finished)
        {
            // get the pose error
            error_ = KDL::diff(current_pose_, reference_pose_);
//            RCLCPP_INFO(LOGGER, "Current: %.2f %.2f %.2f",
//                        current_pose_.p.data[0], current_pose_.p.data[1], current_pose_.p.data[2]);
//            RCLCPP_INFO(LOGGER, "Reference_pose_: %.2f %.2f %.2f",
//                        reference_pose_.p.data[0], reference_pose_.p.data[1], reference_pose_.p.data[2]);
//            RCLCPP_INFO(LOGGER, "Error: %.2f %.2f %.2f %.2f %.2f %.2f",
//                        error_(0), error_(1), error_(2), error_(3), error_(4), error_(5));
            finished=true;
            // compute Jacobian in realtime
            jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);
            // jnt_effort = Jac^transpose * cart_wrench
            for (unsigned int i = 0; i < jnt_pos_.rows(); i++)
            {

                jnt_effort_(i) = 0;
                for (unsigned int j=0; j<6; j++)
                {
                    jnt_effort_(i) += (jacobian_(j,i) * 50 * error_(j));
                }
                if(std::abs(jnt_effort_(i))>tau_max_[i])
                    jnt_effort_(i) = tau_max_[i];
//                if(std::abs(dq_[i])>dq_max_[i]*factor_)
//                    jnt_effort_(i) = 0;
            }

            for (int i = 0; i < 7; ++i) {
                command_interfaces_[i].set_value(jnt_effort_(i));
            }
//            RCLCPP_INFO(LOGGER, "Effort: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                        jnt_effort_(0), jnt_effort_(1), jnt_effort_(2), jnt_effort_(3), jnt_effort_(4), jnt_effort_(5), jnt_effort_(6));
        }
        else{
            for (auto& command_interface : command_interfaces_) {
                command_interface.set_value(0);
            }
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
        jnt_to_pose_solver_->JntToCart(jnt_pos_, current_pose_);
        jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);

        eef_t_ = {current_pose_.p.data[0], current_pose_.p.data[1], current_pose_.p.data[2]};
        current_pose_.M.GetQuaternion(eef_r_[0], eef_r_[1], eef_r_[2], eef_r_[3]);
//        RCLCPP_INFO(LOGGER, "EEF translation: %.2f %.2f %.2f, rotation: %.2f %.2f %.2f %.2f",
//                eef_t_[0], eef_t_[1], eef_t_[2],
//                eef_r_[0], eef_r_[1], eef_r_[2], eef_r_[3]);

    }

    void PositionServoController::XyzCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
    {
//        auto array_data = msg->data;
//        error_(0) = array_data[0];
//        error_(1) = array_data[1];
    }
}

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(am_franka_controllers::PositionServoController,
        controller_interface::ControllerInterface)
