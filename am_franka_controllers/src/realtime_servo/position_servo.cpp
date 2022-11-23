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
            config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
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
        jnt_to_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(robot_chain_));
        cart_to_jnt_solver_.reset(new KDL::ChainIkSolverPos_NR(robot_chain_, *jnt_to_pose_solver_, *jnt_to_vel_solver_));
        jnt_dyn_solver_.reset(new KDL::ChainDynParam(robot_chain_, KDL::Vector(0, 0, -9.81)));

        // Resizes the joint state, joint effort and jacobian array vectors in non-realtime
        jnt_pos_.resize(robot_chain_.getNrOfJoints());
        jnt_pos_des_.resize(robot_chain_.getNrOfJoints());
        jnt_pos_error_.resize(robot_chain_.getNrOfJoints());
        jnt_pos_last_error_.resize(robot_chain_.getNrOfJoints());
        jnt_velocity_.resize(robot_chain_.getNrOfJoints());
        jnt_vel_delta_.resize(robot_chain_.getNrOfJoints());
        jnt_effort_.resize(robot_chain_.getNrOfJoints());
        jacobian_.resize(robot_chain_.getNrOfJoints());
        cart_mass_.resize(robot_chain_.getNrOfJoints());

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
        // Set scaled velocity limitation
        cart_vel_max_ = cart_vel_max_ * speed_factor_;

        updateJointStates();

        jnt_pos_des_ = jnt_pos_;
        reference_pose_ = current_pose_;

        start_time_ = this->node_->now();
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

    controller_interface::return_type PositionServoController::update() {
        /**
         * Implement the update method as the main entry point.
         * The method should be implemented with real-time constraints in mind.
         * When this method is called, the state interfaces have the most recent values from the hardware,
         * and new commands for the hardware should be written into command interfaces.
         * */
        updateJointStates();

        for (unsigned int i = 0; i < jnt_pos_.rows(); i++)
        {
            jnt_pos_error_(i) = k_gains_(i) * (jnt_pos_des_(i)-jnt_pos_(i));
        }
//        RCLCPP_INFO(LOGGER, "Cart Error: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                    jnt_pos_error_(0), jnt_pos_error_(1), jnt_pos_error_(2), jnt_pos_error_(3),
//                    jnt_pos_error_(4), jnt_pos_error_(5), jnt_pos_error_(6));
//        RCLCPP_INFO(LOGGER, "Cart Error: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                    jnt_pos_des_(0)-jnt_pos_(0), jnt_pos_des_(1)-jnt_pos_(1), jnt_pos_des_(2)-jnt_pos_(2),
//                    jnt_pos_des_(3)-jnt_pos_(3),
//                    jnt_pos_des_(4)-jnt_pos_(4), jnt_pos_des_(5)-jnt_pos_(5), jnt_pos_des_(6)-jnt_pos_(6));
        for (unsigned int i = 0; i < jnt_pos_.rows(); i++)
        {
            jnt_effort_(i) = 0;
            for (unsigned int j = 0; j < jnt_pos_.rows(); j++) {
                jnt_effort_(i) += cart_mass_(i, j) * jnt_pos_error_(j);
            }
            RCLCPP_INFO(LOGGER, "Cart Error: %f %f %f %f %f %f %f",
                        cart_mass_(i,0), cart_mass_(i,1), cart_mass_(i,2), cart_mass_(i,3),
                        cart_mass_(i,4), cart_mass_(i,5), cart_mass_(i,6));
        }
//        RCLCPP_INFO(LOGGER, "Cart Error: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                    jnt_effort_(0), jnt_effort_(1), jnt_effort_(2), jnt_effort_(3),
//                    jnt_effort_(4), jnt_effort_(5), jnt_effort_(6));
        for (unsigned int i = 0; i < jnt_pos_.rows(); i++)
        {
            for (unsigned int j = 0; j < 6; j++) {
                jnt_effort_(i) += jacobian_(j, i) * extra_gravity_(j);
            }
        }
        // Update time
//        auto dt = this->node_->now() - start_time_;
//        start_time_ = this->node_->now();
//
//        // Get the pose error
//        cart_error_ = KDL::diff(current_pose_, reference_pose_);
//        // Print Cartesian Error

//        double factor = 1;
//        for(unsigned int i = 0; i < 6; i++)
//            if(cart_error_(i)/0.001 > factor)
//            {
//                factor = cart_error_(i)/0.001;
//            }
//        cart_error_ = cart_error_ / factor;
//        KDL::Frame des_pose_ = current_pose_;
//        KDL::addDelta(des_pose_, cart_error_);
//
//        cart_to_jnt_solver_->CartToJnt(jnt_pos_, reference_pose_, jnt_pos_des_);

//
//        // Regard Cartesian error as twist
//        // q_dot = Jac^-1 * x_dot
//        cart_error_(2) *= 1.5;
//        jnt_to_vel_solver_->CartToJnt(jnt_pos_, cart_error_* 100, jnt_vel_delta_);
//                RCLCPP_INFO(LOGGER, "Cart Error: %.2f %.2f %.2f %.2f %.2f %.2f",
//                            jnt_vel_delta_(0), jnt_vel_delta_(1), jnt_vel_delta_(2), jnt_vel_delta_(3),
//                            jnt_vel_delta_(4), jnt_vel_delta_(5));
//        // q_des = q + q_dot*delta_t
//        for (unsigned int i = 0; i < jnt_pos_.rows(); i++)
//        {
//            jnt_pos_des_(i) = jnt_pos_(i) + jnt_vel_delta_(i)*dt.seconds();
//        }
//        double factor = 0;
//        for(int i=0; i<6; i++)
//        {
//            double temp = error_(i) / max_twist(i);
//            if(temp>factor)
//                factor = temp;
//        }
//        if(factor>1)
//            error_ = error_ / factor;
//            RCLCPP_INFO(LOGGER, "Current: %.2f %.2f %.2f",
//                        current_pose_.p.data[0], current_pose_.p.data[1], current_pose_.p.data[2]);
//            RCLCPP_INFO(LOGGER, "Reference_pose_: %.2f %.2f %.2f",
//                        reference_pose_.p.data[0], reference_pose_.p.data[1], reference_pose_.p.data[2]);
//        RCLCPP_INFO(LOGGER, "Error: %.2f %.2f %.2f %.2f %.2f %.2f",
//                    error_(0), error_(1), error_(2), error_(3), error_(4), error_(5));


        // Compute Jacobian in realtime
//        jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);
        // jnt_velocity = Kp * Jac^-1 * twist(pose error)
        // jnt_effort = Jac^transpose * cart_wrench

        /** Position Feedback Control **/
//        for (unsigned int i = 0; i < jnt_pos_.rows(); i++)
//        {
//            jnt_pos_error_(i) = jnt_pos_des_(i) - jnt_pos_(i);
//            // tau = kp * (q_des - q)
//            jnt_effort_(i) = k_gains_(i) * jnt_pos_error_(i);
//            // Save the error
//            jnt_pos_last_error_(i) = jnt_pos_error_(i);
//            // Hand the joint limitations
//            if(std::abs(jnt_effort_(i))>tau_max_[i])
//                jnt_effort_(i) = tau_max_[i];

//        }
        // Print Joint Error
//        RCLCPP_INFO(LOGGER, "Joint Error: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                    jnt_pos_error_(0), jnt_pos_error_(1), jnt_pos_error_(2), jnt_pos_error_(3),
//                    jnt_pos_error_(4), jnt_pos_error_(5), jnt_pos_error_(6));

        /** Position Feedback Control **/

        // Deploy the control force
        for (int i = 0; i < 7; ++i) {
            command_interfaces_[i].set_value(jnt_effort_(i));
        }
//            RCLCPP_INFO(LOGGER, "Effort: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                        jnt_effort_(0), jnt_effort_(1), jnt_effort_(2), jnt_effort_(3), jnt_effort_(4), jnt_effort_(5), jnt_effort_(6));
//        rclcpp::sleep_for(std::chrono::milliseconds(20));
        return controller_interface::return_type::OK;
    }

    void PositionServoController::updateJointStates() {
        for (auto i = 0; i < num_joints; ++i) {
            const auto& position_interface = state_interfaces_.at(2 * i);
            const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

            assert(position_interface.get_interface_name() == "position");
            assert(velocity_interface.get_interface_name() == "velocity");

            jnt_pos_(i) = position_interface.get_value();
            jnt_velocity_(i) = velocity_interface.get_value();

            // Output current q value
//            RCLCPP_INFO(LOGGER, "Current q: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                       q_[0], q_[1], q_[2], q_[3], q_[4], q_[5], q_[6]);
            // Output current q_dot value
//            RCLCPP_INFO(LOGGER, "Current q: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                       q_[0], q_[1], q_[2], q_[3], q_[4], q_[5], q_[6]);
        }

        // Computes Cartesian pose and Jacobain matrix
        jnt_to_pose_solver_->JntToCart(jnt_pos_, current_pose_);
        jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);
        jnt_dyn_solver_->JntToMass(jnt_pos_, cart_mass_);
    }

    void PositionServoController::updateFKStates()
    {


        eef_t_ = {current_pose_.p.data[0], current_pose_.p.data[1], current_pose_.p.data[2]};
        current_pose_.M.GetQuaternion(eef_r_[0], eef_r_[1], eef_r_[2], eef_r_[3]);
//        RCLCPP_INFO(LOGGER, "EEF translation: %.2f %.2f %.2f, rotation: %.2f %.2f %.2f %.2f",
//                eef_t_[0], eef_t_[1], eef_t_[2],
//                eef_r_[0], eef_r_[1], eef_r_[2], eef_r_[3]);

    }

    void PositionServoController::XyzCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
    {
        auto array_data = msg->data;
//        reference_pose_.p(0) = current_pose_.p(0) - array_data[0];
//        reference_pose_.p(1) = current_pose_.p(1) - array_data[1];
    }
}

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(am_franka_controllers::PositionServoController,
        controller_interface::ControllerInterface)
