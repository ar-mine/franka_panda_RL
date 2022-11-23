//
// Created by armine on 9/24/22.
//

#include <rclcpp/logging.hpp>

#include <franka/control_tools.h>
#include "am_franka_controllers/controller_base.h"

namespace am_franka_controllers {
    ControllerBase::ControllerBase(const std::string &node_name, const rclcpp::NodeOptions &node_options,
                                   const std::string &robot_ip, const ControllerType &controller_type)
                                   : Node(node_name, node_options), controller_type_(controller_type) {
        initialize(robot_ip, this->get_logger());

        // Initialize controller loop
        if(controller_type_ == JntTorque)
            initializeTorqueControl();
        else if(controller_type_ == JntVelocity)
            ;
        else if(controller_type_ == JntPosition)
            ;
        else if(controller_type_ == CartVelocity)
            ;
        else if(controller_type_ == CartPose)
            initializeCartPoseControl();

    }

    void ControllerBase::initialize(const std::string &robot_ip, const rclcpp::Logger &logger) {
        // Initialize command variable
        if(controller_type_ <= 2)
            jnt_command_.fill(0.);
        else if(controller_type_ == 3)
            cart_vel_command_.fill(0.);
        else if(controller_type_ == 4)
            cart_pose_command_.fill(0.);
        else
            RCLCPP_ERROR(this->get_logger(), "Not supported controller type, please check again!");

            // Check realtime kernel
        franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
        if (not franka::hasRealtimeKernel()) {
            rt_config = franka::RealtimeConfig::kIgnore;
            RCLCPP_WARN(logger, "You are not using a real-time kernel. Using a real-time kernel is strongly recommended!");
        }

        // Initialize robot and its model
        robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
        // collision thresholds are set to high values, make sure no collision will happen
        robot_->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        model_ = std::make_unique<franka::Model>(robot_->loadModel());
    }

    void ControllerBase::jntWrite(const std::array<double, 7> &jnt_command) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        jnt_command_ = jnt_command;
    }

    void ControllerBase::cartVelWrite(const std::array<double, 6> &cart_vel_command) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        cart_vel_command_ = cart_vel_command;
    }

    void ControllerBase::cartPoseWrite(const std::array<double, 16> &cart_pose_command) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        cart_pose_command_ = cart_pose_command;
    }

    franka::RobotState ControllerBase::read() {
        std::lock_guard<std::mutex> lock(read_mutex_);
        return {current_state_};
    }

    void ControllerBase::stopRobot() {
        if (not stopped_) {
            finish_ = true;
            control_thread_->join();
            finish_ = false;
            stopped_ = true;
            vacuum_gripper_->stop();
        }
    }

    void ControllerBase::initializeTorqueControl() {
        assert(isStopped());
        stopped_ = false;
        const auto kTorqueControl = [this]() {
            robot_->control(
                    [this](const franka::RobotState& state, const franka::Duration& /*period*/) {
                        {
                            std::lock_guard<std::mutex> lock(read_mutex_);
                            current_state_ = state;
                        }
                        std::lock_guard<std::mutex> lock(write_mutex_);
                        franka::Torques out(jnt_command_);
                        out.motion_finished = finish_;
                        return out;
                    },
                    true, franka::kMaxCutoffFrequency);
        };
        control_thread_ = std::make_unique<std::thread>(kTorqueControl);
    }

    void ControllerBase::initializeCartPoseControl() {
        assert(isStopped());
        stopped_ = false;
        const auto kCartPoseControl = [this]() {
            robot_->control(
                [this](const franka::RobotState& state, const franka::Duration& period) {
                    {
                        std::lock_guard<std::mutex> lock(read_mutex_);
                        current_state_ = state;
                    }
                    std::lock_guard<std::mutex> lock(write_mutex_);
//                        RCLCPP_INFO_STREAM(this->get_logger(), Eigen::Matrix4d::Map(current_state_.O_T_EE.data()));
                    franka::CartesianPose out(std::array<double, 16>{});
                    if(motion_generator.status == 0){
                        RCLCPP_INFO(this->get_logger(), "Catch new target!");
                        out = franka::CartesianPose(current_state_.O_T_EE_c);
                        motion_generator.status = 1;
                    }
                    if(motion_generator.status == 1){
                        motion_generator.time += period.toSec();
                        double tau = motion_generator.time / motion_generator.max_time;
                        if(tau > 1)
                        {
                            out = franka::CartesianPose(current_state_.O_T_EE_c);
                            motion_generator.status = 2;
                        }
                        else
                        {
                            Eigen::Matrix<double, 4, 4> target_pose = Eigen::MatrixXd::Identity(4, 4);
                            // Translation part
                            double r_t = 10*pow(tau, 3) - 15*pow(tau, 4) + 6*pow(tau, 5);
                            target_pose.topRightCorner(3, 1) << r_t * motion_generator.position_error + motion_generator.init_position;
                            // Rotation part
                            Eigen::Quaterniond target_orientation = motion_generator.init_orientation.slerp(r_t, motion_generator.target_orientation);
                            target_pose.topLeftCorner(3, 3) << target_orientation.normalized().toRotationMatrix();
                            // Deploy to output
                            Eigen::Matrix4d::Map(&out.O_T_EE[0], 4, 4) = target_pose;
                            /*** Log for debug ***/
//                            RCLCPP_INFO(this->get_logger(), "t: %.4f", tau);
//                            RCLCPP_INFO(this->get_logger(), "position_target: %.4f, %.4f, %.4f", out.O_T_EE[12], out.O_T_EE[13], out.O_T_EE[14]);
//                            RCLCPP_INFO(this->get_logger(), "orientation_target: %.8f, %.8f, %.8f, %.8f",
//                                        target_orientation.x(), target_orientation.y(), target_orientation.z(), target_orientation.w());
//                            RCLCPP_INFO(this->get_logger(), "vel: %.4f, %.4f, %.4f", current_state_.O_dP_EE_c[3], current_state_.O_dP_EE_c[4], current_state_.O_dP_EE_c[5]);
//                            RCLCPP_INFO(this->get_logger(), "acc: %.4f, %.4f, %.4f", current_state_.O_ddP_EE_c[3], current_state_.O_ddP_EE_c[4], current_state_.O_ddP_EE_c[5]);
                        }
                    }
                    else if(motion_generator.status == 2){
                        out = franka::CartesianPose(current_state_.O_T_EE_c);
                    }

                    out.motion_finished = finish_;
                    return out;
                },
                franka::ControllerMode::kCartesianImpedance, true, franka::kDefaultCutoffFrequency);
        };
        control_thread_ = std::make_unique<std::thread>(kCartPoseControl);
        RCLCPP_INFO(this->get_logger(), "Init CartPoseControl controller successfully!");
    }

    void ControllerBase::initializeContinuousReading() {
        assert(isStopped());
        stopped_ = false;
        const auto kReading = [this]() {
            robot_->read([this](const franka::RobotState& state) {
                {
                    std::lock_guard<std::mutex> lock(read_mutex_);
                    current_state_ = state;
                }
                return not finish_;
            });
        };
        control_thread_ = std::make_unique<std::thread>(kReading);
    }

    ControllerBase::~ControllerBase() {
        stopRobot();
    }

    bool ControllerBase::isStopped() const {
        return stopped_;
    }
}
