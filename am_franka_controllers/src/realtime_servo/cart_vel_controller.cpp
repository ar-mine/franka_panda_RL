//
// Created by armine on 9/24/22.
//

#include <rclcpp/logging.hpp>
#include <franka/control_tools.h>
#include "am_franka_controllers/cart_vel_controller.h"

namespace am_franka_controllers {
    CartVelController::CartVelController(const std::string &node_name, const rclcpp::NodeOptions &node_options,
                                         const std::string &robot_ip) : Node(node_name, node_options) {
        initialize(robot_ip, this->get_logger());
        initializeCartVelControl();
    }

    void CartVelController::initialize(const std::string &robot_ip, const rclcpp::Logger &logger) {
        cart_vel_command_.fill(0.);
        franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
        if (not franka::hasRealtimeKernel()) {
            rt_config = franka::RealtimeConfig::kIgnore;
            RCLCPP_WARN(logger, "You are not using a real-time kernel. Using a real-time kernel is strongly recommended!");
        }
        robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
        robot_->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
//        gripper_ = std::make_unique<franka::Gripper>(robot_ip);
        gripper_ = std::make_unique<franka::VacuumGripper>(robot_ip);
        franka::VacuumGripperState vacuum_gripper_state = gripper_->readOnce();
        RCLCPP_INFO_STREAM(this->get_logger(), "Initial vacuum gripper state: " << vacuum_gripper_state);
        model_ = std::make_unique<franka::Model>(robot_->loadModel());
    }

    void CartVelController::write(const std::array<double, 6>& vel_cmd) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        cart_vel_command_ = vel_cmd;
    }

    franka::RobotState CartVelController::read() {
        std::lock_guard<std::mutex> lock(read_mutex_);
        return {current_state_};
    }

    void CartVelController::stopRobot() {
        if (not stopped_) {
            finish_ = true;
            control_thread_->join();
            finish_ = false;
            stopped_ = true;
            gripper_->stop();
        }
    }

    void CartVelController::initializeCartVelControl(){
        assert(isStopped());
        stopped_ = false;
        const auto kCartVelControl = [this]() {
            robot_->control(
                    [this](const franka::RobotState& state, const franka::Duration& period) {
                        {
                            std::lock_guard<std::mutex> lock(read_mutex_);
                            current_state_ = state;
                        }
                        double time = period.toSec();
                        double delta_acc_limit = time*0.5; // 5 m/s^2
                        std::array<double, 6> current_vel = current_state_.O_dP_EE_d;
                        for(int i=0; i<6; i++)
                        {
                            if(cart_vel_command_[i]-current_vel[i] > delta_acc_limit)
                                current_vel[i] += delta_acc_limit;
                            else if (cart_vel_command_[i]-current_vel[i] < -delta_acc_limit)
                                current_vel[i] -= delta_acc_limit;
                        }
//                        RCLCPP_INFO(this->get_logger(), "The cart_vel: %.4f %.4f %.4f %.4f %.4f %.4f",
//                                    current_vel[0], current_vel[1], current_vel[2], current_vel[3], current_vel[4], current_vel[5]);
                        std::lock_guard<std::mutex> lock(write_mutex_);
                        franka::CartesianVelocities out(current_vel);
                        out.motion_finished = finish_;
                        return out;
                    },
                    franka::ControllerMode::kCartesianImpedance, true, franka::kDefaultCutoffFrequency);
        };
        control_thread_ = std::make_unique<std::thread>(kCartVelControl);
    }

    void CartVelController::initializeContinuousReading() {
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

    CartVelController::~CartVelController() {
        stopRobot();
    }

    bool CartVelController::isStopped() const {
        return stopped_;
    }
}
