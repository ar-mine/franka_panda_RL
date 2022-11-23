//
// Created by armine on 9/24/22.
//

#ifndef BUILD_CONTROLLER_BASE_H
#define BUILD_CONTROLLER_BASE_H

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/vacuum_gripper.h>
#include <franka/model.h>

namespace am_franka_controllers{

enum ControllerType{
    JntTorque, JntVelocity, JntPosition, CartVelocity, CartPose
};

struct MotionGenerator{
    int status = 2;
    double time = 0.0;
    double max_time = 0.0;
    Eigen::Matrix<double, 3, 1> init_position{};
    Eigen::Matrix<double, 3, 1> position_error;

    Eigen::Quaterniond init_orientation{};
    Eigen::Quaterniond target_orientation{};
};

class ControllerBase : public rclcpp::Node{
    public:
    /**
     * Connects to the robot. This method can block for up to one minute if the robot is not
     * responding. An exception will be thrown if the connection cannot be established.
     *
     * @param[in] node_name name used to initialize ROS node.
     * @param[in] node_options options used to initialize ROS node.
     * @param[in] robot_ip IP address or hostname of the robot.
     * @param[in] robot_ip IP address or hostname of the robot.
     */
    explicit ControllerBase(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                            const std::string& robot_ip, const ControllerType& controller_type);
    ControllerBase(const ControllerBase &) = delete;
    ControllerBase & operator =(const ControllerBase &other) = delete;
    ControllerBase & operator = (ControllerBase && other) = delete;
    ControllerBase(ControllerBase && other) = delete;

    /// Stops the currently running loop and closes the connection with the robot.
    ~ControllerBase() override;

    /**
     * */
    void initialize(const std::string &robot_ip, const rclcpp::Logger &logger);

    /**
     * Starts a torque control loop. Before using this method make sure that no other
     * control or reading loop is currently active.
     */
    void initializeTorqueControl();
    void initializeCartPoseControl();

    /**
     * Starts a reading loop of the robot state. Before using this method make sure that no other
     * control or reading loop is currently active.
     */
    void initializeContinuousReading();

    /// stops the control or reading loop of the robot.
    void stopRobot();

    /**
     * Get the current robot state in a thread-safe way.
     * @return current robot state.
     */
    franka::RobotState read();

    /**
     * Sends new desired torque commands to the control loop in a thread-safe way.
     * The robot will use these torques until a different set of torques are commanded.
     * @param[in] efforts torque command for each joint.
     */
    void jntWrite(const std::array<double, 7> &jnt_command);
    void cartVelWrite(const std::array<double, 6> &cart_vel_command);
    void cartPoseWrite(const std::array<double, 16> &cart_pose_command);


    /// @return true if there is no control or reading loop running.
    bool isStopped() const;

    franka::RobotState current_state_;
    std::unique_ptr<franka::Model> model_;
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::VacuumGripper> vacuum_gripper_;
    std::unique_ptr<franka::Gripper> gripper_;

    bool stopped_ = true;
    MotionGenerator motion_generator;
private:
    std::unique_ptr<std::thread> control_thread_;
    std::mutex read_mutex_;
    std::mutex write_mutex_;
    std::atomic_bool finish_{false};

    ControllerType controller_type_;

    std::array<double, 7> jnt_command_{};
    std::array<double, 6> cart_vel_command_{};
    std::array<double, 16> cart_pose_command_{};
};
}

#endif //BUILD_CONTROLLER_BASE_H
