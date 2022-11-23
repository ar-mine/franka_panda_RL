//
// Created by armine on 9/24/22.
//

#ifndef BUILD_TORQUE_CONTROLLER_BASE_H
#define BUILD_CONTROLLER_BASE_H

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/robot.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/vacuum_gripper.h>

namespace am_franka_controllers{

class CartVelController : public rclcpp::Node{
    public:
    /**
     * Connects to the robot. This method can block for up to one minute if the robot is not
     * responding. An exception will be thrown if the connection cannot be established.
     *
     * @param[in] robot_ip IP address or hostname of the robot.
     * @param[im] logger ROS Logger to print eventual warnings.
     */
    explicit CartVelController(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                            const std::string& robot_ip);
    CartVelController(const CartVelController &) = delete;
    CartVelController & operator =(const CartVelController &other) = delete;
    CartVelController & operator = (CartVelController && other) = delete;
    CartVelController(CartVelController && other) = delete;

    /// Stops the currently running loop and closes the connection with the robot.
    ~CartVelController() override;

    /**
     * */
    void initialize(const std::string &robot_ip, const rclcpp::Logger &logger);

    /**
     * Starts a torque control loop. Before using this method make sure that no other
     * control or reading loop is currently active.
     */
    void initializeCartVelControl();

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
    void write(const std::array<double, 6> &cart_vel_cmd);


    /// @return true if there is no control or reading loop running.
    bool isStopped() const;

    franka::RobotState current_state_;
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    std::unique_ptr<franka::VacuumGripper> gripper_;

private:
    std::unique_ptr<std::thread> control_thread_;
    std::mutex read_mutex_;
    std::mutex write_mutex_;
    std::atomic_bool finish_{false};
    bool stopped_ = true;
    std::array<double, 6> cart_vel_command_{};

};
}

#endif //BUILD_TORQUE_CONTROLLER_BASE_H
