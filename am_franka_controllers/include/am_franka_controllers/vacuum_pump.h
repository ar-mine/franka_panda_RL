//
// Created by armine on 21/12/22.
//

#ifndef BUILD_VACUUM_PUMP_H
#define BUILD_VACUUM_PUMP_H

#include <franka/vacuum_gripper_state.h>
#include <franka/vacuum_gripper.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <rclcpp/node.hpp>

namespace am_franka_controllers{

    class VacuumPump{
    public:
//        using StatusInt = std::underlying_type<franka::VacuumGripperDeviceStatus>::type;
        using ProductionSetupProfile = franka::VacuumGripper::ProductionSetupProfile;
        using StatusInt = std::underlying_type<franka::VacuumGripperDeviceStatus>::type;

        enum class Status : StatusInt
        {
            kGreen = static_cast<int>(franka::VacuumGripperDeviceStatus::kGreen),
            kYellow = static_cast<int>(franka::VacuumGripperDeviceStatus::kYellow),
            kOrange = static_cast<int>(franka::VacuumGripperDeviceStatus::kOrange),
            kRed = static_cast<int>(franka::VacuumGripperDeviceStatus::kRed),
            disconnected = static_cast<int>(kRed + 1)
        };

        explicit VacuumPump(const rclcpp::Node::SharedPtr& node);

        ~VacuumPump();

        /** Connect the pump device to an actual pump, the pump operations are then done in a background thread*/
        bool connect(const std::string & ip);

        /** Disconnect from the actual pump */
        void disconnect();

        /** Access the vacuum gripper state */
        const franka::VacuumGripperState & state() const;

        /** Get the pump status */
        Status status() const;

        /** True if the pump is currently busy */
        bool busy() const;

        /** True if the last command succeeded, false otherwise */
        bool success() const;

        /** Message describing the latest error */
        const std::string & error() const;

        /**
         * Vacuums an object.
         *
         * @param[in] vacuum Setpoint for control mode. Unit: \f$[10*mbar]\f$.
         * @param[in] timeout Vacuum timeout. Unit: \f$[ms]\f$.
         * @param[in] profile Production setup profile P0 to P3. Default: P0.
         *
         * @return True if the command was requested, false otherwise
         */
        bool vacuum(uint8_t vacuum,
                    std::chrono::milliseconds timeout,
                    ProductionSetupProfile profile = ProductionSetupProfile::kP0);

        /**
         * Drops the grasped object off.
         *
         * @param[in] timeout Dropoff timeout. Unit: \f$[ms]\f$.
         *
         * @return True if the command was requested, false otherwise
         */
        bool dropOff(std::chrono::milliseconds timeout);

        /**
         * Stops a currently running vacuum gripper vacuum or drop off operation.
         *
         * @return True if the command was requested, false otherwise
         */
        bool stop();

    private:
        // ROS node for logger
        const rclcpp::Node::SharedPtr& node_;
        // Name of pump
        const std::string name_ = "Pump";
        // Status of the pump
        Status status_ = Status::disconnected;
        // Only non-null if the pump is connected
        std::unique_ptr<franka::VacuumGripper> gripper_;
        // Thread for reading the gripper state
        std::thread stateThread_;
        // Mutex for protecting the gripper state
        mutable std::mutex stateMutex_;
        // Current state
        franka::VacuumGripperState state_;
        // Thread for sending commands
        std::thread commandThread_;
        // Thread for interrupting commands
        std::thread interruptThread_;
        // Only true while the gripper is connected
        std::atomic<bool> connected_{false};
        // Only true while a command is being executed
        std::atomic<bool> busy_{false};
        // Only true if a command has been interrupted
        std::atomic<bool> interrupted_{false};
        struct Command
        {
            std::string name;
            std::function<bool()> callback;
        };
        // Only valid while a command is being executed
        Command command_;
        // Represent the last command executed
        uint8_t last_command_id_ = 0;
        // Store the last command success
        bool success_ = false;
        // Store the last command error (if any)
        std::string error_;
    };
}

#endif //BUILD_VACUUM_PUMP_H
