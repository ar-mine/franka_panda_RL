//
// Created by armine on 21/12/22.
//

#include <franka/exception.h>

#include <memory>
#include "am_franka_controllers/vacuum_pump.h"

namespace am_franka_controllers{

    VacuumPump::VacuumPump(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        ;
    }

    VacuumPump::~VacuumPump()
    {
        disconnect();
    }

    bool VacuumPump::connect(const std::string & ip)
    {
        if(gripper_)
        {
            RCLCPP_WARN(node_->get_logger(), "%s is already connected", name_.c_str());
        }
        try
        {
            gripper_ = std::make_unique<franka::VacuumGripper>(ip);
            RCLCPP_INFO(node_->get_logger(), "%s connected to %s", name_.c_str(), ip.c_str());
        }
        catch(const franka::NetworkException & exc)
        {
            RCLCPP_ERROR(node_->get_logger(), "%s failed to connect to %s: %s", name_.c_str(), ip.c_str(), exc.what());
            return false;
        }
        catch(const franka::IncompatibleVersionException & exc)
        {
            RCLCPP_ERROR(node_->get_logger(), "%s incompatible version with %s: %s", name_.c_str(), ip.c_str(), exc.what());
            return false;
        }
        connected_ = true;
        stateThread_ = std::thread([this]() {
            while(connected_)
            {
                try
                {
                    auto stateIn = gripper_->readOnce();
                    std::unique_lock<std::mutex> lock(stateMutex_);
                    state_ = stateIn;
                    status_ = Status(static_cast<StatusInt>(state_.device_status));
                }
                catch(const franka::NetworkException & exc)
                {
                    RCLCPP_ERROR(node_->get_logger(), "%s connection lost, failed to read state: %s", name_.c_str(), exc.what());
                }
            }
        });
        commandThread_ = std::thread([this]() {
            while(connected_)
            {
                if(busy_)
                {
                    bool s = false;
                    std::string error;
                    try
                    {
                        s = command_.callback();
                        error = "";
                    }
                    catch(const franka::CommandException & exc)
                    {
                        error = exc.what();
                        RCLCPP_ERROR(node_->get_logger(), "%s %s command failed: %s", name_.c_str(), command_.name.c_str(), exc.what());
                    }
                    catch(const franka::NetworkException & exc)
                    {
                        error = exc.what();
                        RCLCPP_ERROR(node_->get_logger(), "%s connection lost, failed to execute %s command: %s", name_.c_str(), command_.name.c_str(), exc.what());
                    }
                    if(!interrupted_)
                    {
                        success_ = s;
                        error_ = error;
                    }
                    else
                    {
                        interrupted_ = false;
                    }
                    busy_ = false;
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        });
        interruptThread_ = std::thread([this]() {
            while(connected_)
            {
                if(interrupted_)
                {
                    bool busy = busy_;
                    bool s = false;
                    std::string error;
                    try
                    {
                        s = gripper_->stop();
                        error = "";
                    }
                    catch(const franka::CommandException & exc)
                    {
                        error = exc.what();
                        RCLCPP_ERROR(node_->get_logger(), "%s stop command failed: %s", name_.c_str(), error.c_str());
                    }
                    catch(const franka::NetworkException & exc)
                    {
                        error = exc.what();
                        RCLCPP_ERROR(node_->get_logger(), "%s connection lost, failed to execute stop command: %s", name_.c_str(), error.c_str());
                    }
                    success_ = s;
                    error_ = error;
                    if(!busy)
                    {
                        interrupted_ = false;
                    }
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        });
        return true;
    }

    void VacuumPump::disconnect()
    {
        if(!gripper_)
        {
            return;
        }
        connected_ = false;
        commandThread_.join();
        stateThread_.join();
        interruptThread_.join();
        gripper_.reset(nullptr);
    }

    const franka::VacuumGripperState & VacuumPump::state() const
    {
        std::unique_lock<std::mutex> lock(stateMutex_);
        return state_;
    }

    auto VacuumPump::status() const -> Status
    {
        std::unique_lock<std::mutex> lock(stateMutex_);
        return status_;
    }

    bool VacuumPump::busy() const
    {
        return busy_;
    }

    bool VacuumPump::success() const
    {
        return success_;
    }

    const std::string & VacuumPump::error() const
    {
        return error_;
    }

    bool VacuumPump::vacuum(uint8_t vacuum, std::chrono::milliseconds timeout, ProductionSetupProfile profile)
    {
        if(!gripper_)
        {
            return true;
        }
        if(busy_)
        {
            RCLCPP_ERROR(node_->get_logger(), "%s is already busy executing %s command", name_.c_str(), command_.name.c_str());
            return false;
        }
        busy_ = true;
        command_ = {"vacuum", [=]() { return gripper_->vacuum(vacuum, timeout, profile); }};
        last_command_id_ = 1;
        return true;
    }

    bool VacuumPump::dropOff(std::chrono::milliseconds timeout)
    {
        if(!gripper_)
        {
            return true;
        }
        if(busy_)
        {
            RCLCPP_ERROR(node_->get_logger(), "%s is already busy executing %s command", name_.c_str(), command_.name.c_str());
            return false;
        }
        busy_ = true;
        command_ = {"dropOff", [=]() { return gripper_->dropOff(timeout); }};
        last_command_id_ = 2;
        return true;
    }

    bool VacuumPump::stop()
    {
        if(!gripper_)
        {
            return true;
        }
        if(interrupted_)
        {
            RCLCPP_ERROR(node_->get_logger(), "%s stop command has already been requested", name_.c_str());
            return false;
        }
        interrupted_ = true;
        last_command_id_ = 3;
        return true;
    }
}