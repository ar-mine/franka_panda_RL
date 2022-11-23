//
// Created by armine on 9/24/22.
//

#ifndef BUILD_POSITION_CONTROLLER_H
#define BUILD_POSITION_CONTROLLER_H

#include "am_franka_controllers/controller_base.h"
namespace am_franka_controllers{

    class PositionController : public ControllerBase{
    public:
        explicit PositionController(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                                    const std::string& robot_ip);

        void timer_callback();

        rclcpp::TimerBase::SharedPtr timer_;
    private:
        std::array<double, 7> jnt_pos_des_{};
        std::array<double, 7> jnt_pos_err_{};

        std::vector<double> k_gains_;

        std::vector<double> effort_compensation;
    };

}

#endif //BUILD_POSITION_CONTROLLER_H
