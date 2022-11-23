//
// Created by armine on 9/24/22.
//
#include <Eigen/Core>
#include "franka/model.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "am_franka_controllers/position_controller.h"

namespace am_franka_controllers{

    PositionController::PositionController(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                                           const std::string& robot_ip) : ControllerBase(node_name, node_options, robot_ip)
    {
        this->declare_parameter("k_gains");
        this->declare_parameter("effort_compensation");

        timer_  = this->create_wall_timer(std::chrono::milliseconds(20),
                                          std::bind(&PositionController::timer_callback, this));

        k_gains_ = this->get_parameter("k_gains").as_double_array();
        if (k_gains_.empty()) {
            RCLCPP_FATAL(this->get_logger(), "k_gains parameter not set");
            k_gains_ = {1, 1, 1, 1, 1, 1, 1};
        }
        effort_compensation = this->get_parameter("effort_compensation").as_double_array();
        if (effort_compensation.empty()) {
            RCLCPP_FATAL(this->get_logger(), "k_gains parameter not set");
            effort_compensation = {0, 0, 0, 0, 0, 0, 0};
        }
        jnt_pos_des_ = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    }

    void PositionController::timer_callback() {
        read();
//        RCLCPP_INFO(this->get_logger(), "The q: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                    current_state_.q[0], current_state_.q[1], current_state_.q[2], current_state_.q[3], current_state_.q[4], current_state_.q[5], current_state_.q[6]);
        std::array<double, 7> error{};
        /* Calculate Loop */
        for(size_t i=0; i<7; i++)
        {
            error.at(i) = jnt_pos_des_.at(i)-current_state_.q.at(i);
            jnt_pos_err_.at(i) = k_gains_.at(i) * error.at(i);
        }
        RCLCPP_INFO(this->get_logger(), "The error: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    error[0], error[1], error[2], error[3], error[4], error[5], error[6]);
//        RCLCPP_INFO(this->get_logger(), "The error: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//                    current_state_.O_T_EE.at(12), current_state_.O_T_EE[13], current_state_.O_T_EE[14], current_state_.O_T_EE[15],
//                    current_state_.O_T_EE[15], current_state_.O_T_EE[15], current_state_.O_T_EE[15]);
        Eigen::Map<const Eigen::Matrix<double, 7, 7>> M_q(model_->mass(current_state_).data());
//        RCLCPP_INFO_STREAM(this->get_logger(), M_q);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_error(jnt_pos_err_.data());
        auto tau = M_q*q_error;
        std::array<double, 7> efforts{};
        Eigen::VectorXd::Map(&efforts[0], 7) = tau;
        for(size_t i=0; i<7; i++)
        {
            if(abs(error[i])>0.05)
            {
                if(efforts[i] > 0)
                    efforts[i] += effort_compensation[i];
                else if(efforts[i] < 0)
                    efforts[i] += -effort_compensation[i];
                else
                {
                    if(error[i] > 0)
                        efforts[i] += effort_compensation[i];
                    else if(error[i] < 0)
                        efforts[i] += -effort_compensation[i];
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "The efforts: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    efforts[0], efforts[1], efforts[2], efforts[3], efforts[4], efforts[5], efforts[6]);
        /* Calculate Loop */

        write(efforts);
    }

}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
//    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<am_franka_controllers::PositionController> node =
            std::make_shared<am_franka_controllers::PositionController>("position_controller_node", node_options, "172.16.0.2");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

