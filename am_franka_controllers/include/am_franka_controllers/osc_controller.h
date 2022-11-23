//
// Created by armine on 9/24/22.
//

#ifndef BUILD_POSITION_CONTROLLER_H
#define BUILD_POSITION_CONTROLLER_H

#include <kdl/frames.hpp>
#include "am_franka_controllers/controller_base.h"
namespace am_franka_controllers{

    class OSCController : public ControllerBase{
    public:
        explicit OSCController(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                                    const std::string& robot_ip);

        void timer_callback();
        void target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;

    private:
        KDL::Frame kdl_cart_pos_des_;
        KDL::Frame kdl_cart_pos_current;

        std::vector<double> k_gains_;
        std::vector<double> d_gains_;
        Eigen::Matrix<double, 6, 6> k_gains_diag_;
        Eigen::Matrix<double, 6, 6> d_gains_diag_;

        KDL::Twist kdl_twist_limit{KDL::Vector{0.05, 0.05, 0.05},
                                   KDL::Vector(0.25, 0.25, 0.25)};
        KDL::Vector kdl_frame_upper_limit{0.5, 0.3, 0.5};
        KDL::Vector kdl_frame_lower_limit{0.1, -0.3, 0.0};
    };

    void tr2kdlFrame(const std::array<double, 16> &cart_pos, KDL::Frame &kdl_cart_pos);
    std::array<double, 4> kdlFrame2quat(KDL::Frame &kdl_cart_pos);
    void twist_scale(KDL::Twist& src_kdl_twist, KDL::Twist& limit_kdl_twist);

}

#endif //BUILD_POSITION_CONTROLLER_H
