//
// Created by armine on 9/24/22.
//
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include "franka/model.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "am_franka_controllers/osc_controller.h"

namespace am_franka_controllers{

    OSCController::OSCController(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                                 const std::string& robot_ip) : ControllerBase(node_name, node_options, robot_ip)
    {
        // The control freq is 1000/20=50hz
        timer_  = this->create_wall_timer(std::chrono::milliseconds(20),
                                          std::bind(&OSCController::timer_callback, this));
        // Subscribe target pose topic
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose",
                                                                                      rclcpp::QoS(1), std::bind(&OSCController::target_pose_callback, this, std::placeholders::_1));

        // Init k and d parameters, and create diag matrix for easier computation later
        k_gains_ = this->get_parameter("k_gains").as_double_array();
        if (k_gains_.empty()) {
            RCLCPP_FATAL(this->get_logger(), "k_gains parameter not set");
            k_gains_ = {1, 1, 1, 1, 1, 1};
        }
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> k_gain_vector(k_gains_.data());
        k_gains_diag_ = k_gain_vector.asDiagonal();
        d_gains_ = this->get_parameter("d_gains").as_double_array();
        if (d_gains_.empty()) {
            RCLCPP_FATAL(this->get_logger(), "d_gains parameter not set");
            d_gains_ = {0, 0, 0, 0, 0, 0};
        }
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> d_gain_vector(d_gains_.data());
        d_gains_diag_ = d_gain_vector.asDiagonal();

        // Delay to keep the system stable
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Observe current pose and assign target pose to current pose
        read();
        tr2kdlFrame(current_state_.O_T_EE, kdl_cart_pos_des_);
        RCLCPP_INFO(this->get_logger(), "Controller init completed!");
        auto init_quat = kdlFrame2quat(kdl_cart_pos_des_);
        RCLCPP_INFO(this->get_logger(), "The initial pose: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    kdl_cart_pos_des_.p.x(), kdl_cart_pos_des_.p.y(), kdl_cart_pos_des_.p.z(),
                    init_quat[0], init_quat[1], init_quat[2], init_quat[3]);
        RCLCPP_INFO(this->get_logger(), "The initial joints: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    current_state_.q[0], current_state_.q[1], current_state_.q[2], current_state_.q[3],
                    current_state_.q[4], current_state_.q[5], current_state_.q[6]);
    }

    void OSCController::timer_callback() {
        // Get current states
        read();
//        RCLCPP_INFO(this->get_logger(), "The q: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//            current_state_.q[0], current_state_.q[1], current_state_.q[2], current_state_.q[3], current_state_.q[4], current_state_.q[5], current_state_.q[6]);

        // Calculate twist error
        tr2kdlFrame(current_state_.O_T_EE, kdl_cart_pos_current);
        auto error = KDL::diff(kdl_cart_pos_current, kdl_cart_pos_des_);
        RCLCPP_INFO(this->get_logger(), "The cart_error: %.2f %.2f %.2f %.2f %.2f %.2f",
                    error.vel[0], error.vel[1], error.vel[2],
                    error.rot[0], error.rot[1], error.rot[2]);
        twist_scale(error, kdl_twist_limit);
        Eigen::Matrix<double, 6, 1> vec_cart_error = (Eigen::Matrix<double, 6, 1>() << error.vel[0], error.vel[1], error.vel[2],
                                                                                       error.rot[0], error.rot[1], error.rot[2]).finished();
//        RCLCPP_INFO(this->get_logger(), "The cart_error: %.2f %.2f %.2f %.2f %.2f %.2f",
//                    vec_cart_error[0], vec_cart_error[1], vec_cart_error[2], vec_cart_error[3], vec_cart_error[4], vec_cart_error[5]);

        // Calculate needed states
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> jnt_vel_current(current_state_.dq.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 7>> M_q(model_->mass(current_state_).data());
//        RCLCPP_INFO_STREAM(this->get_logger(), M_q);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> Jac(model_->zeroJacobian(franka::Frame::kEndEffector, current_state_).data());
//        RCLCPP_INFO_STREAM(this->get_logger(), Jac);
        auto M_q_inverse = M_q.inverse();
        auto M_eef_inv = Jac * M_q_inverse * Jac.transpose();
        auto M_eef = M_eef_inv.inverse();

        // Get the target torques
        auto tau = Jac.transpose()*M_eef*(k_gains_diag_*vec_cart_error - d_gains_diag_*Jac*jnt_vel_current);

        // Assign calculated torques to controller
        std::array<double, 7> efforts{};
        Eigen::VectorXd::Map(&efforts[0], 7) = tau;
        RCLCPP_INFO(this->get_logger(), "The efforts: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    efforts[0], efforts[1], efforts[2], efforts[3], efforts[4], efforts[5], efforts[6]);
//        efforts = {0, 0, 0, 0, 0, 0, 0};
        write(efforts);
    }

    void OSCController::target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        static KDL::Frame kdl_msg(KDL::Rotation::Quaternion(0, 0, 0, 1),
                                  KDL::Vector{0, 0, 0});
        KDL::Frame kdl_temp(KDL::Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w),
                            KDL::Vector{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z});

        // Whether current target pose is equal to the last one
        if(KDL::Equal(kdl_msg, kdl_temp, 1e-3))
        {
            return;
        }
        else
        {
            kdl_msg = kdl_temp;
        }

        // Whether under limit
        bool off_limit = false;
        for(int i=0; i<3; i++)
        {
            if(kdl_msg.p.data[i]<kdl_frame_lower_limit.data[i] || kdl_msg.p.data[i]>kdl_frame_upper_limit.data[i])
            {
                off_limit = true;
                break;
            }
        }

        if(off_limit)
        {
            RCLCPP_FATAL(this->get_logger(), "The target pose is out of bounds!");
            return;
        }
        else
        {
            kdl_cart_pos_des_ = kdl_msg;
            RCLCPP_INFO(this->get_logger(), "Change target pose successfully!");
        }

    }

    void tr2kdlFrame(const std::array<double, 16> &cart_pos, KDL::Frame &kdl_cart_pos)
    {
        kdl_cart_pos = {KDL::Rotation(cart_pos[0], cart_pos[4], cart_pos[8],
                                         cart_pos[1], cart_pos[5], cart_pos[9],
                                         cart_pos[2], cart_pos[6], cart_pos[10]),
                        KDL::Vector(cart_pos[12], cart_pos[13], cart_pos[14])};
    }

    std::array<double, 4> kdlFrame2quat(KDL::Frame &kdl_cart_pos)
    {
        std::array<double, 4> quat{};
        kdl_cart_pos.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
        return quat;
    }

    void twist_scale(KDL::Twist& src_kdl_twist, KDL::Twist& limit_kdl_twist)
    {
        double factor = 1, temp;
        for(int i=0; i<3; i++)
        {
            temp = abs(src_kdl_twist.vel.data[i]/limit_kdl_twist.vel.data[i]);
            if(temp > factor)
                factor = temp;
        }
        src_kdl_twist.vel = src_kdl_twist.vel/factor;

        factor = 1;
        for(int i=0; i<3; i++)
        {
            temp = abs(src_kdl_twist.rot.data[i]/limit_kdl_twist.rot.data[i]);
            if(temp > factor)
                factor = temp;
        }
        src_kdl_twist.rot = src_kdl_twist.rot/factor;
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<am_franka_controllers::OSCController> node =
            std::make_shared<am_franka_controllers::OSCController>("osc_controller_node", node_options, "172.16.0.2");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

