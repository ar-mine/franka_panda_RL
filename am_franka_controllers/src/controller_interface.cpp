//
// Created by armine on 9/24/22.
//
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include "franka/model.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <franka/exception.h>
#include "am_franka_controllers/controller_interface.h"

namespace am_franka_controllers{
    using namespace std::placeholders;
    ControllerInterface::ControllerInterface(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                                             const std::string& robot_ip) : CartVelController(node_name, node_options, robot_ip)
    {
        // The control freq is 1000/20=50hz
        timer_  = this->create_wall_timer(std::chrono::milliseconds(1000/control_freq),
                                          std::bind(&ControllerInterface::timer_callback, this));
        // Subscribe target pose topic
        cart_pose_server_ = rclcpp_action::create_server<CartPoseSet>(
                this,
                "~/cart_pose_set",
                std::bind(&ControllerInterface::cart_pose_goal, this, _1, _2),
                std::bind(&ControllerInterface::cart_pose_cancel, this, _1),
                std::bind(&ControllerInterface::cart_pose_accepted, this, _1));
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose",
                                                                                      rclcpp::QoS(1), std::bind(&ControllerInterface::target_pose_callback, this, _1));
        gripper_sub_ = this->create_subscription<std_msgs::msg::Bool>("~/gripper",
                                                                      rclcpp::QoS(1), std::bind(&ControllerInterface::gripper_callback, this, _1));

        vec_cart_error_last.fill(0.0);

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

    void ControllerInterface::timer_callback() {
        if(not target_changed)
            return;

        // Get current states
        read();

        // Calculate twist error
        tr2kdlFrame(current_state_.O_T_EE, kdl_cart_pos_current);
        auto error = KDL::diff(kdl_cart_pos_current, kdl_cart_pos_des_);
        Eigen::Matrix<double, 6, 1> vec_cart_error = (Eigen::Matrix<double, 6, 1>() << error.vel[0], error.vel[1], error.vel[2],
                                                                                       error.rot[0], error.rot[1], error.rot[2]).finished();
        Eigen::Matrix<double, 6, 1> cmd = vec_cart_error + 0.5*(vec_cart_error-vec_cart_error_last)*control_freq;
        vec_cart_error_last = vec_cart_error;
        if(cmd_scale(cmd, twist_limit, 5e-3))
        {
            target_changed = false;
            write({0, 0, 0, 0, 0, 0});
            return;
        }
        cmd *= control_freq;
        //        RCLCPP_INFO(this->get_logger(), "The original error: %.2f %.2f %.2f %.2f %.2f %.2f",
//                    error.vel[0], error.vel[1], error.vel[2],
//                    error.rot[0], error.rot[1], error.rot[2]);
//        RCLCPP_INFO(this->get_logger(), "The cart_error: %.4f %.4f %.4f %.4f %.4f %.4f",
//                    vec_cart_error[0], vec_cart_error[1], vec_cart_error[2], vec_cart_error[3], vec_cart_error[4], vec_cart_error[5]);

        // Assign calculated torques to controller
        std::array<double, 6> cart_vel{};
        Eigen::VectorXd::Map(&cart_vel[0], 6) = cmd;
//        RCLCPP_INFO(this->get_logger(), "The cart_vel: %.4f %.4f %.4f %.4f %.4f %.4f",
//                    cart_vel[0], cart_vel[1], cart_vel[2], cart_vel[3], cart_vel[4], cart_vel[5]);
//        cart_vel = {0, 0, 0, 0, 0, 0};
        write(cart_vel);
    }

    void ControllerInterface::target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        static KDL::Frame kdl_msg(KDL::Rotation::Quaternion(0, 0, 0, 1),
                                  KDL::Vector{0, 0, 0});
        KDL::Frame kdl_temp(KDL::Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w),
                            KDL::Vector{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z});

        // Whether current target pose is equal to the last one
        if(KDL::Equal(kdl_msg, kdl_temp, 5e-3))
            return;
        else
            kdl_msg = kdl_temp;

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
            target_changed = true;
            RCLCPP_INFO(this->get_logger(), "Change target pose successfully!");
        }

    }

    void ControllerInterface::gripper_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        try
        {
            if(msg->data)
            {
                gripper_->vacuum(100, std::chrono::milliseconds(1000));
                RCLCPP_INFO(this->get_logger(), "Try to suck object");
            }
            else
            {
//                gripper_->stop();
                gripper_->dropOff(std::chrono::milliseconds(1000));
                RCLCPP_INFO(this->get_logger(), "Try to drop out object");
            }
        }
        catch (franka::Exception const& e)
        {
            gripper_->stop();
            RCLCPP_FATAL_STREAM(this->get_logger(), e.what());
        }
    }

    rclcpp_action::GoalResponse ControllerInterface::cart_pose_goal(
            const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CartPoseSet::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received target pose as: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    goal->target.position.x, goal->target.position.y, goal->target.position.z,
                    goal->target.orientation.x, goal->target.orientation.y, goal->target.orientation.z, goal->target.orientation.w);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ControllerInterface::cart_pose_cancel(
            const std::shared_ptr<GoalHandleCartPoseSet> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ControllerInterface::cart_pose_accepted(const std::shared_ptr<GoalHandleCartPoseSet> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ControllerInterface::execute, this, _1), goal_handle}.detach();
    }

    void ControllerInterface::execute(const std::shared_ptr<GoalHandleCartPoseSet> goal_handle)
    {
        auto goal = goal_handle->get_goal()->target;
        auto feedback = std::make_shared<CartPoseSet::Feedback>();
        auto & sequence = feedback->current;
        auto result = std::make_shared<CartPoseSet::Result>();

        static KDL::Frame kdl_msg(KDL::Rotation::Quaternion(0, 0, 0, 1),
                                  KDL::Vector{0, 0, 0});
        KDL::Frame kdl_temp(KDL::Rotation::Quaternion(goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w),
                            KDL::Vector{goal.position.x, goal.position.y, goal.position.z});

        // Whether current target pose is equal to the last one
        if(KDL::Equal(kdl_msg, kdl_temp, 5e-3))
            return;
        else
            kdl_msg = kdl_temp;

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
            target_changed = true;
            RCLCPP_INFO(this->get_logger(), "Change target pose successfully!");
        }

        while(not target_changed)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Execute the path successfully!");
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

    void twist_scale(KDL::Twist& src_kdl_twist, KDL::Twist& limit_kdl_twist, double eps)
    {
        double factor = 1, temp;
        for(int i=0; i<3; i++)
        {
            if(abs(src_kdl_twist.vel.data[i])  <= eps)
            {
                src_kdl_twist.vel.data[i] = 0.0;
                continue;
            }
            temp = abs(src_kdl_twist.vel.data[i]/limit_kdl_twist.vel.data[i]);
            if(temp > factor)
                factor = temp;
        }
        src_kdl_twist.vel = src_kdl_twist.vel/factor;

        factor = 1;
        for(int i=0; i<3; i++)
        {
            if(abs(src_kdl_twist.rot.data[i]) <= eps)
            {
                src_kdl_twist.rot.data[i] = 0.0;
                continue;
            }
            temp = abs(src_kdl_twist.rot.data[i]/limit_kdl_twist.rot.data[i]);
            if(temp > factor)
                factor = temp;
        }
        src_kdl_twist.rot = src_kdl_twist.rot/factor;
    }

    bool cmd_scale(Eigen::Matrix<double, 6, 1>& cmd_src, Eigen::Matrix<double, 6, 1>& cmd_limit, double eps)
    {
        bool get_target = true;
        double factor = 1, temp;
        for(int i=0; i<6; i++)
        {
            if(i < 3)
            {
                if(abs(cmd_src[i])  <= eps)
                {
                    cmd_src[i] = 0.0;
                    continue;
                }
            }
            else
            {
                if(abs(cmd_src[i])  <= eps*10)
                {
                    cmd_src[i] = 0.0;
                    continue;
                }
            }
            get_target = false;
            temp = abs(cmd_src[i]/cmd_limit[i]);
            if(temp > factor)
                factor = temp;
        }
        cmd_src = cmd_src/factor;
        return get_target;
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<am_franka_controllers::ControllerInterface> node =
            std::make_shared<am_franka_controllers::ControllerInterface>("controller_interface_node", node_options, "172.16.0.2");

    try{
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch(franka::Exception const& e)
    {
        rclcpp::shutdown();
        RCLCPP_FATAL_STREAM(node->get_logger(), e.what());
    }

    return EXIT_SUCCESS;
}

