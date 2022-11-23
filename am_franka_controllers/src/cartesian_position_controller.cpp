//
// Created by armine on 9/24/22.
//

#include "franka/model.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <franka/exception.h>
#include "am_franka_controllers/cartesian_position_controller.h"

namespace am_franka_controllers{
    using namespace std::chrono_literals;
    using namespace std::placeholders;
    CartesianPositionController::CartesianPositionController(const std::string& node_name, const rclcpp::NodeOptions& node_options,
                                                             const std::string& robot_ip) : ControllerBase(node_name, node_options, robot_ip, ControllerType::CartPose)
    {
//        robot_->automaticErrorRecovery();
        // Set timer as signal generator
        timer_  = this->create_wall_timer(std::chrono::milliseconds(50),
                                          std::bind(&CartesianPositionController::timer_callback, this));
        std::thread([this, robot_ip]() {
            // Initialize gripper
            vacuum_gripper_ = std::make_unique<franka::VacuumGripper>(robot_ip);
            while(not stopped_)
            {
                if(gripper_status >= 0){
                    try
                    {
                        if(gripper_status)
                        {
                            vacuum_gripper_->vacuum(50, std::chrono::milliseconds(1000));
                            RCLCPP_INFO(this->get_logger(), "Try to suck object");
                        }
                        else
                        {
                            vacuum_gripper_->dropOff(std::chrono::milliseconds(1000));
                            RCLCPP_INFO(this->get_logger(), "Try to drop out object");
                        }
                    }
                    catch (franka::Exception const& e)
                    {
                        vacuum_gripper_->stop();
                        RCLCPP_FATAL_STREAM(this->get_logger(), e.what());
                    }
                    gripper_status = -1;
                }
                else {
                    try {
                        franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper_->readOnce();
                        RCLCPP_INFO_STREAM(this->get_logger(), "Vacuum gripper state: " << vacuum_gripper_state);
                    }
                    catch (franka::Exception const &e) {
                        vacuum_gripper_->stop();
                        RCLCPP_FATAL_STREAM(this->get_logger(), e.what());
                    }
                    std::this_thread::sleep_for(100ms);
                }
            }
        }).detach();
        // Pose set
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("~/target_pose",
                                                                               rclcpp::QoS(1), std::bind(&CartesianPositionController::target_pose_callback, this, _1));
        cart_pose_server_ = rclcpp_action::create_server<CartPoseSet>(
                        this,
                        "cart_pose_set",
                        std::bind(&CartesianPositionController::cart_pose_goal, this, _1, _2),
                        std::bind(&CartesianPositionController::cart_pose_cancel, this, _1),
                        std::bind(&CartesianPositionController::cart_pose_accepted, this, _1));
        // Set gripper subscription
        gripper_sub_ = this->create_subscription<std_msgs::msg::Bool>("~/gripper",
                                                                      rclcpp::QoS(1), std::bind(&CartesianPositionController::gripper_callback, this, _1));

        // Delay to keep the system stable
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Equilibrium point is the initial pose
        read();

        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(current_state_.O_T_EE_c.data()));
        cartPoseWrite(current_state_.O_T_EE_c);
        Eigen::Vector3d position_d(initial_transform.translation());
        Eigen::Quaterniond orientation_d(initial_transform.linear());
        position_d_ = position_d, orientation_d_ = orientation_d;

        RCLCPP_INFO(this->get_logger(), "The initial pose: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    position_d_[0], position_d_[1], position_d_[2],
                    orientation_d_.coeffs()[0], orientation_d_.coeffs()[1], orientation_d_.coeffs()[2], orientation_d_.coeffs()[3]);
    }

    void CartesianPositionController::timer_callback() {
        ;
    }

    void CartesianPositionController::target_pose_callback(const geometry_msgs::msg::Pose::ConstSharedPtr msg)
    {
        position_d_[0] = msg->position.x;
        position_d_[1] = msg->position.y;
        position_d_[2] = msg->position.z;

        orientation_d_ = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
    }

    void CartesianPositionController::gripper_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
    {
        if(msg->data)
        {
            gripper_status = 1;
        }
        else
        {
            gripper_status = 0;
        }
    }

    int CartesianPositionController::pose_error_compute()
    {
        // Get current states
        read();
        std::array<double, 16> cart_pose = current_state_.O_T_EE_c;
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(cart_pose.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

        // Compute error to desired equilibrium pose
        Eigen::Matrix<double, 3, 1> position_error = position_d_ - position;
        // When rotation error is larger than pi/2
        double orientation_error = orientation_d_.coeffs().dot(orientation.coeffs());
        if (orientation_error < 0.0)
        {
            orientation.coeffs() << -orientation.coeffs();
        }
        orientation_error = acos(std::max(abs(orientation_error), 1.0));

        // When not reach target pose
        if((position_error.norm()>position_eps || orientation_error>orientation_eps) && motion_generator.status == 2)
        {
            MotionGenerator temp;
            // Set status
            temp.status = 0;
            // Set error
            temp.position_error << position_error;
            // Set duration
            double position_max_time = (15*temp.position_error.head(3).cwiseAbs()/8/position_max_vel).              // time for velocity
                    cwiseMax((10*temp.position_error.cwiseAbs()/sqrt(3)/position_max_acc).cwiseSqrt()).   // time for acceleration
                    maxCoeff();                                                                                   // max element
            double orientation_max_time = 0.0;
            if(orientation_error > orientation_eps){
                double theta = acos(abs(orientation_error));
                orientation_max_time = std::max(15*abs(theta)/8/orientation_max_vel,
                                                sqrt(10*abs(theta)/sqrt(3)/orientation_max_acc));
            }
            temp.max_time = std::max(position_max_time, orientation_max_time);
            // Set init and target
            temp.init_position = {current_state_.O_T_EE_c[12], current_state_.O_T_EE_c[13], current_state_.O_T_EE_c[14]};
            temp.init_orientation = orientation;
            temp.target_orientation = orientation_d_;

            motion_generator = temp;
            RCLCPP_INFO(this->get_logger(), "Max time: %.2f, orientation_error: %.4f", temp.max_time, orientation_error);
            return 1;
        }
        else{
            RCLCPP_INFO(this->get_logger(), "The error is less than the threshold.");
            return 0;
        }
    }

    rclcpp_action::GoalResponse CartesianPositionController::cart_pose_goal(
            const rclcpp_action::GoalUUID& uuid, const std::shared_ptr<const CartPoseSet::Goal>& goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received target pose: %.2f %.2f %.2f, %.2f %.2f %.2f %.2f",
                    goal->target.position.x, goal->target.position.y, goal->target.position.z,
                    goal->target.orientation.x, goal->target.orientation.y, goal->target.orientation.z, goal->target.orientation.w);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse CartesianPositionController::cart_pose_cancel(
            const std::shared_ptr<GoalHandleCartPoseSet>& goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CartesianPositionController::cart_pose_accepted(const std::shared_ptr<GoalHandleCartPoseSet>& goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&CartesianPositionController::execute, this, _1), goal_handle}.detach();
    }

    void CartesianPositionController::execute(const std::shared_ptr<GoalHandleCartPoseSet>& goal_handle)
    {
        geometry_msgs::msg::Pose goal = goal_handle->get_goal()->target;
        auto feedback = std::make_shared<CartPoseSet::Feedback>();
        auto& sequence = feedback->current;
        auto result = std::make_shared<CartPoseSet::Result>();

        position_d_[0] = goal.position.x;
        position_d_[1] = goal.position.y;
        position_d_[2] = goal.position.z;
        orientation_d_ = {goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z};

        if(pose_error_compute()){
            while(motion_generator.status==2)
            {
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            RCLCPP_INFO(this->get_logger(), "Controller starts!");
            while(motion_generator.status!=2)
            {
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Execute the path successfully!");
        }
        else{
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Target is identical to current pose, ignored!");
        }

    }
}



