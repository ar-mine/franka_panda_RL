#include "cmath"

#include "rclcpp/node.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class DemoHand : public rclcpp::Node {
public:
    /// Constructor
    explicit DemoHand(const rclcpp::NodeOptions& node_options) : rclcpp::Node("demo_hand", node_options)
    {
        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/demo_hand/target_pose", rclcpp::QoS(1));;
    }

    void run()
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.4;
        pose.position.y = 0.0;
        pose.position.z = 0.5;
        pose.orientation.x = 1.0;
        pose.orientation.w = 0.0;
        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.header.stamp = this->get_clock()->now();
        poseStamped.pose = pose;
        for(size_t i = 0; i<10; i++)
        {
            target_pose_pub_->publish(poseStamped);
            rclcpp::sleep_for(std::chrono::milliseconds (100));

        }


    }

    /// Publisher for target pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

    /// Subscription for hand's relative x_y_z
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr relative_xyz_sub_;
    /// Timer
    rclcpp::TimerBase::SharedPtr timer_;
    /// Tf Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;

    /// Target pose that is used to detect changes
    geometry_msgs::msg::Pose previous_target_pose_;
    /// Tf from effector to end camera
    geometry_msgs::msg::Pose transform_eef2cam;
    /// Tf from camera to base link
    geometry_msgs::msg::Pose transform_cam2base;

private:
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<DemoHand>(node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    node->run();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
