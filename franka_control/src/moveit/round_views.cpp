//
// Created by armine on 2/21/22.
//
#include <ctgmath>
#include "franka_control/utils/moveit_utils.h"
#include "franka_control/utils/geometry_utils.h"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"

void parameters_init(const std::shared_ptr<rclcpp::Node>& node_,
                     std::vector<double>& position, double& angel, double& distance)
{
    node_->get_parameter("position", position);
    node_->get_parameter("angel", angel);
    node_->get_parameter("distance", distance);
}

void corn_points_calculation(std::vector<double> position, double angel,
                             double distance, std::vector<geometry_msgs::msg::Pose>& out, int factor=8)
{
    geometry_msgs::msg::Pose pose_;

    const double angel_rad = deg2rad(angel);
    const double angel_delta = 2*PI/factor;
    double alpha = 0.0;

    for(int i=0; i<factor; i++)
    {
        pose_.position.x = position[0] - distance * cos(angel_rad) * cos(alpha);
        pose_.position.y = position[1] + distance * cos(angel_rad) * sin(alpha);
        pose_.position.z = position[2] + distance * sin(angel_rad);

        tf2::Quaternion quaternion_;
        quaternion_.setRPY(PI, deg2rad(90.0-angel),PI-alpha);
        quaternion_ = quaternion_.normalize();
        quaternion_to_orientation(quaternion_, pose_);

//        posePrint(pose_);

        out.emplace_back(pose_);

        alpha += angel_delta;
    }
}

int main(int argc, char **argv) {
    cv::Mat image;
    // Node init
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("round_views", node_options);
    rclcpp::Logger logger = move_group_node->get_logger();
    auto subscription_ = move_group_node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 3,
            [&logger, &image](sensor_msgs::msg::Image::SharedPtr msg)
            {
                try
                {
                    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
                }
                catch (cv_bridge::Exception& e)
                {
                    RCLCPP_ERROR(logger, "cv_bridge exception: %s", e.what());
                    return;
                }
            });

    // Run the node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Moveit planning interface init
    static const std::string PLANNING_GROUP = "panda_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    move_group.setPlannerId("RRTConnectkConfigDefault");
    add_collision(move_group, planning_scene_interface);

    // ****************Task Begin*****************
    // Get the parameters
    std::vector<double> position;
    double angel, distance;
    parameters_init(move_group_node, position, angel, distance);

    // Move to home
    moveit_move_ready(move_group, logger);

    // Calculate the points and move to them
    int count = 0;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    corn_points_calculation(position, angel, distance, waypoints);
    for(auto point : waypoints)
    {
        move_group.setPoseTarget(point, "panda_hand_tcp");
        move_group.move();
        cv::imwrite("record/"+std::to_string(count)+".png", image);
        count++;
    }

    // Move to home
    moveit_move_ready(move_group, logger);
    // ****************Task End*****************
    rclcpp::shutdown();
    return 0;
}