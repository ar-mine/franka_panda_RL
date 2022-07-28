//
// Created by armine on 2/24/22.
//

#ifndef BUILD_GEOMETRY_UTILS_H
#define BUILD_GEOMETRY_UTILS_H

#endif //BUILD_GEOMETRY_UTILS_H

#include <iostream>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>


#define PI 3.1415926

void posePrint(geometry_msgs::msg::Pose_<std::allocator<void>>& pose);

void deg2rad(std::vector<double> &deg);

std::vector<double> deg2rad(std::vector<double> deg);

double deg2rad(double deg);

void quaternion_to_orientation(tf2::Quaternion& quaternion_, geometry_msgs::msg::Pose& pose_);

void pose_multiply(geometry_msgs::msg::Pose& pose_m, geometry_msgs::msg::Pose& pose_t);

bool transform_get(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Logger& logger,
                   const std::string& toFrameRel, const std::string& fromFrameRel, geometry_msgs::msg::Pose& pose);

bool transform_get(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Logger& logger,
                   const std::string& toFrameRel, const std::string& fromFrameRel, geometry_msgs::msg::TransformStamped& pose);
