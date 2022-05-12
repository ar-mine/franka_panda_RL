//
// Created by armine on 2/24/22.
//

#ifndef BUILD_GEOMETRY_UTILS_H
#define BUILD_GEOMETRY_UTILS_H

#endif //BUILD_GEOMETRY_UTILS_H

#include <iostream>

#include "geometry_msgs/msg/pose.hpp"

#define PI 3.1415926

void posePrint(geometry_msgs::msg::Pose_<std::allocator<void>>& pose);

void deg2rad(std::vector<double> &deg);

std::vector<double> deg2rad(std::vector<double> deg);

double deg2rad(double deg);

void quaternion_to_orientation(tf2::Quaternion& quaternion_, geometry_msgs::msg::Pose& pose_);
