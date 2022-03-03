//
// Created by armine on 2/24/22.
//

#ifndef BUILD_GEOMETRY_UTILS_H
#define BUILD_GEOMETRY_UTILS_H

#endif //BUILD_GEOMETRY_UTILS_H

#include <iostream>

#include <geometry_msgs/msg/pose.hpp>

#define PI 3.1415926

void posePrint(geometry_msgs::msg::Pose_<std::allocator<void>>& pose);

void deg2rad(std::vector<double> &deg);