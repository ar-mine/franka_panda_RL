//
// Created by armine on 2/24/22.
//

#include <tf2/LinearMath/Quaternion.h>
#include "franka_control/utils/geometry_utils.h"

void posePrint(geometry_msgs::msg::Pose_<std::allocator<void>>& pose)
{
    std::cout<<"position.x:"<<pose.position.x<<"  "
             <<"position.y:"<<pose.position.y<<"  "
             <<"position.z:"<<pose.position.z<<std::endl
             <<"orientation.x:"<<pose.orientation.x<<"  "
             <<"orientation.y:"<<pose.orientation.y<<"  "
             <<"orientation.z:"<<pose.orientation.z<<"  "
             <<"orientation.w:"<<pose.orientation.w<<std::endl;
}

void deg2rad(std::vector<double> &deg)
{
    for (double & i : deg)
    {
        i *= (PI/180);
    }
}

std::vector<double> deg2rad(std::vector<double> deg)
{
    for (double & i : deg)
    {
        i *= (PI/180);
    }
    return deg;
}

double deg2rad(double deg)
{
    deg *= (PI/180);
    return deg;
}

void quaternion_to_orientation(tf2::Quaternion& quaternion_, geometry_msgs::msg::Pose& pose_)
{
    pose_.orientation.x = quaternion_.x();
    pose_.orientation.y = quaternion_.y();
    pose_.orientation.z = quaternion_.z();
    pose_.orientation.w = quaternion_.w();
}