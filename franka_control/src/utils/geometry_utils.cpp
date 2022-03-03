//
// Created by armine on 2/24/22.
//

#include "franka_control/geometry_utils.h"

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
