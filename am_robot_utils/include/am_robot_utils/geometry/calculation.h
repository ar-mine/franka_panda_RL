//
// Created by armine on 6/16/22.
//

#ifndef BUILD_CALCULATION_H
#define BUILD_CALCULATION_H

#include "am_robot_utils/geometry/geometry_base.h"

namespace Am {
    #define PI 3.1415926

    void posePrint(geometry_msgs::msg::Pose_<std::allocator<void>>& pose);

    void deg2rad(std::vector<double> &deg);

    std::vector<double> deg2rad(std::vector<double> deg);

    double deg2rad(double deg);

    void quaternion_to_orientation(tf2::Quaternion& quaternion_, geometry_msgs::msg::Pose& pose_);

    bool transform_get(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Logger& logger,
                       const std::string& toFrameRel, const std::string& fromFrameRel, geometry_msgs::msg::Pose& pose);

    bool transform_get(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Logger& logger,
                       const std::string& toFrameRel, const std::string& fromFrameRel, geometry_msgs::msg::TransformStamped& pose);

    std::array<double, 4> hamilton_product(const std::array<double, 4>& q, const std::array<double, 4>& r);

    std::array<double, 4> quaternion_product(std::array<double, 4>& q, std::array<double, 4>& r);

    std::array<double, 4> point_quaternion_rotate(std::array<double, 3>& t, std::array<double, 4>& q);

    void pose_transform(geometry_msgs::msg::Pose& pose_base, geometry_msgs::msg::Pose& pose_delta);
}

#endif //BUILD_CALCULATION_H
