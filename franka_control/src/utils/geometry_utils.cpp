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

void pose_multiply(geometry_msgs::msg::Pose& pose_m, geometry_msgs::msg::Pose& pose_t)
{
    geometry_msgs::msg::PoseStamped pose_tmp;
    pose_tmp.pose = pose_m;

    geometry_msgs::msg::TransformStamped transform_tmp;
    transform_tmp.transform.translation.x = pose_t.position.x;
    transform_tmp.transform.translation.y = pose_t.position.y;
    transform_tmp.transform.translation.z = pose_t.position.z;
    transform_tmp.transform.rotation.x = pose_t.orientation.x;
    transform_tmp.transform.rotation.y = pose_t.orientation.y;
    transform_tmp.transform.rotation.z = pose_t.orientation.z;
    transform_tmp.transform.rotation.w = pose_t.orientation.w;

    tf2::doTransform(pose_tmp, pose_tmp, transform_tmp);

    pose_m = pose_tmp.pose;
}

bool transform_get(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Logger& logger,
                   const std::string& toFrameRel, const std::string& fromFrameRel, geometry_msgs::msg::Pose& pose)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer->lookupTransform(
                toFrameRel, fromFrameRel,
                tf2::TimePointZero,
                std::chrono::seconds (1));
    } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(
                logger, "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return false;
    }
    pose.position.x = transformStamped.transform.translation.x;
    pose.position.y = transformStamped.transform.translation.y;
    pose.position.z = transformStamped.transform.translation.z;

    pose.orientation.x = transformStamped.transform.rotation.x;
    pose.orientation.y = transformStamped.transform.rotation.y;
    pose.orientation.z = transformStamped.transform.rotation.z;
    pose.orientation.w = transformStamped.transform.rotation.w;

    return true;
}

bool transform_get(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Logger& logger,
                   const std::string& toFrameRel, const std::string& fromFrameRel, geometry_msgs::msg::TransformStamped& pose)
{
    try {
        pose = tf_buffer->lookupTransform(
                toFrameRel, fromFrameRel,
                tf2::TimePointZero,
                std::chrono::seconds (1));
    } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(
                logger, "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return false;
    }
    return true;
}