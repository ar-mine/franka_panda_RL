#include "am_robot_utils/geometry/calculation.h"

void Am::posePrint(geometry_msgs::msg::Pose_<std::allocator<void>>& pose)
{
    std::cout<<"position.x:"<<pose.position.x<<"  "
             <<"position.y:"<<pose.position.y<<"  "
             <<"position.z:"<<pose.position.z<<std::endl
             <<"orientation.x:"<<pose.orientation.x<<"  "
             <<"orientation.y:"<<pose.orientation.y<<"  "
             <<"orientation.z:"<<pose.orientation.z<<"  "
             <<"orientation.w:"<<pose.orientation.w<<std::endl;
}

void Am::deg2rad(std::vector<double> &deg)
{
    for (double & i : deg)
    {
        i *= (PI/180);
    }
}

std::vector<double> Am::deg2rad(std::vector<double> deg)
{
    for (double & i : deg)
    {
        i *= (PI/180);
    }
    return deg;
}

double Am::deg2rad(double deg)
{
    deg *= (PI/180);
    return deg;
}

void Am::quaternion_to_orientation(tf2::Quaternion& quaternion_, geometry_msgs::msg::Pose& pose_)
{
    pose_.orientation.x = quaternion_.x();
    pose_.orientation.y = quaternion_.y();
    pose_.orientation.z = quaternion_.z();
    pose_.orientation.w = quaternion_.w();
}

bool Am::transform_multi_get(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Logger& logger,
                       const std::string& toFrameRel, const std::string& fromFrameRel, geometry_msgs::msg::Pose& pose, int times)
{
    for(int i=0; i<times; i++)
    {
        if(Am::transform_get(tf_buffer, logger, toFrameRel, fromFrameRel, pose))
            return true;
    }
    return false;
}

bool Am::transform_get(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Logger& logger,
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

bool Am::transform_get(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const rclcpp::Logger& logger,
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

std::array<double, 4> Am::hamilton_product(const std::array<double, 4>& q, const std::array<double, 4>& r)
{
    // It actually represents " r x q ".
    return(std::array<double, 4>{
            r[3] * q[0] + r[0] * q[3] - r[1] * q[2] + r[2] * q[1],
            r[3] * q[1] + r[0] * q[2] + r[1] * q[3] - r[2] * q[0],
            r[3] * q[2] - r[0] * q[1] + r[1] * q[0] + r[2] * q[3],
            r[3] * q[3] - r[0] * q[0] - r[1] * q[1] - r[2] * q[2]
    });
}

std::array<double, 4> Am::quaternion_product(std::array<double, 4>& q, std::array<double, 4>& r)
{
    // The result calculated by hamilton product should be normalized.
    std::array<double, 4> res = hamilton_product(q, r);
    double factor = sqrt(pow(res[0], 2) + pow(res[1], 2) + pow(res[2], 2) + pow(res[3], 2));
    return (std::array<double, 4>{
            res[0]/factor, res[1]/factor, res[2]/factor, res[3]/factor
    });
}

std::array<double, 4> Am::point_quaternion_rotate(std::array<double, 3>& t, std::array<double, 4>& q)
{
    std::array<double, 4> t_tmp = {t[0], t[1], t[2], 0.0};
    std::array<double, 4> q_conj = {-1*q[0],-1*q[1],-1*q[2], q[3]};
    return hamilton_product(hamilton_product(q, t_tmp), q_conj);
}

void Am::pose_transform(geometry_msgs::msg::Pose& pose_base, geometry_msgs::msg::Pose& pose_delta)
{
    std::array<double, 3> t2 = {pose_delta.position.x, pose_delta.position.y, pose_delta.position.z};
    std::array<double, 4> q1 = {pose_base.orientation.x, pose_base.orientation.y, pose_base.orientation.z, pose_base.orientation.w};
    std::array<double, 4> q2 = {pose_delta.orientation.x, pose_delta.orientation.y, pose_delta.orientation.z, pose_delta.orientation.w};
    std::array<double, 4> t_delta = Am::point_quaternion_rotate(t2, q1);
    std::array<double, 4> qq = Am::quaternion_product(q1, q2);
    pose_base.position.x += t_delta[0];
    pose_base.position.y += t_delta[1];
    pose_base.position.z += t_delta[2];
    pose_base.orientation.x = qq[0];
    pose_base.orientation.y = qq[1];
    pose_base.orientation.z = qq[2];
    pose_base.orientation.w = qq[3];
}
