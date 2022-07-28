//
// Created by armine on 4/18/22.
//

#ifndef BUILD_ARUCO_DETECTOR_H
#define BUILD_ARUCO_DETECTOR_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/aruco.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace franka_vision
{
    typedef visualization_msgs::msg::Marker Marker;
    typedef visualization_msgs::msg::MarkerArray MarkerArray;

    class ArucoDetector : public rclcpp::Node
    {
    public:
        ArucoDetector(const std::string& node_name, const std::string& img_tpc_in, const std::string& img_tpc_out);

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        // Input parameters
        std::string node_name;
        std::string img_tpc_in;
        std::string img_tpc_out;

        // Logger
        rclcpp::Logger logger = this->get_logger();
        // Subscription && Publisher
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;

        // The ros parameters can be assigned in yaml file
        float aruco_size = 0.018;
        cv::Mat camera_k = (cv::Mat_<double>(3,3) <<
                1359.4664306640625, 0.0, 955.1310424804688,
                0.0, 1358.664794921875, 528.2626342773438,
                0.0, 0.0, 1.0);
        cv::Mat camera_d = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

        /* The variables exposed to show the results of detector(Also be published to ROS sever).
         *
         * */
        unsigned long total_num = 0;
        std::vector<int> idx_list;
        std::vector<std::vector<cv::Point2f>> corner_list;
        cv::Mat img_in, img_out;

    private:
        // TF
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        // Detection parameters
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        // Aruco Info Structure
        struct ArucoInfo
        {
            std::vector<cv::Point2f> corner;
            int id = 0;
        };
    };
}


#endif //BUILD_ARUCO_DETECTOR_H
