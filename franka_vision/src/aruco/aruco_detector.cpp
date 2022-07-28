//
// Created by armine on 4/18/22.
//

#include "franka_vision/aruco/aruco_detector.h"

namespace franka_vision
{
    ArucoDetector::ArucoDetector(const std::string& node_name,
                                 const std::string& img_tpc_in,
                                 const std::string& img_tpc_out) :
    Node(node_name)
    {
        this->node_name = node_name;
        this->img_tpc_in = img_tpc_in;
        this->img_tpc_out = node_name+"/"+img_tpc_out;

//        this->declare_parameter("visualize_flag", 1);
//        this->declare_parameter("tf_flag", 1);
//        this->declare_parameter("marker_flag", 1);
//
//        this->declare_parameter("prefix", "");
//        this->declare_parameter("link_name", "camera_color_optical_frame");
//
//        if(this->get_parameter("tf_flag").get_parameter_value().get<int>())
//        {
//            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//        }

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                this->img_tpc_in, rclcpp::SensorDataQoS(),
                std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1)
        );
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                this->img_tpc_out, rclcpp::QoS(10)
        );
//        markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
//                "/markers", rclcpp::QoS(10)
//        );

        RCLCPP_INFO(logger, node_name+" initialization is completed!");

    }

    void ArucoDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Clone input image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(logger, "cv_bridge exception: %s", e.what());
            return;
        }
        auto image = cv_ptr->image.clone();

        // Aruco markers detection
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(image, this->dictionary, corners, ids,
                                 this->parameters, rejected);
        cv::aruco::drawDetectedMarkers(image, corners, ids);

        // Sort to make it in small-to-large order
        std::vector<ArucoInfo> aruco_info_vec;
        ArucoInfo m;
        for(size_t i=0; i<ids.size(); i++)
        {
            m.id = ids[i];
            m.corner = corners[i];
            aruco_info_vec.emplace_back(m);
        }
        std::sort(aruco_info_vec.begin(), aruco_info_vec.end(), [](const ArucoInfo& a, const ArucoInfo& b)
        {
            return(a.id < b.id);
        });

        // Publish results
        this->idx_list.clear();
        this->corner_list.clear();
        this->total_num = aruco_info_vec.size();
        for(const auto& marker : aruco_info_vec)
        {
            this->idx_list.emplace_back(marker.id);
            this->corner_list.emplace_back(marker.corner);
        }
        this->img_in = cv_ptr->image.clone();
        this->img_out = image.clone();

        // Publish for visualization
        sensor_msgs::msg::Image::SharedPtr img_msg =
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_publisher_->publish(*img_msg);
    }
}

