//
// Created by armine on 4/18/22.
//

#include "franka_vision/aruco/aruco_detector.h"


franka_vision::ArucoDetector::ArucoDetector(const std::string& node_name,
                                            const std::string& img_tpc_in,
                                            const std::string& img_tpc_out) : Node(node_name)
{
    this->declare_parameter("visualize_flag", 1);
    this->declare_parameter("tf_flag", 1);
    this->declare_parameter("marker_flag", 1);

    this->declare_parameter("prefix", "");
    this->declare_parameter("link_name", "camera_color_optical_frame");
    this->declare_parameter("marker_size", 0.018);

    if(this->get_parameter("tf_flag").get_parameter_value().get<int>())
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 3, std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1)
    );
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/detected_shape", 3
    );
    markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/markers", 1
    );


}

void franka_vision::ArucoDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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
        aruco_info_vec.push_back(m);
    }
    std::sort(aruco_info_vec.begin(), aruco_info_vec.end(), [](const ArucoInfo& a, const ArucoInfo& b)
    {
        return(a.id < b.id);
    });

    // Publish for visualization
    cv_ptr->image = image;
    image_publisher_->publish(*cv_ptr->toImageMsg());
}
