#include "franka_vision/aruco/box_detector.h"

namespace franka_vision {
    BoxDetector::BoxDetector(const std::string &node_name,
                  const std::string &img_tpc_in,
                  const std::string &img_tpc_out) :
            ArucoDetector(node_name, img_tpc_in, img_tpc_out)
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&BoxDetector::timer_callback, this));
        image_publisher_extra = this->create_publisher<sensor_msgs::msg::Image>(
                this->img_tpc_out+"_extra", rclcpp::QoS(10)
        );
        RCLCPP_INFO(this->logger, node_name+" wrapper initialization is completed!");
    }

    void BoxDetector::timer_callback()
    {
//        RCLCPP_INFO(this->logger, "Time start!");

        auto idx_list_ = this->idx_list;
        auto corner_list_ = this->corner_list;
        auto image = this->img_out;
        if(image.empty())
            return;
        if(this->idx_list.size() != 4)
            return;
//        RCLCPP_INFO(this->logger, "Image is not empty!");

        // Draw the rectangle
        std::vector<cv::Point> markers_center;
        cv_bridge::CvImagePtr cv_ptr;
        for(const auto& m : corner_list_)
        {
            float x = 0, y = 0;
            int l = m.size();
            for(const auto& p : m)
            {
                x += p.x;
                y += p.y;
            }
            markers_center.emplace_back(x/l, y/l);
        }
        std::vector<std::vector<cv::Point>> contours = {markers_center};
        cv::drawContours(image, contours, 0, cv::Scalar(255, 0, 0), 2);
//        RCLCPP_INFO(this->logger, "Draw is completed!");

        // Detect the boxes
        shape_detector(image);

        // Publish for visualization
        sensor_msgs::msg::Image::SharedPtr img_msg =
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_publisher_extra->publish(*img_msg);
//        RCLCPP_INFO(this->logger, "Publisher is completed!");

    }

    void BoxDetector::shape_detector(cv::Mat& image)
    {
        auto image_ = image.clone();
        cv::cvtColor(image_, image_, cv::COLOR_BGR2GRAY);


    }
}
