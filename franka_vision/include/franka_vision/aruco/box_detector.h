//
// Created by armine on 5/22/22.
//

#ifndef BUILD_BOX_DETECTOR_H
#define BUILD_BOX_DETECTOR_H

#include "franka_vision/aruco/aruco_detector.h"

namespace franka_vision {
    class BoxDetector : public ArucoDetector
    {
    public:
        BoxDetector(const std::string &node_name, const std::string &img_tpc_in, const std::string &img_tpc_out);

        void shape_detector(cv::Mat& image);

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_extra;

    private:
        void timer_callback();

        rclcpp::TimerBase::SharedPtr timer_;
    };
}

#endif //BUILD_BOX_DETECTOR_H
