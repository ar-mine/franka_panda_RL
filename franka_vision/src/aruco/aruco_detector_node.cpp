//
// Created by armine on 4/18/22.
//

#include "franka_vision/aruco/aruco_detector.h"

int main(int argc, char **argv)
{
    // Node init
    rclcpp::init(argc, argv);
    auto aruco_detector_node = std::make_shared<franka_vision::ArucoDetector>("aruco_detector_node",
                                                                                       "/camera/color/image_raw",
                                                                                      "/detected_shape");
//    // Execute node thread
//    rclcpp::executors::MultiThreadedExecutor executor;
//    executor.add_node(aruco_detector_node);
//    std::thread([&executor]() { executor.spin(); }).detach();

    rclcpp::spin(aruco_detector_node);
    rclcpp::shutdown();
    return 0;
}
