#include "franka_vision/aruco/aruco_detector.h"
#include "franka_vision/aruco/box_detector.h"

int main(int argc, char **argv)
{
    // Node init
    rclcpp::init(argc, argv);
//    auto aruco_detector_node = std::make_shared<franka_vision::ArucoDetector>("aruco_detector",
//                                                                              "/camera/color/image_raw",
//                                                                              "markers");
    auto aruco_detector_node = std::make_shared<franka_vision::BoxDetector>("aruco_detector",
                                                                              "/camera/color/image_raw",
                                                                              "markers");
////    // Execute node thread
////    rclcpp::executors::MultiThreadedExecutor executor;
////    executor.add_node(aruco_detector_node);
////    std::thread([&executor]() { executor.spin(); }).detach();
//
    rclcpp::spin(aruco_detector_node);
    rclcpp::shutdown();
    return 0;
}
