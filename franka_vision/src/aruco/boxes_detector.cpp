//#include "franka_vision/aruco/aruco_detector.h"
//
//namespace franka_vision
//{
//    class BoxesDetector
//    {
//    public:
//        BoxesDetector(std::shared_ptr<franka_vision::ArucoDetector> &node)
//        {
//            this->node_ = node;
//        }
//
//        void run()
//        {
//            while(node_->)
//            if(node_->total_num == 4)
//            {
//
//            }
//        }
//    private:
//        std::shared_ptr<franka_vision::ArucoDetector> node_;
//    };
//}
//int main(int argc, char **argv)
//{
//    // Node init
//    rclcpp::init(argc, argv);
//    auto aruco_detector_node = std::make_shared<franka_vision::ArucoDetector>("aruco_detector_node",
//                                                                                       "/camera/color/image_raw",
//                                                                                      "/detected_shape");
//    auto boxes_detector_node = franka_vision::BoxesDetector(aruco_detector_node);
//    // Execute node thread
//    rclcpp::executors::MultiThreadedExecutor executor;
//    executor.add_node(aruco_detector_node);
//    std::thread([&executor]() { executor.spin(); }).detach();
//
//    boxes_detector_node.run();
//
//    rclcpp::shutdown();
//    return 0;
//}
