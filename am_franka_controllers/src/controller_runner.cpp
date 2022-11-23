//
// Created by armine on 10/25/22.
//
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "am_franka_controllers/cartesian_impedance_controller.h"
#include "am_franka_controllers/cartesian_position_controller.h"


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
//    std::shared_ptr<am_franka_controllers::CartesianImpedanceController> node =
//            std::make_shared<am_franka_controllers::CartesianImpedanceController>("cart_impedance_controller_node", node_options, "172.16.0.2");
    std::shared_ptr<am_franka_controllers::CartesianPositionController> node =
            std::make_shared<am_franka_controllers::CartesianPositionController>("cart_position_controller_node", node_options, "172.16.0.2");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    try{
        executor.spin();
    }
    catch (...)
    {
        RCLCPP_FATAL_STREAM(node->get_logger(), "Detect exception");
    }

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}