//
// Created by armine on 12/12/22.
//

#ifndef BUILD_MOVEIT_CPP_BASE_H
#define BUILD_MOVEIT_CPP_BASE_H

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

namespace am_franka_controllers{

class MoveitCppBase : public rclcpp::Node {
public:
    explicit MoveitCppBase(const std::string& node_name, const rclcpp::NodeOptions& node_options);
};

}

#endif //BUILD_MOVEIT_CPP_BASE_H
