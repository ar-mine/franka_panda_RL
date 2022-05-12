//
// Created by armine on 5/11/22.
//

#ifndef BUILD_MOVEIT_UTILS_H
#define BUILD_MOVEIT_UTILS_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

bool moveit_move_ready(moveit::planning_interface::MoveGroupInterface& move_group,
                       rclcpp::Logger& LOGGER);

void add_collision(moveit::planning_interface::MoveGroupInterface& move_group,
                   moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

#endif //BUILD_MOVEIT_UTILS_H
