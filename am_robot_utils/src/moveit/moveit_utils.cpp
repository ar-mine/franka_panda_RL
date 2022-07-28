//
// Created by armine on 2/24/22.
//

#include "am_robot_utils/moveit/moveit_utils.h"

bool Am::moveit_move_ready(moveit::planning_interface::MoveGroupInterface& move_group,
                       rclcpp::Logger& LOGGER)
{
    move_group.setJointValueTarget(std::vector<double>({0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854}));

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Move to ready pose %s", success ? "SUCCESS" : "FAILED");

    if (success)
        move_group.move();

    return success;
}

void Am::add_collision_full(moveit::planning_interface::MoveGroupInterface& move_group,
                   moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.0;
    primitive.dimensions[primitive.BOX_Y] = 2.0;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    collision_object.id = "world_plane1";

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.6;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.1;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    primitive.dimensions[primitive.BOX_X] = 0.6;
    primitive.dimensions[primitive.BOX_Y] = 1.0;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.2;
    box_pose.position.y = 0.6;
    box_pose.position.z = -0.05;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.2;
    box_pose.position.y = -0.6;
    box_pose.position.z = -0.05;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

//    RCLCPP_INFO(LOGGER, "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
}

void Am::add_collision_half(moveit::planning_interface::MoveGroupInterface& move_group,
                        moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.0;
    primitive.dimensions[primitive.BOX_Y] = 2.0;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    collision_object.id = "world_plane1";

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.6;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.08;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

//    RCLCPP_INFO(LOGGER, "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
}