find_package(moveit_common REQUIRED)
moveit_package()

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(Franka REQUIRED)
find_package(franka_msgs REQUIRED)

set(
        MOVEIT_DEPEND
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_visual_tools
        moveit_servo
        interactive_markers
        tf2_geometry_msgs
        pluginlib
        control_msgs
        rviz_visual_tools
        tf2
        tf2_ros
        geometry_msgs
        )
set(FRANKA_DEPEND
        Franka
        franka_msgs
        )


add_library(
        ${PROJECT_NAME}_moveit SHARED
        moveit_base.cpp
        moveit_utils.cpp
        )
ament_target_dependencies(
        ${PROJECT_NAME}_moveit
        ${PUBLIC_DEPENDS}
        ${FRANKA_DEPEND}
        ${MOVEIT_DEPEND}
        )
