# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(franka_action_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)

set(MOVEIT_DEPEND
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
        )
add_executable(moveit_control moveit_control.cpp)
ament_target_dependencies(moveit_control
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        franka_action_interface
        )
target_link_libraries(moveit_control
        franka_client_lib franka_utils_lib)

install(TARGETS
        moveit_control
        DESTINATION lib/${PROJECT_NAME}
        )
