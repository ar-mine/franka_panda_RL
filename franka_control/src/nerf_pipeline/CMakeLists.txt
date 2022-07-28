# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Franka REQUIRED)
find_package(franka_msgs REQUIRED)
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
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(RapidJSON REQUIRED)
find_package(franka_action_interface REQUIRED)
find_package(am_robot_utils REQUIRED)

set(FRANKA_DEPEND
        Franka
        franka_msgs
        )

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
        tf2
        tf2_ros
        geometry_msgs
        )

set(CV_DEPEND
        OpenCV
        cv_bridge
        )

set(
        EXECUTABLE
        round_move
        pick_place
)

add_executable(round_move round_move.cpp)
add_executable(pick_place pick_place.cpp)

# round_move
ament_target_dependencies(round_move
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        ${CV_DEPEND}
        am_robot_utils
        )
# pick_place
ament_target_dependencies(pick_place
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        ${CV_DEPEND}
        am_robot_utils
        )

install(TARGETS
        ${EXECUTABLE}
        DESTINATION lib/${PROJECT_NAME}
        )