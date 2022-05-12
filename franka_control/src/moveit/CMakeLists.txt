# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

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

# utils library
add_library(moveit_utils_lib SHARED
        utils/moveit_utils.cpp
        )
ament_target_dependencies(moveit_utils_lib
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        )
install(TARGETS
        moveit_utils_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

add_executable(moveit_control moveit_control.cpp)
add_executable(moveit_circle moveit_circle.cpp)
add_executable(round_views round_views.cpp)

target_include_directories(
        moveit_control
        PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        tf2_geometry_msgs
)
ament_target_dependencies(moveit_control
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        franka_msgs
        OpenCV
        cv_bridge
        tf2_geometry_msgs
        )
target_link_libraries(moveit_control
        franka_client_lib franka_utils_lib)
install(TARGETS
        moveit_control
        DESTINATION lib/${PROJECT_NAME}
        )

ament_target_dependencies(moveit_circle
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        )
target_link_libraries(moveit_circle
        franka_client_lib franka_utils_lib)
install(TARGETS
        moveit_circle
        DESTINATION lib/${PROJECT_NAME}
        )

# round_views
ament_target_dependencies(round_views
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        ${CV_DEPEND}
        )
target_link_libraries(round_views
        moveit_utils_lib franka_utils_lib)
install(TARGETS
        round_views
        DESTINATION lib/${PROJECT_NAME}
        )

