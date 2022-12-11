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
find_package(franka_interface REQUIRED)
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

# utils library
add_library(moveit_utils_lib SHARED
        utils/moveit_utils.cpp
        utils/moveit_base.cpp
        )
ament_target_dependencies(moveit_utils_lib
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        ${FRANKA_DEPEND}
        )
install(TARGETS
        moveit_utils_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

add_executable(moveit_control moveit_control.cpp)
add_executable(moveit_circle moveit_circle.cpp)
add_executable(follow_targets_node follow_targets_node.cpp)
add_executable(hand_process_node hand_process_node.cpp)

add_executable(round_views demo_hand/round_views.cpp)
add_executable(demo_hand demo_hand/command_executor.cpp)

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
        franka_client_lib franka_utils_lib moveit_utils_lib)
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
        franka_interface
        )
target_link_libraries(round_views
        moveit_utils_lib franka_utils_lib)
install(TARGETS
        round_views
        DESTINATION lib/${PROJECT_NAME}
        )

# follow_targets
ament_target_dependencies(follow_targets_node
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        ${CV_DEPEND}
        franka_interface
        )
target_link_libraries(follow_targets_node
        moveit_utils_lib franka_utils_lib)
install(TARGETS
        follow_targets_node
        DESTINATION lib/${PROJECT_NAME}
        )

# hand_process_node
ament_target_dependencies(hand_process_node
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        ${CV_DEPEND}
        franka_interface
        )
target_link_libraries(hand_process_node
        moveit_utils_lib franka_utils_lib)
install(TARGETS
        hand_process_node
        DESTINATION lib/${PROJECT_NAME}
        )

# demo_hand
ament_target_dependencies(demo_hand
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${MOVEIT_DEPEND}
        ${CV_DEPEND}
        am_robot_utils
        )
target_link_libraries(demo_hand
        moveit_utils_lib franka_utils_lib)
install(TARGETS
        demo_hand
        DESTINATION lib/${PROJECT_NAME}
        )