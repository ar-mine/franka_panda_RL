# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(franka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)

set(ARUCO_DEPEND
        tf2_geometry_msgs
        pluginlib
        control_msgs
        rviz_visual_tools
        tf2
        tf2_ros
        geometry_msgs
        OpenCV
        cv_bridge
    )


#add_executable(aruco_detector_node aruco_detector_node.cpp)
#add_executable(boxes_detector_node boxes_detector_node.cpp)
add_library(aruco_detector_component SHARED
        aruco_detector.cpp
        )

include_directories(
        ${THIS_PACKAGE_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(aruco_detector_component
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${ARUCO_DEPEND}
        )
#ament_target_dependencies(aruco_detector_node
#        ${THIS_PACKAGE_INCLUDE_DEPENDS}
#        ${ARUCO_DEPEND}
#        )
#ament_target_dependencies(boxes_detector_node
#        ${THIS_PACKAGE_INCLUDE_DEPENDS}
#        ${ARUCO_DEPEND}
#        )

rclcpp_components_register_nodes(aruco_detector_component "franka_vision::ArucoDetector")

#target_link_libraries(aruco_detector_node
#        aruco_detector_component)
#target_link_libraries(boxes_detector_node
#        aruco_detector_component)

install(TARGETS
        aruco_detector_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

