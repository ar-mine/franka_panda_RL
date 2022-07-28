find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(RapidJSON REQUIRED)

set(ROS2_EXTEND_DEPEND
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs)
set(CV_DEPEND
        OpenCV
        cv_bridge
        )

add_executable(demo_hand_runner demo_hand_runner.cpp)

target_include_directories(
        demo_hand_runner
        PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
)
# demo_runner
ament_target_dependencies(demo_hand_runner
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${ROS2_EXTEND_DEPEND}
        ${CV_DEPEND}
        )
install(TARGETS
        demo_hand_runner
        DESTINATION lib/${PROJECT_NAME}
        )