cmake_minimum_required(VERSION 3.5)
project(franka_vision)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# For different compiler
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Public dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(franka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
        include
        ${colcon_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(
        ${PROJECT_NAME} SHARED
        src/aruco/aruco_detector.cpp
)
ament_target_dependencies(
        ${PROJECT_NAME}
        rclcpp
        OpenCV
        cv_bridge
        geometry_msgs
        )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

#=========
# install
#=========
#install(DIRECTORY launch
#        DESTINATION share/${PROJECT_NAME}
#        )
install(DIRECTORY include/
        DESTINATION include
        )
install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

ament_package()
