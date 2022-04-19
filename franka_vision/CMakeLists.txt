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

set(THIS_PACKAGE_INCLUDE_DIRS
        include
        )
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        rclcpp
        rclcpp_components
        Eigen3
        Boost)


install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

# The sub-modules
add_subdirectory(src/aruco)


ament_package()
