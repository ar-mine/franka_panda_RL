cmake_minimum_required(VERSION 3.5)
project(franka_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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
        Boost
        )

install(DIRECTORY launch configs
        DESTINATION share/${PROJECT_NAME}
        )

#add_subdirectory(src/client)
#add_subdirectory(src/moveit)
#add_subdirectory(src/utils)
#add_subdirectory(src/realtime)
#add_subdirectory(src/nerf_pipeline)
#add_subdirectory(src/layout_repeat)
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#endif()
#
#ament_export_dependencies(
#        ${THIS_PACKAGE_INCLUDE_DEPENDS}
#)
#
ament_export_include_directories(include)

ament_package()