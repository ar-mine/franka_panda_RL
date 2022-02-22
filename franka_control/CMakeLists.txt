cmake_minimum_required(VERSION 3.5)
project(franka_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
        include/
        )

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        rclcpp
        rclcpp_action
        tf2_geometry_msgs
        tf2_ros
        moveit_core
        moveit_ros_planning_interface
        interactive_markers
        tf2_geometry_msgs
        moveit_ros_planning
        pluginlib
        Eigen3
        Boost
        control_msgs
        moveit_servo
        )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_executable(moveit_control
        src/moveit_control.cpp)
target_include_directories(moveit_control
        PUBLIC include)
ament_target_dependencies(moveit_control
        ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(TARGETS moveit_control
        DESTINATION lib/${PROJECT_NAME}
        )
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
