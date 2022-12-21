cmake_minimum_required(VERSION 3.5)
project(moveit_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Franka REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_interface REQUIRED)
find_package(am_franka_controllers REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        Eigen3
        Boost
        ament_cmake
        rclcpp
        rclcpp_action
        moveit_core
        moveit_ros_planning
        rviz_visual_tools
        moveit_visual_tools
        geometric_shapes
        pluginlib
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        )

add_executable(moveit_cpp_base src/moveit_cpp_base.cpp)
target_include_directories(moveit_cpp_base PUBLIC include)
ament_target_dependencies(moveit_cpp_base  ${THIS_PACKAGE_INCLUDE_DEPENDS} Franka franka_msgs franka_interface am_franka_controllers)
install(TARGETS moveit_cpp_base
        DESTINATION lib/${PROJECT_NAME}
        )

install(
        DIRECTORY config launch rviz
        DESTINATION share/${PROJECT_NAME}
)
install(
        DIRECTORY include/
        DESTINATION include
)
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

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        rclcpp
        rclcpp_lifecycle
)

ament_package()
