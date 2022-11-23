cmake_minimum_required(VERSION 3.5)
project(am_franka_controllers)

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
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Franka REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(franka_interface REQUIRED)

############### Library #################
add_library(
        ${PROJECT_NAME}
        SHARED
        src/controller_base.cpp
        src/cartesian_impedance_controller.cpp
        src/cartesian_position_controller.cpp
)
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(
        ${PROJECT_NAME}
        rclcpp
        rclcpp_action
        geometry_msgs
        kdl_parser
        Franka
        franka_interface
)

############### Executable #################
add_executable(controller_runner src/controller_runner.cpp)
target_include_directories(
        controller_runner
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(
        controller_runner
)
target_link_libraries(controller_runner
        ${PROJECT_NAME})
install(TARGETS
        controller_runner
        DESTINATION lib/${PROJECT_NAME}
        )

############### Misc #################
install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)
install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
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
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
)

ament_package()
