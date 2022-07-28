cmake_minimum_required(VERSION 3.5)
project(am_robot_utils)

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

set(
        LIBRARIES
        ${PROJECT_NAME}_geometry
        ${PROJECT_NAME}_moveit
)
set(
        PUBLIC_DEPENDS
        rclcpp
)
include_directories(
        include
        ${colcon_INCLUDE_DIRS}
)

add_subdirectory(src/geometry)
add_subdirectory(src/moveit)

#=========
# install
#=========
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )
install(DIRECTORY include/
        DESTINATION include
        )
install(
        TARGETS ${LIBRARIES}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARIES})
#ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
