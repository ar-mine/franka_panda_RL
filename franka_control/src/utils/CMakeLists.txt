find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(franka_utils_lib SHARED
        geometry_utils.cpp
        )

ament_target_dependencies(franka_utils_lib
        geometry_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        )

install(TARGETS
        franka_utils_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)