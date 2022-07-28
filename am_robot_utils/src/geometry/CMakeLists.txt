find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(
    GEOMETRY_DEPENDS
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

add_library(
        ${PROJECT_NAME}_geometry SHARED
        calculation.cpp
        )
ament_target_dependencies(
        ${PROJECT_NAME}_geometry
        ${PUBLIC_DEPENDS}
        ${GEOMETRY_DEPENDS}
        )
