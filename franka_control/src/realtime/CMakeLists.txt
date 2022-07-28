find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(Franka REQUIRED)
find_package(franka_msgs REQUIRED)
set(FRANKA_DEPEND
        Franka
        franka_msgs
        )

find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)
set(CV_DEPEND
        OpenCV
        cv_bridge
        )
# Library
add_library(motion_generator
        SHARED motion_generator.cpp)
ament_target_dependencies(motion_generator
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${FRANKA_DEPEND}
        )
install(TARGETS
        motion_generator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

# Executable
add_executable(hand_follow_node hand_follow_node.cpp)

target_include_directories(
        hand_follow_node
        PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

# hand_follow_node
ament_target_dependencies(hand_follow_node
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
        ${CV_DEPEND}
        ${FRANKA_DEPEND}
        )
target_link_libraries(hand_follow_node
        franka_client_lib franka_utils_lib motion_generator)
install(TARGETS
        hand_follow_node
        DESTINATION lib/${PROJECT_NAME}
        )
