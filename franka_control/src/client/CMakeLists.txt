find_package(franka_interface REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(franka_client_lib SHARED
        ros2_reachClient.cpp
        )

ament_target_dependencies(franka_client_lib
        franka_interface
        rclcpp_action
        )

install(TARGETS
        franka_client_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)