#include <ctgmath>
#include "am_robot_utils/moveit/moveit_base.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_control.layout_copy_demo");


class Runner : public Am::MoveitBase
{
public:
    explicit Runner(const rclcpp::NodeOptions& node_options)
            : MoveitBase("layout_copy_demo", node_options, 1)
    {
        translation_subscription_ = this->create_subscription<std_msg::msg::Float64MultiArray>(
                "/ShapeDetector/MarkerDetector/translation", 3,
                [this](std_msg::msg::Float64MultiArray msg)
                {
                    ;
                });

        // Sleep to ensure system stable
        rclcpp::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(LOGGER, "Node initialization successful.");
    }

    void run()
    {
        moveit_move_ready(LOGGER);


    }

    rclcpp::Subscription<std_msg::msg::Float64MultiArray>::SharedPtr translation_subscription_;
    rclcpp::Subscription<std_msg::msg::Float64MultiArray>::SharedPtr box_info_subscription_;

private:

};



int main(int argc, char **argv) {
    // Node init
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    const std::shared_ptr<Runner> node = std::make_shared<Runner>(node_options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    node->run();

    rclcpp::shutdown();
    return 0;
}