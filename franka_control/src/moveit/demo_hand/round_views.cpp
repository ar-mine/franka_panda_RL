#include <ctgmath>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "franka_control/utils/moveit_base.h"
#include "franka_control/utils/geometry_utils.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_control.demo_hand.round_views");

void corn_points_calculation(std::vector<double> position, double angel,
                             double distance, std::vector<geometry_msgs::msg::Pose>& out, int factor)
{
    geometry_msgs::msg::Pose pose_;

    const double angel_rad = deg2rad(angel);
    const double angel_delta = 2*PI/factor;
    double alpha = 0.0;

    if(factor == 1)
    {
        pose_.position.x = position[0];
        pose_.position.y = position[1];
        pose_.position.z = position[2] + distance;
        pose_.orientation.x = 1.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
        pose_.orientation.w = 0.0;
        out.emplace_back(pose_);
        return;
    }

    for(int i=1; i<factor; i++)
    {
        alpha = angel_delta*i;
//        if(i==factor/2)
//            continue;
        pose_.position.x = position[0] - distance * cos(angel_rad) * cos(alpha);
        pose_.position.y = position[1] + distance * cos(angel_rad) * sin(alpha);
        pose_.position.z = position[2] + distance * sin(angel_rad);

        tf2::Quaternion quaternion_;
        quaternion_.setRPY(PI, deg2rad(angel-90),-alpha);
        quaternion_ = quaternion_.normalize();
        quaternion_to_orientation(quaternion_, pose_);

//        posePrint(pose_);

        out.emplace_back(pose_);

    }
}

class Runner : public MoveitBase
{
public:
    explicit Runner(const rclcpp::NodeOptions& node_options)
    : MoveitBase("round_views", node_options, 1)
    {
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/color/image_raw", 3,
                [this](sensor_msgs::msg::Image::SharedPtr msg)
                {
                    try
                    {
                        this->image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
                    }
                    catch (cv_bridge::Exception& e)
                    {
                        RCLCPP_ERROR(LOGGER, "cv_bridge exception: %s", e.what());
                        return;
                    }
                });

        // Get the parameters
//        position = {0.55, 0.0, -0.02};
//        angel= {80.0, 70.0, 60.0, 50.0};
//        distance = {0.30, 0.30, 0.30, 0.30};
//        factor = {8.0, 12.0, 16.0, 20.0};
//        run_flag: false
        this->get_parameter("position", position);
        this->get_parameter("angel", angel);
        this->get_parameter("distance", distance);
        this->get_parameter("factor", factor);

        // Sleep to ensure system stable
        rclcpp::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(LOGGER, "Node initialization successful.");
    }

    void run()
    {
        moveit_move_ready(LOGGER);

        // Calculate the points and move to them
        int count = 0;
        bool run_flag = false;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        corn_points_calculation(position, 90, 0.30, waypoints, 1);
        for(size_t i = 0; i < angel.size(); i++)
            corn_points_calculation(position, angel[i], distance[i], waypoints, int(factor[i]));
        geometry_msgs::msg::Pose transform_cam;
        if(!transform_get(tf_buffer, LOGGER, "camera_color_optical_frame", "panda_hand_tcp", transform_cam))
        {
            rclcpp::shutdown();
        }
        for(auto point : waypoints)
        {
            // Post-calculate
            geometry_msgs::msg::Pose target_tmp;
            target_tmp = transform_cam;
            // Transform to get object pose
            pose_multiply(target_tmp, point);

            // Move
            move_group_.setPoseTarget(target_tmp, "panda_hand_tcp");
            move_group_.move();
            std::string num_str = std::to_string(count);
            num_str = std::string(3-num_str.length(), '0') + num_str;
            while(!run_flag)
                this->get_parameter("run_flag", run_flag);
            cv::imwrite("record/input/"+num_str+".png", image);
            count++;
        }

        // Move to home
        moveit_move_ready(LOGGER);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
private:
    cv::Mat image;

    std::vector<double> position, angel, distance, factor;

    std::vector<double> pick_pose;
    std::vector<double> place_pose;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener>transform_listener;
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