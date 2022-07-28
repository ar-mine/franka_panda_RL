#include <ctgmath>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "am_robot_utils/moveit/moveit_base.h"
#include "am_robot_utils/geometry/calculation.h"
#include <boost/filesystem.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_control.nerf_pipeline.round_move");

void corn_points_calculation(std::vector<double> position, double angel,
                             double distance, std::vector<geometry_msgs::msg::Pose>& out, int factor)
{
    geometry_msgs::msg::Pose pose_;

    const double angel_rad = Am::deg2rad(angel);
    const double angel_delta = 2*PI/factor;
    double alpha;

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

        pose_.position.x = position[0] - distance * cos(angel_rad) * cos(alpha);
        pose_.position.y = position[1] + distance * cos(angel_rad) * sin(alpha);
        pose_.position.z = position[2] + distance * sin(angel_rad);

        tf2::Quaternion quaternion_;
        quaternion_.setRPY(PI, Am::deg2rad(angel-90),-alpha);
        quaternion_ = quaternion_.normalize();
        Am::quaternion_to_orientation(quaternion_, pose_);

//        posePrint(pose_);

        out.emplace_back(pose_);

    }
}

class Runner : public Am::MoveitBase
{
public:
    explicit Runner(const rclcpp::NodeOptions& node_options)
    : MoveitBase("round_move", node_options, 1)
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

        this->get_parameter("position", position);
        this->get_parameter("angel", angel);
        this->get_parameter("distance", distance);
        this->get_parameter("factor", factor);
        this->get_parameter("save_path", save_path);
        boost::filesystem::create_directories(save_path.c_str());

        // Sleep to ensure system stable
        rclcpp::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(LOGGER, "Node initialization successful.");
    }

    bool run()
    {
        moveit_move_ready(LOGGER);

        // ****** Calculate the points and move to them ******
        std::vector<geometry_msgs::msg::Pose> waypoints;
        // Pre-defined pose to ensure object is in the center of camera
        corn_points_calculation(position, 90, 0.30, waypoints, 1);
        // Poses assigned by parameters
        for(size_t i = 0; i < angel.size(); i++)
            corn_points_calculation(position, angel[i], distance[i], waypoints, int(factor[i]));
        // Get the static transform between tcp and camera
        geometry_msgs::msg::Pose transform_cam;
        if(!Am::transform_get(tf_buffer, LOGGER, "camera_color_optical_frame", "panda_hand_tcp", transform_cam))
        {
            // If transform cannot be gotten, shutdown.
            return false;
        }
        // Follow the points calculated
        int count = 0;
        bool run_flag = false;
        for(auto point : waypoints)
        {
            // Post-calculate
            // Transform to get object pose
            Am::pose_transform(point, transform_cam);
            // Move
            move_group_.setPoseTarget(point, "panda_hand_tcp");
            move_group_.move();
            std::string num_str = std::to_string(count);
            num_str = std::string(3-num_str.length(), '0') + num_str;
            while(!run_flag)
            {
                this->get_parameter("run_flag", run_flag);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
            cv::imwrite(save_path+num_str+".png", image);
            count++;
        }

        // Move to home
        moveit_move_ready(LOGGER);

        return true;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
private:
    cv::Mat image;

    // Ros parameters
    std::vector<double> position, angel, distance, factor;
    std::string save_path;

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

    bool ret = node->run();
    if(!ret)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}