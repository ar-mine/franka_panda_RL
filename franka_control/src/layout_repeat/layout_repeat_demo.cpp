#include <ctgmath>
#include "am_robot_utils/moveit/moveit_base.h"
#include "am_robot_utils/geometry/calculation.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_control.layout_copy_demo");

struct transInfo
{
    std::array<double, 3> data = {0, 0, 0};
    bool flag = false;
};

struct boxInfo
{
    geometry_msgs::msg::Pose pose;
    std::array<double, 3> translation;
    std::array<double, 4> rotation;
    std::array<double, 2> size;
};

struct boxInfoArray
{
    bool flag = false;
    int length = 0;
    std::vector<boxInfo> data;
};

void move_predefined(moveit::planning_interface::MoveGroupInterface& move_group, const std::string& pose)
{
    std::vector<double> home_joint_positions;
    if(pose == "home")
        home_joint_positions = {0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785};
    else if(pose == "waiting")
        home_joint_positions = {0.80, -0.31, -0.06, -2.38, -0.08, 2.10, 1.58};
    else if(pose == "detect")
        home_joint_positions = {0.50, -0.52, -1.22, -2.17, -0.52, 1.97, 0.33};
    else if(pose == "box1")
        home_joint_positions = {-0.94, 0.79, -0.59, -1.97, 0.82, 2.51, -1.30};
    else if(pose == "box2")
        home_joint_positions = {-1.08, 0.81, -0.61, -1.93, 0.82, 2.46, -1.44};
    else if(pose == "box3")
        home_joint_positions = {-1.19, 0.85, -0.65, -1.89, 0.82, 2.46, -1.56};
    else
        return;

//    deg2rad(home_joint_positions);

    move_group.setJointValueTarget(home_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Move to home pose %s", success ? "" : "FAILED");

    if (success)
        move_group.move();
}

int face_define(double ratio)
{
    // face     A       B       C       wrong
    // define   3       2       1       0
    // width    0.018   0.025   0.019   0
    if(ratio>=2.0 && ratio<=2.5)
        return 3;
    else if(ratio>=1.5 && ratio<2.0)
        return 2;
    else if(ratio>=1.0 && ratio<1.5)
        return 1;
    else
        return 0;
}

class Runner : public Am::MoveitBase
{
public:
    explicit Runner(const rclcpp::NodeOptions& node_options)
            : MoveitBase("layout_repeat_demo", node_options, 1, 0.25, 0.25)
    {
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        this->declare_parameter("next", false);

        translation_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/ShapeDetector/MarkerDetector/translation", 3,
                [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
                {
                    std::vector<double> tmp = msg->data;
                    if(tmp[0] == 1)
                    {
                        this->trans.flag = true;
                        this->trans.data = {tmp[1], tmp[2], tmp[3]};
                    }
                    else
                    {
                        this->trans.flag = false;
                        this->trans.data = {0.0, 0.0, 0.0};
                    }
                });
        box_info_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/ShapeDetector/box_info", 3,
                [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
                {
                    std::vector<double> tmp = msg->data;
                    if(tmp[0] > 0)
                    {
                        this->box_info.flag = true;
                        this->box_info.length = int(tmp[0]);

                        this->box_info.data.clear();
                        boxInfo box_info_tmp;
                        for(size_t i = 0; i < tmp[0]; i++)
                        {
                            box_info_tmp.pose.position.x = tmp[9*i+1];
                            box_info_tmp.pose.position.y = tmp[9*i+2];
                            box_info_tmp.pose.position.z = tmp[9*i+3];
                            box_info_tmp.pose.orientation.x = tmp[9*i+4];
                            box_info_tmp.pose.orientation.y = tmp[9*i+5];
                            box_info_tmp.pose.orientation.z = tmp[9*i+6];
                            box_info_tmp.pose.orientation.w = tmp[9*i+7];

                            box_info_tmp.translation = {tmp[9*i+1], tmp[9*i+2], tmp[9*i+3]};
                            box_info_tmp.rotation = {tmp[9*i+4], tmp[9*i+5], tmp[9*i+6], tmp[9*i+7]};
                            box_info_tmp.size = {tmp[9*i+8], tmp[9*i+9]};

                            box_info.data.push_back(box_info_tmp);
                        }
                    }
                    else
                    {
                        this->box_info.flag = false;
                        this->box_info.length = 0;
                    }
                });
        // Sleep to ensure system stable
        rclcpp::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(LOGGER, "Node initialization successful.");
    }

    void run()
    {
        this->hand_action(true, LOGGER);
        move_predefined(this->move_group_, "waiting");

        while(!this->get_parameter("next").get_parameter_value().get<bool>())
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        for(size_t i = 0; i<10; i++)
        {
            if(this->trans.flag and this->box_info.flag)
            {
                this->trans_saved = this->trans;
                this->box_info_saved = this->box_info;
                for(auto& pose_ : this->box_info_saved.data)
                {
                    pose_.pose.position.x -= this->trans_saved.data[0];
                    pose_.pose.position.y -= this->trans_saved.data[1];
                    pose_.pose.position.z -= this->trans_saved.data[2];
                }
                break;
            }
            else
            {
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
        }


        // Start work loop
//        move_predefined(this->move_group_, "home");
        for(int count=0; count<this->box_info_saved.length; count++)
        {
            // Move to fixed pose
            if(count < 3)
            {
                int face = face_define(this->box_info_saved.data[count].size[1]/this->box_info_saved.data[count].size[0]);
                move_to_box(count, face);

                // Do pose transform
                if(face==1)
                    transform_a2c();
                else if(face==2)
                    transform_a2b();

            }
            else
            {
                continue;
            }

            // Move to predefined detect pose
            move_predefined(this->move_group_, "detect");

            geometry_msgs::msg::Pose target, abs_trans_cam;
            if(Am::transform_get(tf_buffer, LOGGER, "panda_link0", "camera_color_optical_frame", abs_trans_cam))
            {
                // Transform to get object pose
                target = this->box_info_saved.data[count].pose;
                target.position.x += this->trans.data[0];
                target.position.y += this->trans.data[1];
                target.position.z += this->trans.data[2];
                RCLCPP_INFO(LOGGER, "position: %f, %f, %f, orientation: %f, %f, %f, %f"
                        , target.position.x, target.position.y, target.position.z,
                        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);
                Am::pose_transform(abs_trans_cam, target);

                // Move to grasp
                abs_trans_cam.position.z = 0.02;

                this->move_group_.setPoseTarget(abs_trans_cam, "panda_hand_tcp");
                this->move_group_.move();

                this->hand_action(true, LOGGER);

                abs_trans_cam.position.z = 0.15;
                this->move_group_.setPoseTarget(abs_trans_cam, "panda_hand_tcp");
                this->move_group_.move();
            }
            move_predefined(this->move_group_, "detect");
        }
        move_predefined(this->move_group_, "waiting");
    }

    void transform_a2c()
    {
        this->move_group_.setJointValueTarget({-0.622, 0.312, -0.873, -1.930, 0.350, 2.141, 2.270});
        this->move_group_.move();

        this->move_group_.setJointValueTarget({0.600, 1.290, -1.548, -2.440, -0.233, 1.748, 2.642});
        this->move_group_.move();

        this->move_group_.setJointValueTarget({0.350, 1.149, -1.035, -2.364, -0.099, 1.853, -0.117+PI});
        this->move_group_.move();

        this->hand_action(true, LOGGER);

        this->move_group_.setJointValueTarget({0.600, 1.290, -1.548, -2.440, -0.233, 1.748, 2.642});
        this->move_group_.move();

        this->move_group_.setJointValueTarget({0.835, 0.158, -2.060, -2.888, 0.486, 2.789, -0.854+PI});
        this->move_group_.move();

        this->hand_action(false, LOGGER, 0.019);

        this->move_group_.setJointValueTarget({0.593, 0.913, -1.783, -2.789, 1.119, 2.098, -1.401+PI});
        this->move_group_.move();
    }

    void transform_a2b()
    {
        this->move_group_.setJointValueTarget({-0.622, 0.312, -0.873, -1.930, 0.350, 2.141, 2.270-PI/2});
        this->move_group_.move();

        this->move_group_.setJointValueTarget({-1.088, 1.089, -0.732, -1.285, 0.239, 0.693, 0.325});
        this->move_group_.move();

        this->move_group_.setJointValueTarget({-1.095, 1.256, -0.704, -1.276, 0.292, 0.799, 0.323});
        this->move_group_.move();

        this->hand_action(true, LOGGER);

        this->move_group_.setJointValueTarget({-1.088, 1.089, -0.732, -1.285, 0.239, 0.693, 0.325});
        this->move_group_.move();

        this->move_group_.setJointValueTarget({-0.894, -0.249, -0.730, -2.547, -0.214, 2.347, 0.939-PI/2});
        this->move_group_.move();

        this->move_group_.setJointValueTarget({-0.809, 0.226, -0.717, -2.589, 0.358, 2.752, 0.501-PI/2});
        this->move_group_.move();

        this->hand_action(false, LOGGER, 0.018);

        this->move_group_.setJointValueTarget({-0.894, -0.249, -0.730, -2.547, -0.214, 2.347, 0.939-PI/2});
        this->move_group_.move();
    }

    void move_to_box(int idx, int face)
    {
        if(face == 1 || face == 3)
        {
            if(idx == 0)
            {
                this->move_group_.setJointValueTarget({-0.914, 0.491, -0.644, -2.011, 0.414, 2.378, -1.050+PI});
                this->move_group_.move();
                this->move_group_.setJointValueTarget({-0.939, 0.789, -0.582, -1.967, 0.825, 2.543, -1.295+PI});
                this->move_group_.move();
            }
            else if(idx == 1)
            {
                this->move_group_.setJointValueTarget({-0.801, 0.719, -0.904, -2.007, 0.753, 2.375, -1.406+PI});
                this->move_group_.move();
                this->move_group_.setJointValueTarget({-1.054, 0.799, -0.602, -1.969, 0.818, 2.524, -1.404+PI});
                this->move_group_.move();
            }
            else if(idx == 2)
            {
                this->move_group_.setJointValueTarget({-1.002, 0.744, -0.877, -1.951, 0.771, 2.344, -1.517+PI});
                this->move_group_.move();
                this->move_group_.setJointValueTarget({-1.143, 0.904, -0.714, -1.866, 0.928, 2.400, -1.564+PI});
                this->move_group_.move();
            }
            else
            {

            }
            this->hand_action(false, LOGGER, 0.019);
        }
        else if(face == 2)
        {
            if(idx == 0)
            {
                this->move_group_.setJointValueTarget({-0.914, 0.491, -0.644, -2.011, 0.414, 2.378, -1.050+PI/2});
                this->move_group_.move();
                this->move_group_.setJointValueTarget({-0.939, 0.789, -0.582, -1.967, 0.825, 2.543, -1.295+PI/2});
                this->move_group_.move();
            }
            else if(idx == 1)
            {
                this->move_group_.setJointValueTarget({-0.801, 0.719, -0.904, -2.007, 0.753, 2.375, -1.406+PI/2});
                this->move_group_.move();
                this->move_group_.setJointValueTarget({-1.054, 0.799, -0.602, -1.969, 0.818, 2.524, -1.404+PI/2});
                this->move_group_.move();
            }
            else if(idx == 2)
            {
                this->move_group_.setJointValueTarget({-1.002, 0.744, -0.877, -1.951, 0.771, 2.344, -1.517+PI/2});
                this->move_group_.move();
                this->move_group_.setJointValueTarget({-1.143, 0.904, -0.714, -1.866, 0.928, 2.400, -1.564+PI/2});
                this->move_group_.move();
            }
            else
            {

            }
            this->hand_action(false, LOGGER, 0.042);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr translation_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr box_info_subscription_;

private:
    transInfo trans;
    boxInfoArray box_info;

    transInfo trans_saved;
    boxInfoArray box_info_saved;

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
// position: -0.052150, 0.033516, 0.414817, orientation: 0.000000, 0.000000, 0.100394, -0.994948
// position: -0.050712, 0.032148, 0.415049, orientation: 0.000000, 0.000000, -0.089806, 0.995959