#include <fstream>
#include <iomanip>

#include "rapidjson/document.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/aruco.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "franka_control/moveit_control.hpp"

#include "franka_msgs/action/grasp.hpp"

using namespace rapidjson;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_moveit_logger");

typedef visualization_msgs::msg::Marker Marker;
typedef visualization_msgs::msg::MarkerArray MarkerArray;


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

void move_predefined(moveit::planning_interface::MoveGroupInterface& move_group, const std::vector<double>& pose)
{
    std::vector<double> home_joint_positions = pose;
    move_group.setJointValueTarget(home_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Move to home pose %s", success ? "" : "FAILED");

    if (success)
        move_group.move();
}

void add_collision(moveit::planning_interface::MoveGroupInterface& move_group,
                   moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.0;
    primitive.dimensions[primitive.BOX_Y] = 2.0;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    collision_object.id = "world_plane1";

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.6;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.08;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

//    box_pose.position.y = -0.7;
//    collision_object.primitives.push_back(primitive);
//    collision_object.primitive_poses.push_back(box_pose);

    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    RCLCPP_INFO(LOGGER, "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
}

void angle_modify(geometry_msgs::msg::PoseStamped& poseStamped)
{
    tf2::Quaternion q_orig, q_rot, q_new;

    // Get the original orientation of 'commanded_pose'
    tf2::convert(poseStamped.pose.orientation , q_orig);

    double r=0, p=0, y=-3.1415926;  // Rotate the previous pose by 180* about X
    q_rot.setRPY(r, p, y);

    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();

// Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_new, poseStamped.pose.orientation);
}

class MoveitControl : public rclcpp::Node
{

public:
    MoveitControl()
            : Node("moveit_control")
    {
        this->declare_parameter("shutdown", 0);
        this->declare_parameter("next", 0);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//        json_read("demo_data.json", this->r_num, this->r_translation, this->r_rotation, this->r_joint_positions);
        json_read("demo_data.json", this->r_num, this->r_pose, this->r_joint_positions);
        json_read("repo_data.json", this->r_num_repo, this->r_pose_repo, this->r_joint_positions_repo);

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/color/image_raw", 3, std::bind(&MoveitControl::image_callback, this, std::placeholders::_1)
                );
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/detected_shape", 3
                );
        markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/markers", 1
                );

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&MoveitControl::timer_callback, this));

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


        detect_flag = false;

        grasp_client_ptr_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
                this, "/panda_gripper/grasp");

    }

    void hand_action(bool open_close)
    {

//        this->timer_->cancel();

        if (!this->grasp_client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = franka_msgs::action::Grasp::Goal();
        if(open_close)
            goal_msg.width = 0.18;
        else
            goal_msg.width = 0.02;
        goal_msg.speed = 0.04;
        goal_msg.force = 100;
        goal_msg.epsilon.inner = 0.01;
        goal_msg.epsilon.outer = 0.01;

        RCLCPP_INFO(LOGGER, "Sending goal");

        auto send_goal_options = rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions();
//        send_goal_options.goal_response_callback =
//                std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
//        send_goal_options.feedback_callback =
//                std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
                std::bind(&MoveitControl::result_callback, this, std::placeholders::_1);
        this->grasp_client_ptr_->async_send_goal(goal_msg, send_goal_options);

        while(!this->finish_flag)
        {
            rclcpp::sleep_for(std::chrono::milliseconds (100));
        }
        this->finish_flag = false;
    }

    bool transform_get(const std::string& toFrameRel, const std::string& fromFrameRel, geometry_msgs::msg::Pose& pose)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = this->tf_buffer_->lookupTransform(
                    toFrameRel, fromFrameRel,
                    tf2::TimePointZero,
                    std::chrono::seconds (1));
        } catch (tf2::TransformException & ex) {
            RCLCPP_INFO(
                    LOGGER, "Could not transform %s to %s: %s",
                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return false;
        }
        pose.position.x = transformStamped.transform.translation.x;
        pose.position.y = transformStamped.transform.translation.y;
        pose.position.z = transformStamped.transform.translation.z;

        pose.orientation.x = transformStamped.transform.rotation.x;
        pose.orientation.y = transformStamped.transform.rotation.y;
        pose.orientation.z = transformStamped.transform.rotation.z;
        pose.orientation.w = transformStamped.transform.rotation.w;

        return true;
    }

    bool transform_get(const std::string& toFrameRel, const std::string& fromFrameRel, geometry_msgs::msg::TransformStamped& pose)
    {
        try {
            pose = this->tf_buffer_->lookupTransform(
                    toFrameRel, fromFrameRel,
                    tf2::TimePointZero,
                    std::chrono::seconds (1));
        } catch (tf2::TransformException & ex) {
            RCLCPP_INFO(
                    LOGGER, "Could not transform %s to %s: %s",
                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return false;
        }
        return true;
    }

    bool detect_flag;
    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Recorded Info
    int r_num = 0;
    std::vector<std::vector<double>> r_translation, r_rotation;
    std::vector<geometry_msgs::msg::PoseStamped> r_pose;
    std::vector<double> r_joint_positions;
    int r_num_repo = 0;
    std::vector<geometry_msgs::msg::PoseStamped> r_pose_repo;
    std::vector<double> r_joint_positions_repo;

    // Marker Info
    std::vector<cv::Point> markers_center;
    std::vector<double> trans;
    double rot = 0.0;
    std::vector<cv::Vec3d> rvecs, tvecs;

private:
    void timer_callback()
    {
        if(!this->detect_flag)
            return;

        this->markers.markers.clear();

        std::vector<cv::Vec<double, 3>> tvecs_;
        if(!this->tvecs.empty())
        {
            tvecs_ = this->tvecs;
        }

        for(size_t i; i<tvecs_.size(); i++)
        {
            this->markers.markers.push_back(MoveitControl::marker_gen("camera_color_optical_frame",
                                                                      "markers",
                                                                      i,
                                                                      {255.0, 255.0, 255.0},
                                                                      std::vector<double>{tvecs_[i].val, tvecs_[i].val+3},
                                                                      {0.0, 0.0, 0.0, 1.0},
                                                                      Marker::CUBE,
                                                                      {0.05, 0.05, 0.01}));
        }

        std::vector<std::vector<double>> box_position, box_rot;
        tf2::Quaternion rot_quat;
        rot_quat.setRPY(0.0, 0.0, this->rot);
        if(!this->r_translation.empty() and !this->r_translation.empty() and !this->markers_center.empty())
        {
            box_position = this->r_translation;
            for(size_t i; i<this->r_translation.size(); i++)
            {
                std::transform (box_position[i].begin(), box_position[i].end(),
                                this->trans.begin(), box_position[i].begin(), std::plus<double>());
                tf2::Quaternion tmp_quat(this->r_rotation[i][0],this->r_rotation[i][1],
                                         this->r_rotation[i][2], this->r_rotation[i][3]);
                tmp_quat *= rot_quat;
                box_rot.push_back({tmp_quat.x(), tmp_quat.y(), tmp_quat.z(), tmp_quat.w()});
            }
        }

        for(size_t i; i<box_position.size(); i++)
        {
            const Marker &marker_tmp = MoveitControl::marker_gen("camera_color_optical_frame",
                                                                 "boxes",
                                                                 i,
                                                                 {0.0, 255.0, 128.0},
                                                                 box_position[i],
                                                                 box_rot[i],
                                                                 Marker::CUBE,
                                                                 {0.05, 0.02, 0.03});
            this->markers.markers.push_back(marker_tmp);
            marker2tf(marker_tmp);
        }
        this->markers_publisher_->publish(this->markers);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
//        if(!this->detect_flag)
//            return;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(LOGGER, "cv_bridge exception: %s", e.what());
            return;
        }
        this->image = cv_ptr->image.clone();

        // Add markers' pose
        int ret = this->detect_markers(cv_ptr);
        if(ret == 0)
        {
//            RCLCPP_INFO(LOGGER, "Cannot detect 4 markers!");
            return;
        }

        cv_ptr->image = this->image;
        image_publisher_->publish(*cv_ptr->toImageMsg());
    }

    int detect_markers(cv_bridge::CvImagePtr& img)
    {
        // Aruco markers detection
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(img->image, this->dictionary, corners, ids,
                                 this->parameters, rejected);

        // So far, it only supports 4 corners
        if(corners.size() != 4)
            return 0;

        // Sort to make it in clockwise order
        std::vector<MarkerInfo> markers_info;
        for(size_t i=0; i<ids.size(); i++)
        {
            MarkerInfo m;
            m.id = ids[i];
            m.corner = corners[i];
            markers_info.push_back(m);
        }
        std::sort(markers_info.begin(), markers_info.end(), marker_sort);

        // Draw the rect of workspace
        // Clear list to avoid repeat recording
        this->markers_center.clear();
        corners.clear();
        ids.clear();
        for(const auto& m : markers_info)
        {
            float x = 0, y = 0;
            size_t l = m.corner.size();
            for(const auto& p : m.corner)
            {
                x += p.x;
                y += p.y;
            }
            corners.push_back(m.corner);
            ids.push_back(m.id);
            this->markers_center.emplace_back(x/l, y/l);
        }
        std::vector<std::vector<cv::Point>> contours = {this->markers_center};
        // Visualization
        cv::aruco::drawDetectedMarkers(this->image, corners, ids);
        cv::drawContours(this->image, contours, 0, cv::Scalar(255, 0, 0), 2);

        const cv::RotatedRect &rect = cv::minAreaRect(this->markers_center);
//        this->rot = rect.angle / 180.0 * CV_PI;
        this->rot = 0;
        // Detect pose
        cv::aruco::estimatePoseSingleMarkers(corners, this->aruco_size,
                                             this->camera_k, this->camera_d,
                                             this->rvecs, this->tvecs);

        this->trans = {0.0, 0.0, 0.0};
        for(auto& t : this->tvecs)
        {
            std::transform (this->trans.begin(), this->trans.end(),
                            t.val, trans.begin(), std::plus<double>());
        }
        std::transform(this->trans.begin(), this->trans.end(), this->trans.begin(),
                       [](double &c){return c /= 4.0; });

        return 1;
    }

    struct MarkerInfo
    {
        std::vector<cv::Point2f> corner;
        int id = 0;
    };

    static bool marker_sort(const MarkerInfo& a, const MarkerInfo& b)
    {
        return(a.id < b.id);
    }

    void json_read(const std::string& file_name, int& num,
                   std::vector<std::vector<double>>& translation,
                   std::vector<std::vector<double>>& rotation,
                   std::vector<double>& joint_positions)
    {
        std::ifstream json_file("/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_control/"+file_name);
        std::stringstream json_buf;
        json_buf << json_file.rdbuf();
        std::string json;
        json_buf >> std::quoted(json);

        Document document;
        document.Parse(json.c_str());

        RCLCPP_INFO(LOGGER, "---------------JSON INFO-------------------");
        // num
        num = document["num"].GetInt();
        RCLCPP_INFO(LOGGER, "num = %d", num);
        // translation
        RCLCPP_INFO(LOGGER, "translation:");
        const Value& translation_ = document["translation"];
        assert(translation_.IsArray());
        for (auto& i : translation_.GetArray())
        {
            std::vector<double> tmp;
            for(auto& j : i.GetArray())
            {
                RCLCPP_INFO(LOGGER, "%f, ", j.GetDouble());
                tmp.push_back(j.GetDouble());
            }
            translation.push_back(tmp);
        }
        // rotation
        RCLCPP_INFO(LOGGER, "rotation:");
        const Value& rotation_ = document["rotation"];
        assert(rotation_.IsArray());
        for (auto& i : rotation_.GetArray())
        {
            std::vector<double> tmp;
            for(auto& j : i.GetArray())
            {
                RCLCPP_INFO(LOGGER, "%f, ", j.GetDouble());
                tmp.push_back(j.GetDouble());
            }
            rotation.push_back(tmp);
        }
        // joint_positions
        RCLCPP_INFO(LOGGER, "joint_positions:");
        const Value& joint_positions_ = document["joint_positions"];
        assert(joint_positions_.IsArray());
        for (auto& i : joint_positions_.GetArray())
        {
            joint_positions.push_back(i.GetDouble());
        }
        RCLCPP_INFO(LOGGER, "---------------JSON INFO-------------------");
    }

    void json_read(const std::string& file_name, int& num,
                   std::vector<tf2::Vector3>& translation,
                   std::vector<tf2::Quaternion>& rotation,
                   std::vector<double>& joint_positions)
    {
        /* The overloaded function adapted to ROS types. */
        std::ifstream json_file("/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_control/"+file_name);
        std::stringstream json_buf;
        json_buf << json_file.rdbuf();
        std::string json;
        json_buf >> std::quoted(json);

        Document document;
        document.Parse(json.c_str());

        /* Save the parsed info into class public vars and print them out. */
        RCLCPP_INFO(LOGGER, "---------------JSON INFO-------------------");
        // num
        num = document["num"].GetInt();
        RCLCPP_INFO(LOGGER, "num = %d", num);
        // translation
        RCLCPP_INFO(LOGGER, "translation:");
        const Value& translation_ = document["translation"];
        assert(translation_.IsArray());
        for (auto& i : translation_.GetArray())
        {
            std::vector<double> tmp;
            for(auto& j : i.GetArray())
            {
                RCLCPP_INFO(LOGGER, "%f, ", j.GetDouble());
                tmp.push_back(j.GetDouble());
            }
            assert(tmp.size() == 3);
            translation.emplace_back(tmp[0], tmp[1], tmp[2]);
        }
        // rotation
        RCLCPP_INFO(LOGGER, "rotation:");
        const Value& rotation_ = document["rotation"];
        assert(rotation_.IsArray());
        for (auto& i : rotation_.GetArray())
        {
            std::vector<double> tmp;
            for(auto& j : i.GetArray())
            {
                RCLCPP_INFO(LOGGER, "%f, ", j.GetDouble());
                tmp.push_back(j.GetDouble());
            }
            assert(tmp.size() == 4);
            rotation.emplace_back(tmp[0], tmp[1], tmp[2], tmp[3]);
        }
        // joint_positions
        RCLCPP_INFO(LOGGER, "joint_positions:");
        const Value& joint_positions_ = document["joint_positions"];
        assert(joint_positions_.IsArray());
        for (auto& i : joint_positions_.GetArray())
        {
            RCLCPP_INFO(LOGGER, "%f, ", i.GetDouble());
            joint_positions.push_back(i.GetDouble());
        }
        RCLCPP_INFO(LOGGER, "---------------JSON INFO-------------------");
    }

    void json_read(const std::string& file_name, int& num,
                   std::vector<geometry_msgs::msg::PoseStamped>& pose,
                   std::vector<double>& joint_positions)
    {
        /* The overloaded function adapted to ROS types. */
        std::ifstream json_file("/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_control/"+file_name);
        std::stringstream json_buf;
        json_buf << json_file.rdbuf();
        std::string json;
        json_buf >> std::quoted(json);

        Document document;
        document.Parse(json.c_str());

        /* Save the parsed info into class public vars and print them out. */
        RCLCPP_INFO(LOGGER, "---------------JSON INFO-------------------");
        // num
        num = document["num"].GetInt();
        RCLCPP_INFO(LOGGER, "num = %d", num);
        // translation
        RCLCPP_INFO(LOGGER, "translation:");
        const Value& translation_ = document["translation"];
        assert(translation_.IsArray());
        std::vector<geometry_msgs::msg::Point> translation;
        for (auto& i : translation_.GetArray())
        {
            std::vector<double> tmp;
            for(auto& j : i.GetArray())
            {
                RCLCPP_INFO(LOGGER, "%f, ", j.GetDouble());
                tmp.push_back(j.GetDouble());
            }
            assert(tmp.size() == 3);
            geometry_msgs::msg::Point p;
            p.x = tmp[0]; p.y = tmp[1]; p.z = tmp[2];
            translation.push_back(p);
        }
        // rotation
        RCLCPP_INFO(LOGGER, "rotation:");
        const Value& rotation_ = document["rotation"];
        assert(rotation_.IsArray());
        std::vector<geometry_msgs::msg::Quaternion> rotation;
        for (auto& i : rotation_.GetArray())
        {
            std::vector<double> tmp;
            for(auto& j : i.GetArray())
            {
                RCLCPP_INFO(LOGGER, "%f, ", j.GetDouble());
                tmp.push_back(j.GetDouble());
            }
            assert(tmp.size() == 4);
            geometry_msgs::msg::Quaternion q;
            q.x = tmp[0]; q.y = tmp[1]; q.z = tmp[2]; q.w = tmp[3];
            rotation.push_back(q);
        }
        for (size_t i; i < translation.size(); i++)
        {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position = translation[i]; p.pose.orientation = rotation[i];
            pose.push_back(p);
        }
        // joint_positions
        RCLCPP_INFO(LOGGER, "joint_positions:");
        const Value& joint_positions_ = document["joint_positions"];
        assert(joint_positions_.IsArray());
        for (auto& i : joint_positions_.GetArray())
        {
            RCLCPP_INFO(LOGGER, "%f, ", i.GetDouble());
            joint_positions.push_back(i.GetDouble());
        }
        RCLCPP_INFO(LOGGER, "---------------JSON INFO-------------------");
    }

    using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>;
    void result_callback(const GoalHandleGrasp::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(LOGGER, "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(LOGGER, "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(LOGGER, "Unknown result code");
                break;
        }
        finish_flag = true;
    }

    Marker marker_gen(const std::string& frame_id, const std::string& class_name, int idx,
                      const std::vector<float>& color,
                      const std::vector<double>& pos, const std::vector<double>& ori,
                      const int32_t s_type,
                      const std::vector<double>& scale={0.1, 0.1, 0.1})
    {
        // Generate the needed markers with parameters
        auto marker = visualization_msgs::msg::Marker();

        marker.header.stamp = this->now() - rclcpp::Duration::from_seconds(0.05);
        marker.header.frame_id = frame_id;
        marker.ns = class_name;
        marker.id = idx;
        marker.type = s_type;
        marker.action = Marker::ADD;

        marker.pose.position.x = pos[0];
        marker.pose.position.y = pos[1];
        marker.pose.position.z = pos[2];

        marker.pose.orientation.x = ori[0];
        marker.pose.orientation.y = ori[1];
        marker.pose.orientation.z = ori[2];
        marker.pose.orientation.w = ori[3];

        marker.scale.x = scale[0];
        marker.scale.y = scale[1];
        marker.scale.z = scale[2];

        marker.color.a = 0.5;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];

        return marker;
    }

    void marker2tf(const Marker& msg)
    {
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header = msg.header;
        t.child_frame_id = msg.ns + std::to_string(msg.id);

        t.transform.translation.x = msg.pose.position.x;
        t.transform.translation.y = msg.pose.position.y;
        t.transform.translation.z = msg.pose.position.z;

        t.transform.rotation.x = msg.pose.orientation.x;
        t.transform.rotation.y = msg.pose.orientation.y;
        t.transform.rotation.z = msg.pose.orientation.z;
        t.transform.rotation.w = msg.pose.orientation.w;

        // Send the transformation
        this->tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::Mat image;
    MarkerArray markers;

    float aruco_size = 0.018;
    cv::Mat camera_k = (cv::Mat_<double>(3,3) <<
            906.3109741210938, 0.0, 636.7540283203125,
            0.0, 905.7764892578125, 352.17510986328125,
            0.0, 0.0, 1.0);
    cv::Mat camera_d = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr grasp_client_ptr_;
    bool finish_flag = false;
};

int main(int argc, char **argv)
{
    // Node init
    rclcpp::init(argc, argv);
    auto move_group_node = std::make_shared<MoveitControl>();

    // Execute node thread
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    rclcpp::sleep_for(std::chrono::seconds(2));

    static const std::string PLANNING_GROUP = "panda_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    move_group.setPlannerId("RRTkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.25);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setPlanningTime(2);
    move_group.allowReplanning(true);
    add_collision(move_group, planning_scene_interface);
//    move_group_node->detect_flag = true;

    move_predefined(move_group, "home");
//    move_predefined(move_group, "waiting");
    for(int count=0; count<move_group_node->r_num; count++)
    {
//        move_predefined(move_group, move_group_node->r_joint_positions_repo);
        if(count < 3)
        {
            if(count == 0)
                move_predefined(move_group, "box1");
            else if(count == 1)
                move_predefined(move_group, "box2");
            else if(count == 2)
                move_predefined(move_group, "box3");
            move_group_node->hand_action(false);
        }

        rclcpp::sleep_for(std::chrono::seconds(1));

        move_predefined(move_group, "home");

        rclcpp::sleep_for(std::chrono::seconds(1));

        move_predefined(move_group, "detect");

        rclcpp::sleep_for(std::chrono::seconds(1));

        geometry_msgs::msg::TransformStamped abs_trans_cam;
        geometry_msgs::msg::PoseStamped target_in, target_tmp;
        geometry_msgs::msg::Pose target;

//        for(int i = 0; i<10; i++)
//        {
//            if(move_group_node->transform_get("panda_link0", "camera_color_optical_frame",abs_trans_cam))
//            {
//                // Transform to get object pose
//                target_in = move_group_node->r_pose_repo[count];
//                auto tvec_tmp = move_group_node->trans;
//                target_in.pose.position.x += tvec_tmp[0]; target_tmp.pose.position.y += tvec_tmp[1]; target_tmp.pose.position.z += tvec_tmp[2];
//
//                // Move to grasp
//                tf2::doTransform(target_in, target_tmp, abs_trans_cam);
//
//                target = target_tmp.pose;
//                target.position.z = 0.01;
//                move_group.setPoseTarget(target, "panda_hand_tcp");
//                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//                bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//                if(success)
//                    move_group.move();
//                else
//                {
//                    ;
////                    angle_modify(target_in);
////
////                    tf2::doTransform(target_in, target_tmp, abs_trans_cam);
////
////                    target = target_tmp.pose;
////                    target.position.z = 0.03;
////                    move_group.setPoseTarget(target, "panda_hand_tcp");
////
////                    move_group.move();
//                }
//
//                // Grasp
//                move_group_node->hand_action(false);
//                move_predefined(move_group, move_group_node->r_joint_positions_repo);
//                rclcpp::sleep_for(std::chrono::seconds(1));
//                break;
//            }
//        }
//
//        move_predefined(move_group, move_group_node->r_joint_positions);
////        move_group_node->detect_flag = true;
//        rclcpp::sleep_for(std::chrono::seconds(1));

        for(int i = 0; i<10; i++)
        {
            if(move_group_node->transform_get("panda_link0", "camera_color_optical_frame",abs_trans_cam))
            {
                // Transform to get object pose
                target_tmp = move_group_node->r_pose[count];
                auto tvec_tmp = move_group_node->trans;
                target_tmp.pose.position.x += tvec_tmp[0]; target_tmp.pose.position.y += tvec_tmp[1]; target_tmp.pose.position.z += tvec_tmp[2];
                tf2::doTransform(target_tmp, target_tmp, abs_trans_cam);

                // Move to grasp
                target = target_tmp.pose;
                target.position.z = 0.02;
//                move_group_node->detect_flag = false;

//                for(size_t j = 0; j<5; j++)
//                {
//                    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//                    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//                    if(success)
//                    {
//                        move_group.move();
//                        break;
//                    }
//                }

                move_group.setPoseTarget(target, "panda_hand_tcp");
                move_group.move();

                move_group_node->hand_action(true);

                target.position.z = 0.2;
                move_group.setPoseTarget(target, "panda_hand_tcp");
                move_group.move();
                break;
            }
        }


        move_predefined(move_group, "home");
//        move_predefined(move_group, move_group_node->r_joint_positions);
//        move_group_node->detect_flag = true;

//        while(!move_group_node->get_parameter("next").get_parameter_value().get<int>());
//        move_group_node->set_parameter(rclcpp::Parameter("next", 0));
    }

    while(!move_group_node->get_parameter("next").get_parameter_value().get<int>());
    move_group_node->set_parameter(rclcpp::Parameter("next", 0));
    move_predefined(move_group, "waiting");
//    while(!move_group_node->get_parameter("shutdown").get_parameter_value().get<int>());

    rclcpp::shutdown();
    return 0;
}