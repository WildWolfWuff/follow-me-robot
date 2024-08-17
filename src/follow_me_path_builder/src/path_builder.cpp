
#include <memory>
#include "follow_me_path_builder/path_builder.hpp"

using namespace follow_me;
using namespace std::chrono_literals;

PathBuilder::PathBuilder(const std::string &name)
    : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
    // Setup the parameters
    parent_frame_ = declare_parameter<std::string>("parent_frame", "base_link");
    tag_family_ = declare_parameter<std::string>("tag.family", "tag36h11");
    tag_id_ = declare_parameter<int>("tag.id", 0);
    offset_translation_ = tf2::Vector3(
        declare_parameter<double>("offset.translation.x", 0.0),
        declare_parameter<double>("offset.translation.y", 0.0),
        declare_parameter<double>("offset.translation.z", 0.0));
    offset_rotation_ = tf2::Quaternion(
        declare_parameter<double>("offset.rotation.x", 0.0),
        declare_parameter<double>("offset.rotation.y", 0.0),
        declare_parameter<double>("offset.rotation.z", 0.0),
        declare_parameter<double>("offset.rotation.w", 1.0));
    std::string goal_topic = declare_parameter<std::string>("goal_topic", "goal_pose");
    // Setup the velocity command publisher
    // auto qos = get_node_options().parameter_event_qos();
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 9);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(0.3s, std::bind(&PathBuilder::on_timer, this));
    _tag_frame = "tag" + tag_family_ + ":" + std::to_string(tag_id_);
    RCLCPP_DEBUG(get_logger(), "parent_frame: %s", parent_frame_.c_str());
    RCLCPP_INFO(get_logger(), "path builder node started");
}

void PathBuilder::on_timer()
{
    geometry_msgs::msg::TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(
            parent_frame_, _tag_frame,
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_DEBUG(
            this->get_logger(), "Could not transform %s to %s: %s",
            parent_frame_.c_str(), _tag_frame.c_str(), ex.what());
        return;
    }
    auto translation = tf2::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    auto rotation = tf2::Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    RCLCPP_DEBUG(
        this->get_logger(), "Transform translation: %f %f %f",
        translation.x(),
        translation.y(),
        translation.z());
    RCLCPP_DEBUG(
        this->get_logger(), "Transform rotation: %f %f %f %f",
        rotation.x(),
        rotation.y(),
        rotation.z(),
        rotation.w());
    translation = translation + offset_translation_;
    rotation = rotation * offset_rotation_;
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = parent_frame_;
    goal.header.stamp = this->now();
    goal.pose.position.x = translation.x();
    goal.pose.position.y = translation.y();
    goal.pose.position.z = translation.z();
    goal.pose.orientation.x = rotation.x();
    goal.pose.orientation.y = rotation.y();
    goal.pose.orientation.z = rotation.z();
    goal.pose.orientation.w = rotation.w();
    goal_publisher_->publish(goal);
}