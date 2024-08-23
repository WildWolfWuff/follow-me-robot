
#include <memory>
#include "follow_me_path_builder/path_builder.hpp"

using namespace follow_me;
using namespace std::chrono_literals;

PathBuilder::PathBuilder(const std::string &name)
    : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
    // Setup the parameters
    robot_base_frame_ = declare_parameter<std::string>("robot_base", "base_link");
    odom_frame_ = declare_parameter<std::string>("doom", "odom");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "base_link");
    tag_family_ = declare_parameter<std::string>("tag.family", "tag36h11");
    tag_id_ = declare_parameter<int>("tag.id", 0);
    auto offset_translation = tf2::Vector3(
        declare_parameter<double>("offset.translation.x", 0.0),
        declare_parameter<double>("offset.translation.y", 0.0),
        declare_parameter<double>("offset.translation.z", 0.0));
    auto offset_rotation = tf2::Quaternion(
        declare_parameter<double>("offset.rotation.x", 0.0),
        declare_parameter<double>("offset.rotation.y", 0.0),
        declare_parameter<double>("offset.rotation.z", 0.0),
        declare_parameter<double>("offset.rotation.w", 1.0));
    offset_ = tf2::Transform(offset_rotation, offset_translation);
    std::string goal_topic = declare_parameter<std::string>("goal.topic", "goal_pose");
    goal_frame_ = declare_parameter<std::string>("goal.frame_id", "map");
    // Setup the velocity command publisher
    // auto qos = get_node_options().parameter_event_qos();
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 9);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = create_wall_timer(1s, std::bind(&PathBuilder::on_timer, this));
    _tag_frame = "tag" + tag_family_ + ":" + std::to_string(tag_id_);

    RCLCPP_DEBUG(get_logger(), "parent_frame: %s", parent_frame_.c_str());
    RCLCPP_INFO(get_logger(), "path builder node started");
}
void PathBuilder::publish_debug(const std::string f_id, const std::string c_id, const tf2::Vector3 &translation, const tf2::Quaternion &rotation)
{
    geometry_msgs::msg::Vector3 v;
    v.x = translation.x();
    v.y = translation.y();
    v.z = translation.z();
    geometry_msgs::msg::Quaternion q;
    q.x = rotation.x();
    q.y = rotation.y();
    q.z = rotation.z();
    q.w = rotation.w();
    geometry_msgs::msg::TransformStamped test;
    test.header.frame_id = f_id;
    test.child_frame_id = c_id;

    RCLCPP_INFO(this->get_logger(), "Translation %s -> %s : x=%f, y=%f, z=%f",
                f_id.c_str(),
                c_id.c_str(),
                v.x,
                v.y,
                v.z);
    RCLCPP_INFO(this->get_logger(), "Rotation %s -> %s : x=%f, y=%f, z=%f, w=%f",
                f_id.c_str(),
                c_id.c_str(),
                q.x,
                q.y,
                q.z,
                q.w);

    test.transform.set__translation(v);
    test.transform.set__rotation(q);
    tf_broadcaster_->sendTransform(test);
}
void PathBuilder::on_timer()
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::TransformStamped robot_tf;

    try
    {
        t = tf_buffer_->lookupTransform(
            parent_frame_, _tag_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s",
                     parent_frame_.c_str(), _tag_frame.c_str(), ex.what());

        return;
    }
    try
    {
        robot_tf = tf_buffer_->lookupTransform(
            odom_frame_, robot_base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                    odom_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
        return;
    }
    
    auto bot_translation = tf2::Vector3(robot_tf.transform.translation.x, robot_tf.transform.translation.y, robot_tf.transform.translation.z);
    auto bot_rotation = tf2::Quaternion(robot_tf.transform.rotation.x, robot_tf.transform.rotation.y, robot_tf.transform.rotation.z, robot_tf.transform.rotation.w);
    auto tag_translation = tf2::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z) + bot_translation;
    auto r = tf2::Quaternion(t.transform.rotation.x, 0, t.transform.rotation.y, t.transform.rotation.w);
    publish_debug(goal_frame_, "bot", bot_translation, r);
    publish_debug(goal_frame_, "test", tag_translation, r);
    auto tag_length = tag_translation.length();

    auto tag_angular = atan2(tag_translation.y(), tag_translation.x());

    auto translation = tf2::Vector3(tag_length * -sin(tag_angular), tag_length * cos(tag_angular), 0);
    // auto p = bot_translation + translation;
    publish_debug(goal_frame_, "goal", translation, r);
}