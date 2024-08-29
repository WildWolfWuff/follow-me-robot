
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
    camera_lense_frame_ = declare_parameter<std::string>("camera_lense", "base_link");
    camera_frame_ = declare_parameter<std::string>("camera", "base_link");

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

    RCLCPP_DEBUG(get_logger(), "parent_frame: %s", camera_lense_frame_.c_str());
    RCLCPP_INFO(get_logger(), "path builder node started");
}
tf2::Transform PathBuilder::get_transform(const std::string &from_frame, const std::string &to_frame){
    // auto tp= tf2::get_now();
    auto t = tf_buffer_->lookupTransform(
            from_frame, to_frame, tf2::timeFromSec(0.1), tf2::durationFromSec(0.01));
    // if the transform is older than 100ms throw a tf2::TransformException
    RCLCPP_INFO(this->get_logger(),"TIME %s -> %s : %f",from_frame.c_str(),to_frame.c_str(), t.header.stamp.sec);
    return tf2::Transform(tf2::Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
                               tf2::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
}
void PathBuilder::publish_debug(const std::string f_id, const std::string c_id, const tf2::Transform &tf)
{
    geometry_msgs::msg::Vector3 v;
    v.x = tf.getOrigin().x();
    v.y = tf.getOrigin().y();
    v.z = tf.getOrigin().z();
    geometry_msgs::msg::Quaternion q;
    q.x = tf.getRotation().x();
    q.y = tf.getRotation().y();
    q.z = tf.getRotation().z();
    q.w = tf.getRotation().w();
    geometry_msgs::msg::TransformStamped test;
    test.header.frame_id = f_id;
    test.child_frame_id = c_id;
    test.transform.translation = v;
    test.transform.rotation = q;
    tf_broadcaster_->sendTransform(test);
}
void PathBuilder::on_timer()
{
    tf2::Transform tag_tf;
    try
    {
        auto tag_to_cam_lense = get_transform(camera_lense_frame_, _tag_frame);
        auto cam_lense_to_cam = get_transform(camera_frame_, camera_lense_frame_);
        auto cam_to_robot = get_transform(robot_base_frame_, camera_frame_);
        auto robot_tf = get_transform(odom_frame_, robot_base_frame_);
        tag_tf = robot_tf * cam_to_robot * cam_lense_to_cam * tag_to_cam_lense;
        // tag_tf.getOrigin().setZ(0);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s",
                     camera_lense_frame_.c_str(), _tag_frame.c_str(), ex.what());
        return;
    }

    publish_debug(goal_frame_, "TAG_2D", tag_tf);
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = goal_frame_;
    goal.header.stamp = this-> now();
    goal.pose.position.x = tag_tf.getOrigin().x();
    goal.pose.position.y = tag_tf.getOrigin().y();
    goal.pose.position.z = 0;
    goal.pose.orientation.x = tag_tf.getRotation().x();
    goal.pose.orientation.y=tag_tf.getRotation().y();
    goal.pose.orientation.z=tag_tf.getRotation().z();
    goal.pose.orientation.w = tag_tf.getRotation().w();
    goal_publisher_->publish(goal);
}