
#include <memory>
#include "follow_me_path_builder/path_builder.hpp"

using namespace follow_me;

PathBuilder::PathBuilder(const std::string &name)
    : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
    // Setup the parameters
    std::string tag_topic = declare_parameter<std::string>("tag_topic", "tf");
    std::string goal_topic = declare_parameter<std::string>("goal_topic", "goal_pose");
    // Setup the velocity command publisher
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Setup the joystick message subscriber
    RCLCPP_INFO(get_logger(), "Mecanum path_builder node started");
}
