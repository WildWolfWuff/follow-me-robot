
#include <memory>
#include "follow_me_teleop/mecanum_teleop.hpp"

using namespace follow_me::mecanum::teleop;

MecanumTeleop::MecanumTeleop(const std::string & name)
    : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // Setup the parameters
    std::string twist_topic = declare_parameter<std::string>("joy_topic", "joy");
    std::string joy_topic = declare_parameter<std::string>("twist_topic", "cmd_vel");
    
    move_forward_config_=AxisConfiguration( *this,std::string("forward"),std::string("move"));
    move_reverse_config_=AxisConfiguration( *this,std::string("reverse"),std::string("move"));
    move_left_config_=AxisConfiguration( *this,std::string("left"),std::string("move"));
    move_right_config_=AxisConfiguration( *this,std::string("right"),std::string("move"));
    turn_left_config_=AxisConfiguration( *this,std::string("left"),std::string("turn"));
    turn_right_config_=AxisConfiguration( *this,std::string("right"),std::string("turn"));
    
    // Setup the velocity command publisher
    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>(twist_topic, 10);
    RCLCPP_DEBUG(get_logger(), "Create twist publisher for topic %s", twist_topic.c_str());

    // Setup the joystick message subscriber
    joy_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(joy_topic, 10, std::bind(&MecanumTeleop::on_joy_message, this, std::placeholders::_1));
    RCLCPP_DEBUG(get_logger(), "Create joy subscriber for topic %s", joy_topic.c_str());
    RCLCPP_INFO(get_logger(), "Mecanum teleop node started");
}

void MecanumTeleop::on_joy_message(std::unique_ptr<sensor_msgs::msg::Joy> msg)
{
    geometry_msgs::msg::Twist::UniquePtr twist_message(new geometry_msgs::msg::Twist());
    twist_message->linear.x =move_forward_config_.get_axis_value(msg) + move_reverse_config_.get_axis_value(msg);
    twist_message->linear.y =  move_left_config_.get_axis_value(msg) +  move_right_config_.get_axis_value(msg);
    twist_message->linear.z = 0.0;
    twist_message->angular.x = 0.0;
    twist_message->angular.y = 0.0;
    twist_message->angular.z =  turn_left_config_.get_axis_value(msg) + turn_right_config_.get_axis_value(msg);
    twist_publisher_->publish(std::move(twist_message));
}
