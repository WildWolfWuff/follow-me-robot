
#include <memory>
#include "follow_me_teleop/mecanum_teleop.hpp"

using namespace follow_me::mecanum::teleop;

MecanumTeleop::AxisConfig MecanumTeleop::build_config(const std::string &name, const std::string &type)
{
    const std::string axis_param = type+ "." + name + ".axis";
    const std::string scale_param = type+ "." + name + ".scale";
    const std::string offset_param = type+ "." + name + ".offset";
    const std::string deadzone_param = type+ "." + name + ".deadzone";
    AxisConfig config;
    config.axis = declare_parameter<int>(axis_param, 0);
    RCLCPP_DEBUG(get_logger(), "%s: %d",axis_param.c_str(), config.axis);
    config.deadzone = declare_parameter<double>(deadzone_param, 0.0);
    RCLCPP_DEBUG(get_logger(), "%s: %f",deadzone_param.c_str(), config.deadzone);
    config.scale = declare_parameter<double>(scale_param, 1.0);
    RCLCPP_DEBUG(get_logger(), "%s: %f",scale_param.c_str(), config.scale);
    config.offset = declare_parameter<double>(offset_param, 0.0);
    RCLCPP_DEBUG(get_logger(), "%s: %f",offset_param.c_str(), config.offset);
    return config;
}

MecanumTeleop::MecanumTeleop(const std::string & name)
    : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // Setup the parameters
    std::string joy_topic= declare_parameter<std::string>("joy_topic", "joy");
    std::string twist_topic = declare_parameter<std::string>("twist_topic", "cmd_vel");
    
    move_forward_config_= build_config(std::string("forward"),std::string("move"));
    move_reverse_config_ = build_config(std::string("reverse"),std::string("move"));
    move_left_config_ = build_config(std::string("left"),std::string("move"));
    move_right_config_ = build_config(std::string("right"),std::string("move"));
    turn_left_config_ = build_config(std::string("left"),std::string("turn"));
    turn_right_config_ = build_config(std::string("right"),std::string("turn"));
    
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
    twist_message->linear.x = get_axis_value(msg, move_forward_config_) + get_axis_value(msg, move_reverse_config_);
    twist_message->linear.y = get_axis_value(msg, move_left_config_) + get_axis_value(msg, move_right_config_);
    twist_message->linear.z = 0.0;
    twist_message->angular.x = 0.0;
    twist_message->angular.y = 0.0;
    twist_message->angular.z = get_axis_value(msg, turn_left_config_) + get_axis_value(msg, turn_right_config_);
    twist_publisher_->publish(std::move(twist_message));
}

double MecanumTeleop::get_axis_value(std::unique_ptr<sensor_msgs::msg::Joy> & msg, AxisConfig& config)
{
    if (msg->axes.size() >= (size_t)config.axis) {
        double value = ((double)msg->axes[config.axis] - config.offset) * config.scale;
        if (value > config.deadzone) {
            value -= config.deadzone;
        }
        else if (value < -config.deadzone) {
            value += config.deadzone;
        }
        else {
            value = 0.0;
        }
        return value;
    }
    return 0.0;
}