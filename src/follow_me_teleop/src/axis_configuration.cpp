#include "follow_me_teleop/axis_configuration.hpp"

using namespace follow_me::mecanum::teleop;

AxisConfiguration::AxisConfiguration(rclcpp::Node &node,const std::string &name,const std::string &type)
{
    RCLCPP_INFO(node.get_logger(), "Loading configuration for %s", name.c_str());
    const std::string axis_param = type+ "." + name + ".axis";
    const std::string button_param = type+ "." + name + ".button";
    const std::string scale_param = type+ "." + name + ".scale";
    const std::string offset_param = type+ "." + name + ".offset";
    const std::string deadzone_param = type+ "." + name + ".deadzone";
    axis = node.declare_parameter<int>(axis_param.c_str(), 0);
    RCLCPP_DEBUG(node.get_logger(), "%s: %d",axis_param.c_str(), axis);
    button = node.declare_parameter<int>(button_param.c_str(), 0);
    RCLCPP_DEBUG(node.get_logger(), "%s: %d",button_param.c_str(), button);
    scale= node.declare_parameter<double>(scale_param.c_str(), 1.0);
    RCLCPP_DEBUG(node.get_logger(), "%s: %f",scale_param.c_str(), scale);
    offset = node.declare_parameter<double>(offset_param.c_str(), 0.0);
    RCLCPP_DEBUG(node.get_logger(), "%s: %f",offset_param.c_str(), offset);
    deadzone=node.declare_parameter<double>(deadzone_param.c_str(), 0.0);
    RCLCPP_DEBUG(node.get_logger(), "%s: %f",deadzone_param.c_str(), deadzone);
}
double AxisConfiguration::get_axis_value(std::unique_ptr<sensor_msgs::msg::Joy> & msg)
{
  if (msg->axes.size() >= (size_t)axis) {
        double value = ((double)msg->axes[axis] - offset) * scale;
        if (value > deadzone) {
            value -= deadzone;
        }
        else if (value < -deadzone) {
            value += deadzone;
        }
        else {
            value = 0.0;
        }
        return value;
    }
    return 0.0;
}