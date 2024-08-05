#ifndef __FOLLOWME_MECANUM_TELEOP__FOLLOWME_AXIS_CONFIGURATION_H__
#define __FOLLOWME_MECANUM_TELEOP__FOLLOWME_AXIS_CONFIGURATION_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace follow_me
{
    namespace mecanum
    {
        namespace teleop
        {
            class AxisConfiguration{
              public:
                  AxisConfiguration() = default;
                  AxisConfiguration(rclcpp::Node &node,const std::string &name,const std::string &type);
                  double get_axis_value(std::unique_ptr<sensor_msgs::msg::Joy> & msg);
                  uint32_t axis;
                  uint32_t button;
                  double scale;
                  double offset;
                  double deadzone;
            };
        }
    }
}

#endif //__FOLLOWME_MECANUM_TELEOP__FOLLOWME_AXIS_CONFIGURATION_H__