#ifndef __FOLLOWME_MECANUM_TELEOP__FOLLOWME_MECANUM_JOYSTICK_H__
#define __FOLLOWME_MECANUM_TELEOP__FOLLOWME_MECANUM_JOYSTICK_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>

namespace followme
{
    namespace mecanum
    {
        namespace teleop
        {
            class MecanumJoystick
                : public rclcpp::Node
            {
            public:
                MecanumJoystick(const std::string &name);
                ~MecanumJoystick();

            private:
                void update();

            private:
                rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
                rclcpp::TimerBase::SharedPtr update_timer_;
                std::vector<int> buttons_;
                std::vector<double> axes_;
                int device_handle_;
                double deadzone_;
                double scale_;
                double unscaled_deadzone_;
            };
        }
    }
}
#endif //__FOLLOWME_MECANUM_TELEOP__FOLLOWME_MECANUM_JOYSTICK_H__