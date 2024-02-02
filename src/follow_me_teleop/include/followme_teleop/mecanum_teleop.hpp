#ifndef __FOLLOWME_MECANUM_TELEOP__FOLLOWME_MECANUM_TELEOP_H__
#define __FOLLOWME_MECANUM_TELEOP__FOLLOWME_MECANUM_TELEOP_H__


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace followme
{
    namespace mecanum
    {
        namespace teleop
        {
            class MecanumTeleop
                : public rclcpp::Node
            {
            public:
                struct AxisConfig
                {
                    uint32_t axis;
                    double scale;
                    double offset;
                    double deadzone;
                };

            public:
                MecanumTeleop(const std::string &name);

            private:
                void on_joy_message(std::unique_ptr<sensor_msgs::msg::Joy> msg);
                double get_axis_value(std::unique_ptr<sensor_msgs::msg::Joy> &msg, AxisConfig &config);

            private:
                rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
                AxisConfig move_forward_config_;
                AxisConfig move_reverse_config_;
                AxisConfig move_left_config_;
                AxisConfig move_right_config_;
                AxisConfig turn_left_config_;
                AxisConfig turn_right_config_;
            };
        }
    }
}
#endif //__FOLLOWME_MECANUM_TELEOP__FOLLOWME_MECANUM_TELEOP_H__