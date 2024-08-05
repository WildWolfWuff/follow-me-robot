#ifndef __FOLLOWME_MECANUM_TELEOP__FOLLOWME_MECANUM_TELEOP_H__
#define __FOLLOWME_MECANUM_TELEOP__FOLLOWME_MECANUM_TELEOP_H__


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <follow_me_teleop/axis_configuration.hpp>

namespace follow_me
{
    namespace mecanum
    {
        namespace teleop
        {
            class MecanumTeleop
                : public rclcpp::Node
            {
            // public:
                // struct AxisConfig
                // {
                //     uint32_t axis;
                //     double scale;
                //     double offset;
                //     double deadzone;
                // };

            public:
                MecanumTeleop(const std::string &name);

            private:
                void on_joy_message(std::unique_ptr<sensor_msgs::msg::Joy> msg);

            private:
                rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
                AxisConfiguration move_forward_config_;
                AxisConfiguration move_reverse_config_;
                AxisConfiguration move_left_config_;
                AxisConfiguration move_right_config_;
                AxisConfiguration turn_left_config_;
                AxisConfiguration turn_right_config_;
            };
        }
    }
}
#endif //__FOLLOWME_MECANUM_TELEOP__FOLLOWME_MECANUM_TELEOP_H__