#ifndef __FOLLOWME_MECANUM_path_builder__FOLLOWME_MECANUM_path_builder_H__
#define __FOLLOWME_MECANUM_path_builder__FOLLOWME_MECANUM_path_builder_H__

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
namespace follow_me
{
    class PathBuilder
        : public rclcpp::Node
    {

    public:
        PathBuilder(const std::string &name);

    private:
        void on_timer();

    private:
        std::string parent_frame_;
        std::string tag_family_;
        std::string _tag_frame;
        int tag_id_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        tf2::Vector3 offset_translation_;
        tf2::Quaternion offset_rotation_;
    };
}
#endif //__FOLLOWME_MECANUM_path_builder__FOLLOWME_MECANUM_path_builder_H__