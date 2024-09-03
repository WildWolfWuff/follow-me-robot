#ifndef __FOLLOWME_MECANUM_path_builder__FOLLOWME_MECANUM_path_builder_H__
#define __FOLLOWME_MECANUM_path_builder__FOLLOWME_MECANUM_path_builder_H__

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose.h>
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
        void publish_debug(const std::string f_id, const std::string c_id, const tf2::Transform &tf);
                tf2::Transform get_transform(const std::string &from_frame, const std::string &to_frame,bool validate);
        void publish_goal(const tf2::Transform &tag_tf,const std::string &frame_name);
        void on_inital_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    private:
        std::string map_frame_;
        std::string odom_frame_;
        std::string robot_base_frame_;
        std::string camera_frame_;
        std::string camera_lense_frame_;
        std::string tag_family_;
        std::string _tag_frame;
        std::string goal_frame_;
        double goal_timeout_sec_;
        int tag_id_;
        bool debug_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
        rclcpp::TimerBase::SharedPtr _timer{nullptr};
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _initial_pose_sub;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};
        std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster{nullptr};
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        tf2::Transform _buffer;
        tf2::Transform _home_tf;
        tf2::TimePoint last_publish;
        int32_t _prev_sec;
        uint32_t _prev_nsec;
        bool _is_travel_home=false;

        tf2::Quaternion _tag_offset_rotation=tf2::Quaternion();
    };
}
#endif //__FOLLOWME_MECANUM_path_builder__FOLLOWME_MECANUM_path_builder_H__