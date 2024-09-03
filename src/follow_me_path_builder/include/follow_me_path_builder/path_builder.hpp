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
        /**
         * @brief This function is called periodically by the timer.
         * It performs the necessary transformations to calculate the tag's position in the global frame.
         * If the transformation fails, it logs an error message and checks if it's time to travel back to the home position.
         * If it's time, it sets the flag to travel home and publishes the goal to the home position.
         * Otherwise, it updates the last publish time and publishes the tag's position as the goal.
         */
        void on_timer();
        /**
         * @brief Publishes a debug transform message.
         *
         * This function publishes a debug transform message using the given frame ID, child frame ID, and transform.
         * The transform message includes the translation and rotation information.
         * If the debug flag is set, the transform message is sent using the tf broadcaster.
         *
         * @param f_id The frame ID of the transform message.
         * @param c_id The child frame ID of the transform message.
         * @param tf The transform to be included in the message.
         */
        void publish_debug(const std::string f_id, const std::string c_id, const tf2::Transform &tf);
        /**
         * @brief Retrieves the transform between two frames.
         *
         * This function looks up the transform between the specified `from_frame` and `to_frame` using the tf2 library.
         * It returns the transform as a `tf2::Transform` object.
         *
         * @param from_frame The name of the source frame.
         * @param to_frame The name of the target frame.
         * @param validate Flag indicating whether to validate the transform.
         * @return The transform between the two frames as a `tf2::Transform` object.
         * @throws tf2::TransformException if the transform is too old and `validate` is set to true.
         */
        tf2::Transform get_transform(const std::string &from_frame, const std::string &to_frame,bool validate);
        /**
         * @brief Publishes a goal pose to the goal_publisher_.
         *
         * This function takes a tf2::Transform object and a frame name as input parameters and publishes a goal pose to the goal_publisher_.
         * The goal pose is constructed using the position and orientation information from the tf2::Transform object.
         *
         * @param tf The tf2::Transform object representing the goal pose.
         * @param frame_name The name of the frame in which the goal pose is defined.
         */
        void publish_goal(const tf2::Transform &tag_tf,const std::string &frame_name);
        /**
         * @brief Callback function for handling the initial pose message.
         *
         * This function is called when a new geometry_msgs::msg::PoseWithCovarianceStamped message is received.
         * It is responsible for processing the message and performing any necessary actions based on the received data.
         *
         * @param msg A shared pointer to the received geometry_msgs::msg::PoseWithCovarianceStamped message.
         */
        void on_inital_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    private:
        std::string map_frame_; // map tf frame name, default: map
        std::string odom_frame_; // odom tf frame name, default: odom
        std::string robot_base_frame_; // robot_base tf frame name, default: base_link
        std::string camera_frame_; // camera_lense frame name, default: None
        std::string camera_lense_frame_; // camera_lense frame name, default: camera_lense
        std::string tag_family_; // tag family name, default: tag36h11
        int tag_id_; // tag id, default: 0
        std::string goal_frame_; // goal frame name, default: map
        bool debug_; // debug flag, default: false
        // goal timeout sec: timeout for refinding tag, after timeout robot will travel to initial pose, default: 60.0
        double goal_timeout_sec_; // timeout for finding  in seconds, default: 0.5
        std::string _tag_frame; // tha tag frame name
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_; // goal publisher
        rclcpp::TimerBase::SharedPtr _timer{nullptr}; // timer for reading tf buffer
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _initial_pose_sub; // initial pose subscriber
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr}; // tf listener
        std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster{nullptr}; // tf broadcaster
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer; // tf buffer
        tf2::Transform _distance_offset; // distance offset for the tag
        tf2::Quaternion _tag_offset_rotation=tf2::Quaternion(); // rotation offset for the tag
        tf2::Transform _home_tf; // home position of the robot
        tf2::TimePoint last_publish; // last publish time
        int32_t _prev_sec; // previous second
        uint32_t _prev_nsec; // previous nanosecond
        bool _is_travel_home=false; // flag to indicate if the robot is traveling home

    };
}
#endif //__FOLLOWME_MECANUM_path_builder__FOLLOWME_MECANUM_path_builder_H__