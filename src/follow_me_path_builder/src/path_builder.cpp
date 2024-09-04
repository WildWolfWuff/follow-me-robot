
#include <memory>
#include "follow_me_path_builder/path_builder.hpp"

using namespace follow_me;
using namespace std::chrono_literals;

PathBuilder::PathBuilder(const std::string &name)
    : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    // https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
    // Setup the parameters
    // define parameters for the node

    map_frame_ = declare_parameter<std::string>("frame.map", "map");
    robot_base_frame_ = declare_parameter<std::string>("frame.robot_base", "base_link");
    odom_frame_ = declare_parameter<std::string>("frame.odom", "odom");
    camera_frame_ = declare_parameter<std::string>("frame.camera");
    camera_lense_frame_ = declare_parameter<std::string>("frame.camera_lense");

    tag_family_ = declare_parameter<std::string>("tag.family", "tag36h11");
    tag_id_ = declare_parameter<int>("tag.id", 0);
    debug_ = declare_parameter<bool>("debug", false);
    distance_=declare_parameter<double>("distance", 0.0);
    goal_topic_ = declare_parameter<std::string>("goal.topic", "goal_pose");
    goal_timeout_sec_ = declare_parameter<double>("goal.timeout_sec", 60.0);
    // create publisher and subscriber
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 9);
    _initial_pose_sub= create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&PathBuilder::on_inital_pose, this, std::placeholders::_1));
    
    // create the tf2 buffer, listener, and broadcaster for the transform system
    _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // create a timer to listne to the tf2 buffer
    _timer = create_wall_timer(50ms, std::bind(&PathBuilder::on_timer, this));

    // define the offset for the tag's position
    _distance_offset=tf2::Transform(tf2::Quaternion(0,0,0,1),tf2::Vector3(-distance_,0,0));
    _tag_offset_rotation.setRPY(0,M_PI_2,0);
    
    // create the tag frame name
    _tag_frame = "tag" + tag_family_ + ":" + std::to_string(tag_id_);

    RCLCPP_INFO(get_logger(), "path builder node started for tag: %s", _tag_frame.c_str());
}


void PathBuilder::on_inital_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    RCLCPP_DEBUG(this->get_logger(), "Initial pose received");
    // set the home position of the robot by using the initial pose message
    _home_tf=tf2::Transform(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w),
                               tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
}

tf2::Transform PathBuilder::get_transform(const std::string &from_frame, const std::string &to_frame,bool validate){
    // extract the requested transform from the buffer
    auto t = _tf_buffer->lookupTransform(
            from_frame, to_frame, tf2::TimePointZero, tf2::durationFromSec(0.01));
    
    // throw an exception if no new transform is available in validate mode
    if(validate){
        auto sec=t.header.stamp.sec;
        auto nsec=t.header.stamp.nanosec;
        bool error= sec == _prev_sec && nsec == _prev_nsec;
        _prev_sec = sec;
        _prev_nsec = nsec;
        if(error){
            throw tf2::TransformException("Transform is too old");
        }
    }
    // RCLCPP_INFO(this->get_logger(),"TIME %s -> %s = %d:%d",from_frame.c_str(),to_frame.c_str(), sec,nsec);
    return tf2::Transform(tf2::Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
                               tf2::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
}

void PathBuilder::on_timer()
{
    // calculate the tag's position in the global frame
    tf2::Transform global_goal_tf;
    // get the current time for the travel home timeout
    auto time_now=tf2::get_now();
    try
    {
        // gets the transform from the tag to the camer lense
        auto tag_to_cam_lense = get_transform(camera_lense_frame_, _tag_frame,true);
        // gets the transform from the camera lense to the camera
        auto cam_lense_to_cam = get_transform(camera_frame_, camera_lense_frame_,false);
        // gets the transform from the camera to the robot base
        auto cam_to_robot = get_transform(robot_base_frame_, camera_frame_,false);
        // gets the transform from the robot base to the odom
        auto robot_tf = get_transform(odom_frame_, robot_base_frame_,false);
        // gets the transform from the odom to the map
        auto map_tf = get_transform(map_frame_, odom_frame_,false);
        // apply rotation offset to the tag tf to get the forward direction
        auto r= tag_to_cam_lense.getRotation();
        tag_to_cam_lense.setRotation(r*_tag_offset_rotation);
        // apply a distance offset to the tag tf to move the goal backward from the tag
        tag_to_cam_lense*=_distance_offset;
        // combine all the transforms to the tag tf to create the goal for the world/global frame
        global_goal_tf = map_tf * robot_tf * cam_to_robot * cam_lense_to_cam * tag_to_cam_lense;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s",
                     camera_lense_frame_.c_str(), _tag_frame.c_str(), ex.what());
        // if the tag is not found and the timeout has passed, travel back to the home position
        if(!_is_travel_home && time_now-_last_publish >= std::chrono::duration<double>(goal_timeout_sec_)){
            _is_travel_home=true;
            publish_goal(_home_tf,map_frame_);
        }
        return;
    }
    _is_travel_home=false;
    _last_publish = time_now;
    publish_debug(map_frame_, "TAG_GLOBAL", global_goal_tf);
    publish_goal(global_goal_tf,map_frame_);
}

void PathBuilder::publish_goal(const tf2::Transform &tf, const std::string &frame_name){
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = frame_name;
    goal.header.stamp = this-> now();
    goal.pose.position.x = tf.getOrigin().x();
    goal.pose.position.y = tf.getOrigin().y();
    goal.pose.position.z = 0;
    goal.pose.orientation.x = tf.getRotation().x();
    goal.pose.orientation.y = tf.getRotation().y();
    goal.pose.orientation.z = tf.getRotation().z();
    goal.pose.orientation.w = tf.getRotation().w();
    goal_publisher_->publish(goal);
}

void PathBuilder::publish_debug(const std::string f_id, const std::string c_id, const tf2::Transform &tf){
    
    geometry_msgs::msg::TransformStamped tf_msg;
    geometry_msgs::msg::Vector3 v;
    v.x = tf.getOrigin().x();
    v.y = tf.getOrigin().y();
    v.z = tf.getOrigin().z();
    geometry_msgs::msg::Quaternion q;
    q.x = tf.getRotation().x();
    q.y = tf.getRotation().y();
    q.z = tf.getRotation().z();
    q.w = tf.getRotation().w();
    tf_msg.header.frame_id = f_id;
    tf_msg.child_frame_id = c_id;
    tf_msg.transform.translation = v;
    tf_msg.transform.rotation = q;
    if(debug_){
        _tf_broadcaster->sendTransform(tf_msg);
    }
    
}