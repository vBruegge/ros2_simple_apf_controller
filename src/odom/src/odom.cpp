#include <odom/odom.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace std::chrono_literals;

#define LOOP_TIME 10ms

OdometryManager::OdometryManager() : Node("odom_manager"), position(), heading(0.0), receivedPosition(false), vel() {

    //initialize publisher/subscriber/timer
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    initial_pos_subscription = this->create_subscription<geometry_msgs::msg::Pose>("initial_position", 1,
            std::bind(&OdometryManager::initialization, this, std::placeholders::_1));

    vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>("velocity", 1,
        std::bind(&OdometryManager::velocity_callback, this, std::placeholders::_1));

    pose_publisher = this->create_publisher<geometry_msgs::msg::Pose>("odom",10);
    timer = this->create_wall_timer(LOOP_TIME, std::bind(&OdometryManager::odometry_loop, this));

    path_publisher = this->create_publisher<visualization_msgs::msg::Marker>("tracked_path", 1);
    path_timer = this->create_wall_timer(250ms, std::bind(&OdometryManager::publish_path, this));

    //initialize path marker
    tracked_path.header.frame_id = "world";
    tracked_path.action = visualization_msgs::msg::Marker::ADD;
    tracked_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    tracked_path.scale.x = 0.025;
    tracked_path.color.a = 1.0; // Don't forget to set the alpha!

    tracked_path.pose.orientation.w = 1.0;

}

void OdometryManager::initialization(const geometry_msgs::msg::Pose &msg) {
    position = Eigen::Vector2d(msg.position.x, msg.position.y);
    tf2::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, heading);

    RCLCPP_INFO(this->get_logger(), "Robot initial position: %.2f, %.2f\n", position[0], position[1]);
    receivedPosition = true;
    geometry_msgs::msg::Point pos;
    pos.x = position[0];
    pos.y = position[1];
    pos.z = 0.0;
    tracked_path.points.push_back(pos);
}


void OdometryManager::velocity_callback(const geometry_msgs::msg::Twist &msg) {
    vel << msg.linear.x, msg.linear.y;
    ang_vel = msg.angular.z;

}

void OdometryManager::publish_path()  {
    if(!receivedPosition)
        return;
    geometry_msgs::msg::Point pos;
    pos.x = position[0];
    pos.y = position[1];
    pos.z = 0.0;
    tracked_path.points.push_back(pos);
    tracked_path.header.stamp = rclcpp::Clock().now();
    

    path_publisher->publish(tracked_path);
}

void OdometryManager::odometry_loop() {
    //interrupt if no initial position was received
    if(!receivedPosition)
        return;

    //pose integration
    const double time = (double)LOOP_TIME.count()/1000.0;
    position = position + time * vel;
    heading += ang_vel*time;
    if(abs(heading) > M_PI)
        heading -= 2*M_PI;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading);
    
    //publish pose and transform
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp =  rclcpp::Clock().now();
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.x = position[0];
    t.transform.translation.y = position[1];
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    geometry_msgs::msg::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    pose_publisher->publish(pose);

    
}

int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryManager>());
    rclcpp::shutdown();
    return 0;
}