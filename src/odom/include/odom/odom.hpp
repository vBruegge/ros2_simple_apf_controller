#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <eigen3/Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"

class OdometryManager : public rclcpp::Node
{
    public:
        OdometryManager();

    private:
        /**
         * @brief receives geometry_msgs::msg::Twist velocity
         * 
         * @param msg velocity msg
         */
        void velocity_callback(const geometry_msgs::msg::Twist& msg);
        /**
         * @brief called once, saves starting position of robot
         * 
         * @param msg initial pose
         */
        void initialization(const geometry_msgs::msg::Pose& msg);
        /**
         * @brief pose and transform computation by integrating the robot velocity
         * 
         */
        void odometry_loop();
        /**
         * @brief publishes LineStrip as Marker to show driven path
         * 
         */
        void publish_path();

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscription;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr initial_pos_subscription;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Pose>> pose_publisher;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher;


        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::TimerBase::SharedPtr path_timer;


        Eigen::Vector2d position;
        double heading;
        bool receivedPosition;

        Eigen::Vector2d vel;
        double ang_vel;
        visualization_msgs::msg::Marker tracked_path;
};
#endif