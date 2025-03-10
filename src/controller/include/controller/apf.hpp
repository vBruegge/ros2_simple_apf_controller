#ifndef APF_CONTROLLER_HPP
#define APF_CONTROLLER_HPP


#include <eigen3/Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "obstacle/msg/obstacle_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "obstacle.hpp"

class APFController : public rclcpp::Node
{
    public:
        APFController();

    private:
        void control_loop();
        void obstacle_callback(const obstacle::msg::ObstacleArray& msg);
        void pose_callback(const geometry_msgs::msg::Pose& msg);
        void computeNextForce(Eigen::Vector2d& pos, Eigen::Vector2d& force, Eigen::Vector2d& attract_force, Eigen::Vector2d& reflect_force);
        void checkGoal(geometry_msgs::msg::Twist& msg, Eigen::Vector2d& attract_force);
        void avoidMinima(Eigen::Vector2d& force, Eigen::Vector2d& attract_force);
        void getMinDistance(Obstacle& obs, Eigen::Vector2d& pos, Eigen::Vector2d& dis);
        

    /**
    * @brief Get Arrows in RVIZ (used for force visualization)
    * 
    * @param vector vector to be visualized
    * @param robot_pose robot position in global CoSy
    * @param colour of the arrow
    * @return visualization_msgs::msg::Marker 
    */
    visualization_msgs::msg::Marker getArrow(const Eigen::Vector2d& pos, const Eigen::Vector2d &vector, 
        const std_msgs::msg::ColorRGBA &colour) const;

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Subscription<obstacle::msg::ObstacleArray>::SharedPtr obs_subscription;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription;

        std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> publisher_forces;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> vel_publisher;


        Eigen::Vector2d position;
        double heading;

        std::vector<Obstacle> obstacles;
        bool receivedObstacles;

        bool ignore_heading;
        double max_vel;
        double robot_length;
        double max_rot_vel;

        bool seek_source;
        Eigen::Vector2d goal_pos;
        double sigma_source;
        double grad_val;
        bool reached_goal;

        Eigen::Vector2d minima_location;
        rclcpp::Time minima_reached_at;
        bool minima_override;
        Eigen::Vector2d force_override;
        

};
#endif