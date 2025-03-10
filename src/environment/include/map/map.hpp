#ifndef MAP_HPP
#define MAP_HPP

#include <functional>
#include <memory>
#include <vector>
#include <string>

#include <eigen3/Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "obstacle/msg/obstacle_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "obstacle.hpp"

class MapGenerator : public rclcpp::Node
{
public:
    MapGenerator();
private:
    void timer_callback();
    bool checkDistance(Eigen::Vector2d& pos, Eigen::Vector2d& goal);
    void visualizeObstacles();
    void publishObstacles();
    void generateRandPos(Eigen::Vector2d& pos, const Eigen::Vector2d& rangeX, const Eigen::Vector2d& rangeY);
    double generateRandNbr(const Eigen::Vector2d& range);

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_marker_publisher;
    rclcpp::Publisher<obstacle::msg::ObstacleArray>::SharedPtr obs_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr robot_pos_publisher;


    std::vector<Obstacle> obstacles;
    int obs_nbr;
    double map_dim;
    double obs_dis;
    bool seek_source;
    std::string mesh_file_name;

};
#endif