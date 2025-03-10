#include <map/map.hpp>

#include <random>
#include <algorithm>
#include <string>
#include <chrono>

#include <tf2/LinearMath/Quaternion.h>


using namespace std::chrono_literals;

static inline int sign(double a) {
    if(a < 0)
        return -1.0;
    return 1.0;
}

MapGenerator::MapGenerator() : Node("map_generator"), obstacles() {

    //load parameter
    this->declare_parameter("map_dimension", 4.0);
    this->declare_parameter("min_obstacle_distance", 2.5);
    this->declare_parameter("goal_position", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->get_parameter("map_dimension", map_dim);
    this->get_parameter("min_obstacle_distance", obs_dis);

    std::vector<double> tmp;
    this->get_parameter("goal_position", tmp);
    Eigen::Vector2d goal = Eigen::Vector2d(tmp[0], tmp[1]);

    bool generate_T;
    this->declare_parameter("generate_T", false);
    this->get_parameter("generate_T", generate_T);
    this->declare_parameter("seek_source", false);
    this->get_parameter("seek_source", seek_source);
    this->declare_parameter("source_mesh", rclcpp::PARAMETER_STRING);
    this->get_parameter("source_mesh", mesh_file_name);
    
    //initialize rclcpp objects
    obs_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_vis", 1);
    obs_publisher = this->create_publisher<obstacle::msg::ObstacleArray>("obstacles", 10);
    robot_pos_publisher = this->create_publisher<geometry_msgs::msg::Pose>("initial_position", 10);

    timer = this->create_wall_timer(5s, std::bind(&MapGenerator::timer_callback, this));

    Eigen::Vector2d robot_pos;
    //generate T environment
    if(generate_T)  {
        double width_center, width_top, length;
        this->declare_parameter("width_T_center", 1.5);
        this->declare_parameter("width_T_top", 1.25);
        this->declare_parameter("length_T", 5.0);

        this->get_parameter("width_T_center", width_center);
        this->get_parameter("width_T_top", width_top);
        this->get_parameter("length_T", length);

        const double width = 0.1;
        obstacles.push_back(Obstacle(length*2.0, width, obstacle::msg::Obstacle::RECTANGULAR, Eigen::Vector2d(0.0, width_top/2.0)));
        obstacles.push_back(Obstacle(length-width_center/2, width, obstacle::msg::Obstacle::RECTANGULAR,
                    Eigen::Vector2d(width_center/4.0+length/2.0+width/2.0, -width_top/2.0)));
        obstacles.push_back(Obstacle(length-width_center/2, width, obstacle::msg::Obstacle::RECTANGULAR,
                    Eigen::Vector2d(-width_center/4.0-length/2.0-width/2.0, -width_top/2.0)));
        obstacles.push_back(Obstacle(width, length, obstacle::msg::Obstacle::RECTANGULAR,
                    Eigen::Vector2d(width_center/2.0, -width_top/2.0-length/2.0-width/2.0)));
        obstacles.push_back(Obstacle(width, length, obstacle::msg::Obstacle::RECTANGULAR,
                    Eigen::Vector2d(-width_center/2.0, -width_top/2.0-length/2.0+width/2.0)));
        obstacles.push_back(Obstacle(width_center, width, obstacle::msg::Obstacle::RECTANGULAR,
                    Eigen::Vector2d(0.0, -width_top/2.0-length)));
        
        //generate initial robot position
        generateRandPos(robot_pos, Eigen::Vector2d(-0.15, 0.15), Eigen::Vector2d(-width_top/2.0, -width_top/2.0-length));
        RCLCPP_INFO(this->get_logger(), "Generated T environment with %.2f and %.2f floor widths\n", width_center, width_top);
    }
    //generate random environment
    else {

        double obs_d;
        int obs_nbr;
        this->declare_parameter("obstacle_radius", 1.0);
        this->declare_parameter("obstacle_number", 4);
        this->get_parameter("obstacle_radius", obs_d);
        this->get_parameter("obstacle_number", obs_nbr);

        //generate random obstacle position
        for(int i = 0; i < obs_nbr; i++) {

            Eigen::Vector2d pos;
            do {
                Eigen::Vector2d range = Eigen::Vector2d(-map_dim/2.0, map_dim/2.0);
                generateRandPos(pos, range, range);
            }
            while(!checkDistance(pos, goal));
            obstacles.push_back(Obstacle(obs_d, obs_d, obstacle::msg::Obstacle::CIRCULAR, Eigen::Vector2d(pos[0], pos[1])));
            RCLCPP_INFO(this->get_logger(), "Obstacle position %i: %.2f, %.2f\n", i+1, pos[0], pos[1]);
        }
        
        //generate random robot position
        Eigen::Vector2d range = Eigen::Vector2d(-map_dim, map_dim);
        generateRandPos(robot_pos, range, range);
        if(!checkDistance(robot_pos, goal))
            robot_pos += Eigen::Vector2d(sign(robot_pos[0])*map_dim/2.0, sign(robot_pos[1])*map_dim/2.0);
    }

    //generate random heading
    double heading = generateRandNbr(Eigen::Vector2d(-M_PI, M_PI));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading);
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = robot_pos[0];
    pose.position.y = robot_pos[1];
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    visualizeObstacles();
    rclcpp::sleep_for(10s);

    robot_pos_publisher->publish(pose);

}

void MapGenerator::generateRandPos(Eigen::Vector2d& pos, const Eigen::Vector2d& rangeX, const Eigen::Vector2d& rangeY) {
    
    pos << generateRandNbr(rangeX), generateRandNbr(rangeY);
}

double MapGenerator::generateRandNbr(const Eigen::Vector2d& range) {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<double> dist(range[0],range[1]);
    return dist(rng);
}


bool MapGenerator::checkDistance(Eigen::Vector2d& pos, Eigen::Vector2d& goal) {
    for(auto & iobs : obstacles) {
        if ((iobs.origin - pos).norm() < obs_dis)
            return false;
    }
    if((goal - pos).norm() < obs_dis)
        return false;
    return true;
}

void MapGenerator::timer_callback() {
    visualizeObstacles();
    publishObstacles();
}

void MapGenerator::publishObstacles() {
    obstacle::msg::ObstacleArray obs_msg;
    for(auto & iobs : obstacles) {
        obstacle::msg::Obstacle obs;
        obs.origin.x = iobs.origin[0];
        obs.origin.y = iobs.origin[1];
        obs.shape = iobs.shape;
        obs.length = iobs.length;
        obs.width = iobs.width;
        obs_msg.obstacles.push_back(obs);
    }
    obs_msg.header.frame_id = "world";
    obs_msg.header.stamp = rclcpp::Clock().now();
    obs_publisher->publish(obs_msg);
}

void MapGenerator::visualizeObstacles() {
    const double height = 0.1;

    visualization_msgs::msg::MarkerArray obs;
    for(int i = 0; i < obstacles.size(); i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "obstacle";
        marker.id = i;
        if(obstacles[i].shape == obstacle::msg::Obstacle::CIRCULAR)
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
        else
            marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = obstacles[i].origin[0];
        marker.pose.position.y = obstacles[i].origin[1];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obstacles[i].length;
        marker.scale.y = obstacles[i]. width;
        marker.scale.z = height;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        obs.markers.push_back(marker);
    }

    if(seek_source) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "source";
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 1.5;
        marker.pose.position.y = -0.5;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.1;
        marker.mesh_resource = "file://" + mesh_file_name;
        marker.mesh_use_embedded_materials = true;
        obs.markers.push_back(marker);
    }
    obs_marker_publisher->publish(obs);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapGenerator>());
  rclcpp::shutdown();
  return 0;
}