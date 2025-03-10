#include "controller/apf.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <eigen3/Eigen/Geometry>
#include <math.h>


using namespace std::chrono_literals;

#define LOOP_TIME 5ms
#define MINIMA_WAIT_TIME_S 2.0
#define MINIMA_OVERRIDE_TIME_S 1.0


static inline int sign(double a) {
    if(a < 0)
        return -1.0;
    return 1.0;
}

APFController::APFController() : Node("apf_controller"), obstacles(), receivedObstacles(false), grad_val(100.0), reached_goal(false),
            minima_location(), minima_override(false) {

    //initialize rclcpp objects
    obs_subscription = this->create_subscription<obstacle::msg::ObstacleArray>("obstacles", 1,
            std::bind(&APFController::obstacle_callback, this, std::placeholders::_1));
    
    pose_subscription = this->create_subscription<geometry_msgs::msg::Pose>("odom", 1,
                std::bind(&APFController::pose_callback, this, std::placeholders::_1));

    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("velocity",10);


    publisher_forces = this->create_publisher<visualization_msgs::msg::MarkerArray>("forces",10);

    timer = this->create_wall_timer(LOOP_TIME, std::bind(&APFController::control_loop, this));
    
    //declare/get parameter
    this->declare_parameter("ignore_heading", false);
    this->get_parameter("ignore_heading", ignore_heading);
    this->declare_parameter("max_velocity", 5.0);
    this->get_parameter("max_velocity", max_vel);
    this->declare_parameter("robot_length", 1.5);
    this->get_parameter("robot_length", robot_length);
    this->declare_parameter("seek_source", false);
    this->get_parameter("seek_source", seek_source);
    this->declare_parameter("max_rot_speed_deg", 90.0);
    this->get_parameter("max_rot_speed_deg", max_rot_vel);
    this->declare_parameter("sigma_source", 1.0);
    this->get_parameter("sigma_source", sigma_source);

    std::vector<double> tmp;
    this->declare_parameter("goal_position", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->get_parameter("goal_position", tmp);
    goal_pos << tmp[0], tmp[1];
    max_rot_vel *= M_PI/180.0;

    minima_reached_at = rclcpp::Clock().now();
}    

void APFController::obstacle_callback(const obstacle::msg::ObstacleArray &msg) {
    //save obstacles from msg in obstacles std::vector
    for(int i = 0; i < msg.obstacles.size(); i++) {
        Obstacle obs = Obstacle(msg.obstacles[i].length, msg.obstacles[i].width, msg.obstacles[i].shape,
                    Eigen::Vector2d(msg.obstacles[i].origin.x, msg.obstacles[i].origin.y));
        obstacles.push_back(obs);
    }
    if(receivedObstacles == false || position.norm() < 0.1) {
        receivedObstacles = true;
        start = rclcpp::Clock().now(); //start time measurement
    }
}

void APFController::pose_callback(const geometry_msgs::msg::Pose &msg) {
    //save robot pose
    position = Eigen::Vector2d(msg.position.x, msg.position.y);
    tf2::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, heading);
}

void APFController::getMinDistance(Obstacle &obs, Eigen::Vector2d& pos, Eigen::Vector2d& dis) {

    if(abs(obs.origin[0]-pos[0]) < obs.length/2.0)
        dis << 0.0, pos[1]-obs.origin[1];
    else if(abs(obs.origin[1]-pos[1]) < obs.width/2.0)
        dis << pos[0] - obs.origin[0], 0.0;
    else {
        dis = pos - obs.origin;
        dis -= Eigen::Vector2d(sign(dis[0])*obs.length/2.0, sign(dis[1])*obs.width/2);
    }
}

void APFController::computeNextForce(Eigen::Vector2d& pos, Eigen::Vector2d& force, Eigen::Vector2d& attract_force, Eigen::Vector2d& reflect_force) {
    //attraction force computation
    if(!seek_source)
        attract_force = goal_pos-pos;
    else {
        attract_force << 0.005*(7.0*cos(0.7*pos[0])*cos(0.7*pos[1]))-(pos[0]*exp(-(pow(pos[0],2)+pow(pos[1],2))/(2*pow(sigma_source,2))))/pow(sigma_source, 2)
                -(exp(-(2*(pow(pos[0]-2,2)+pow(pos[1]+1,2)))/pow(sigma_source,2))*(2*pos[0]-4))/(5*pow(sigma_source,2)),
            -0.005*(7.0*sin(0.7*pos[0])*sin(0.7*pos[1]))-(pos[1]*exp(-(pow(pos[0],2)+pow(pos[1],2))/(2*pow(sigma_source,2))))/pow(sigma_source, 2)
                -(exp(-(2*(pow(pos[0]-2,2)+pow(pos[1]+1,2)))/pow(sigma_source,2))*(2*pos[1]+2))/(5*pow(sigma_source,2));     
        
        attract_force *= 12.0;   
    }

    //reflection force computation, done per obstacle
    reflect_force << 0.0, 0.0;
    for(auto & iobs : obstacles) {
        Eigen::Vector2d distance;
        if(iobs.shape == obstacle::msg::Obstacle::CIRCULAR) {
            distance = (pos-iobs.origin);
            distance -= distance.normalized()*iobs.length;
        }
        else 
            getMinDistance(iobs, pos, distance);
        reflect_force += 1.0/pow(distance.norm(),2) * distance.normalized();
        
    }

    //compute combined force, cap it and avoid minima
    force = reflect_force + attract_force;
    if(force.norm() > max_vel)
        force = force.normalized() * max_vel;
    avoidMinima(force, attract_force);
    /*
    RCLCPP_INFO(rclcpp::get_logger("reflect_force"), "x: %f, y: %f", 
        reflect_force[0], reflect_force[1]);
    RCLCPP_INFO(rclcpp::get_logger("attract_force"), "x: %f, y: %f", 
        attract_force[0], attract_force[1]);
    RCLCPP_INFO(rclcpp::get_logger("combined_force"), "x: %f, y: %f", 
        force[0], force[1]);
    */
}

void APFController::avoidMinima(Eigen::Vector2d& force, Eigen::Vector2d& attract_force) {

    rclcpp::Time now = rclcpp::Clock().now();
    //override if robot was stuck for longer than MINIMA_WAIT_TIME_S
    if(minima_override) {
        if((now-minima_reached_at).seconds() < MINIMA_OVERRIDE_TIME_S + MINIMA_WAIT_TIME_S)
            force = force_override;
        else
            minima_override = false;
    }
    else if(force.norm() < 0.3) {
        //add vector perpendicular to the attracting force
        Eigen::Vector3d tmp_cross;
        tmp_cross << attract_force[0], attract_force[1], 0.0;
        tmp_cross = tmp_cross.cross(Eigen::Vector3d::UnitZ());
        Eigen::Vector2d add;
        add << tmp_cross[0], tmp_cross[1];
        force += add;

        //safe position and start timer
        if((minima_location-position).norm() > 0.1) {
            minima_reached_at = now;
            minima_location = position;
        }
        else {
            //start override
            if((now-minima_reached_at).seconds() > MINIMA_WAIT_TIME_S) {
                force = attract_force;
                minima_override = true;
                force_override = attract_force;
            }
        }
    }
}

void APFController::control_loop() {
    //interrupt if not all data was received
    if(!receivedObstacles || position.norm() < 0.1)
        return;

    Eigen::Vector2d force, attract_force, reflect_force;
    computeNextForce(position, force, attract_force, reflect_force);

    geometry_msgs::msg::Twist msg;
    msg.linear.x = force[0];
    msg.linear.y = force[1];

    //heading control using asymetric force comparison
    visualization_msgs::msg::MarkerArray forces;
    if(!ignore_heading && obstacles.size() > 0) {
        Eigen::Vector2d front = position + Eigen::Vector2d(cos(heading), sin(heading)*robot_length/2);
        
        Eigen::Vector2d force_back, force_front, attract_tmp, reflect_tmp;
        computeNextForce(front, force_front, attract_tmp, reflect_tmp);

        std_msgs::msg::ColorRGBA color_turn;
        color_turn.b = (136.0f/255.0f);
        color_turn.a = 0.5;
        forces.markers.push_back(getArrow(front, force_front, color_turn));
        Eigen::Vector2d combined = force_front + force;
        msg.angular.z = std::max(std::min(std::atan2(combined[1], combined[0]), max_rot_vel),-max_rot_vel);
        //RCLCPP_INFO_STREAM(this->get_logger(), "Combined: " << combined);
        //RCLCPP_INFO(this->get_logger(), "angular velocity: %.2f\n", msg.angular.z);


    }

    //check if goal was reached
    checkGoal(msg, attract_force);
    vel_publisher->publish(msg);

    //publish force arrows in RVIZ
    std_msgs::msg::ColorRGBA color_combined;
    color_combined.a = 1.0;
    color_combined.b = (136.0f/255.0f);
    std_msgs::msg::ColorRGBA color_attraction;
    color_attraction.a = 1.0;
    color_attraction.g = 1.0;
    std_msgs::msg::ColorRGBA color_reflection;
    color_reflection.a = 1.0;
    color_reflection.r = 1.0;
    forces.markers.push_back(getArrow(position, force, color_combined));
    forces.markers.push_back(getArrow(position, attract_force, color_attraction));
    forces.markers.push_back(getArrow(position, reflect_force, color_reflection));
    for(int i = 0; i < 4; i++){
        if(ignore_heading && i == 3)
            break;
        forces.markers[i].id = i;
    }
    publisher_forces->publish(forces);
}

void APFController::checkGoal(geometry_msgs::msg::Twist& msg, Eigen::Vector2d& attract_force) {
    if(!seek_source) {
        if((goal_pos-position).norm() < 0.15) {
            //robot reached goal
            if(!reached_goal) {
                RCLCPP_INFO(this->get_logger(), "Reached goal! Current position: %.2f, %.2f\n", position[0], position[1]);
                RCLCPP_INFO(this->get_logger(), "Needed time: %.2f\n", (rclcpp::Clock().now()-start).seconds());
            }
            reached_goal = true;
        }
        else if((goal_pos-position).norm() < 0.5) {
            //robot near goal
            msg.linear.x = 0.5*attract_force[0];
            msg.linear.y = 0.5*attract_force[1];
        }
    }
    else {
        static int counter = 0;
        if(attract_force.norm() > 0.01 || abs(attract_force.norm()-grad_val) > 5e-4){
            //minimum not reached yet
            grad_val = attract_force.norm();
            counter = 0;
            //robot near minima (velocity override)
            if(obstacles.size() > 0 && attract_force.norm() < 0.2) {
                msg.linear.x = 0.5*attract_force[0];
                msg.linear.y = 0.5*attract_force[1];
            }
        }
        else {
            //minimum reached and wait time exceeded
            if(counter > 50 && !reached_goal) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Found source concentration maximum at: " << position);
                RCLCPP_INFO(this->get_logger(), "Needed time: %.2f\n", (rclcpp::Clock().now()-start).seconds());
                reached_goal = true;
            }
            counter++;
        }
    }
    //turn heading to 0
    if(reached_goal) {
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        if(!ignore_heading && abs(heading) > 0.01)
            msg.angular.z = -sign(heading)* M_PI/6.0;
        else
            msg.angular.z = 0.0;
    }
    
}

visualization_msgs::msg::Marker APFController::getArrow
  (const Eigen::Vector2d&pos, const Eigen::Vector2d &vector, const std_msgs::msg::ColorRGBA &colour) const{
    // Creating the marker and initialising its fields

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = rclcpp::Clock().now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.ns = "forces";
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color = colour;
    marker.points.resize(2);
    marker.points[0].x = pos[0];
    marker.points[0].y = pos[1];
    marker.points[0].z = 0.0;
    marker.points[1].x = vector[0]+pos[0];
    marker.points[1].y = vector[1]+pos[1];
    marker.points[1].z = 0.0;

    return marker;
}

int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<APFController>());
    rclcpp::shutdown();
    return 0;
}