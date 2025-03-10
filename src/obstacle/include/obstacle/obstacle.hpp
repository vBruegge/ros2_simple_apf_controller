#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <eigen3/Eigen/Core>

class Obstacle {
    public:
        Obstacle() = default;
        Obstacle(double length_, double width_, int shape_, Eigen::Vector2d origin_) :
            length(length_), width(width_), shape(shape_), origin(origin_) {};
    
        double length;
        double width;
        int shape;
        Eigen::Vector2d origin;
};
#endif