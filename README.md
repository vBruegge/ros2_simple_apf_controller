# Solution for Recruitment Assigment

## Executing the code

### Building
The code is configured to be build using `colcon`. Run

    colcon build

in the `ros2_ws` folder.

### Running
Use 

    . install/setup.bash
    ros2 launch environment simulation.launch.py

in the `ros2_ws` folder to run the code.
Configuration is done using the `environment.yaml` in the `config`folder of the `environment` package.
For deciding which robot should be used during simulation comment the other line in the `simulation.launch.py` file as shown below:

    #model_path = os.path.join(pkg_share, 'models/robot_1.urdf')
    model_path = os.path.join(pkg_share, 'models/robot_2.urdf')

**Note:** To be able to publish the source distribution a mesh-file was generated from a picture as a work-around. A first try was to publish an `nav2::msg::OccupancyGrid`with the map however the topic is not received from rviz. The map server can be called using the `map_server.launch.py`-script (already included in the `simulation.launch.py`-file). To be able to load the large mesh-file and also guarantee that the odometry packages is up and running, the map generator sleeps 10s before publishing the `Obstacle.msg`. 

### Testing
The code was tested on an Ubuntu 22.04 LTS version running ros2 humble.

## Documentation
### General information

The complete code does not use any libraries for simulation and control. Therefore some solutions may not be the most ideal or direct way to tacle the specific problem. My approach was rather to build a flexible and adaptable environment which can be easiliy configured and is flexible to adapt to different conditions. The whole assignment could have bin done using the ros2 navigation2 package by building up a config file and an environment in Gazebo and is in most cases a more reliable way to achieve a good solution, however its scope exceeds the scope of the assignment significantly which is the reason why a small and simple controller based on an artificial potential field (APF) was used to generate these results. The robots are visualized using 2 `URDF`-files where the origin is the center of the robot.

### Packages

The software consists of 4 packages
    - environment
    - controller
    - odom
    - obstacle

The `obstacle`-package consists only of a header defining the `Obstacle` class and generated the custom message types `Obstacle.msg` and `ObstacleArray.msg`. The `odom`-package integrates the `geopmetry_msgs::msg::Twist velocitiy`-messages generated by the controller, integrates them and publishes transforms and `geometry_msgs::msg::Pose odom` for the localization. The `environment`-package generates the desired environment, publishes `obstacles`-messages and generates also the initial position of the robots. The `controller`-package receives `obstacles`- and`odom`-messages to generate a velocity in which the robot has to move to reach the target position.

![rqt_graph](/img/rqt_graph.png)

### Visualization
The complete visuaization is done in rviz using marker and an `URDF`-file dor the robot. This means that the obstacles do not have any physical properties and are just used to show the funtionality of the controller. There are markers publishing:
- obstacles (red)
- robot (blue/cyan)
- the forces of the APF (red reflcting, green attracting and blue combined) as well as a transparent vector used for the heading control
- driven path (black line)
- (source seeking) a mesh showing the distribution

![rviz_visualization](ros2_ws/img/visual.png)
![rviz_visualization_source](ros2_ws/img/visual2.png)

### Functionalities of the controller

#### APF-principle
For the APF, the robot is approximated as a point mass moveable in any direction. The movement is based on forces similar to an el. potential field. The goal acts as attracting force obstacles as reflecting, the addition of the two forces result in the vector of movement (vgl. figure).
![apf](ros2_ws/img/apf.jpg) [Ref: https://doi.org/10.1016/j.jestch.2023.101343]

Classic APF algorithms struggle by:
- environments with local minima
- moving in confined spaces

The reflection force function per obstacle is prescribed by : f_Oi = 1/(dis^2)*norm(dis)

#### Minima Avoidance
For solving the first issue a routine handling local minimas was programmed. It consists of 2 phases.
**Phase 1: The robot drives straight into an obstacle**
A vector perpendicular to the attracting vector is added to the combined (almost zero) vector.
![minima1](ros2_ws/img/minima1.png)

**Phase 2: The robot is stuck between 2 obstacles**
When a minima is detected, a timer starts running. after exceeding a defined space without movement, the controller expects to be stuck between 2 obstacles and the combined force is overriden using half of the attracting force.
![minima2](ros2_ws/img/minima2.png)

#### Moving in confined spaces 
The second issue could not be resolved completely which is why question 1b is only partly solved. The velocity has to be reduced to be able to steer more accurately.

#### Goal reaching
Since the programmed APF-controller acconts for all obstacles in the confined space, the exact goal is impossible to reach. Therefore the reflection force is overriden near the goal or when the attraction force vector of the source approaches 0. When the goal was reached or the maximum of the source distribution is reached, the velocity commands are set to 0.

#### Heading control
The heading is controlled using a comparison of an assymetric force computation. Additional to the force at the origin, the APF-force at the front (point A) is computed. The angle between these vectors (alpha) is used as angular velocity.
![heading_control](ros2_ws/img/heading.png)

### Configuration
#### Environment (map_generator)

The obstacles are generated randomly in the range of `(-map_dimension/2, map_dimension/2)` using an uniform distribution. To guarantee that there is enough space to fit through to obstacles `min_obstacle_distance` in the config-file. The diameter and number of obstcales is configured using `obstacle_number`, `obstacle_radius`. The controller is configurable to lead to any location on the map using the `goal_position` parameter. To guarantee that obstacles have enough distance to the goal position, the parameter is loaded during the map generation and used for a distance comparison to all obstacles as well. To generate a T-floor plan `generate_T` has to be set to true. `width_T_center`, `width_T_top` and `length_T` is then used to configure the width and length of the different T-sections.The robot initial position and heading is also generated randomly.
**Note:** It is recommended to set the source location as `goal_position` to guarantee enough distance to the random obstacles. To generate an environment without obstacles, `obstacle_number` has to be set to 0.

#### Controller (apf_controller)
The controller used is based on an *artificial potential field*. To heading computation can be ignored using the `ignore_heading` parameter. The maximum (rotational) velocity is set using `max_velocity` and `max_rot_speed_deg`. `robot_length` is necessary for the heading control and defines the length of the robot. `signma_source` configures the shape of the source function. As mentioned above `goal_position` is used to set the goal of the robot, however it will be ignored if `seek_source` is set to `true`.


    /map_generator:
        ros__parameters:
            map_dimension: 6.0
            min_obstacle_distance: 2.1
            generate_T: false
            width_T_center: 1.5
            width_T_top: 1.25
            length_T: 5.0
            obstacle_number: 4
            obstacle_diameter: 1.0

        
    /apf_controller:
        ros__parameters:
            ignore_heading: false
            max_vel: 5.0
            robot_length: 1.5
            sigma_source: 4.0
            max_rot_speed_deg: 120.0

    /**:
        ros__parameters:
            goal_position: [0.84, -0.2] #set source [0.84, -0.2] as goal to guarantee distance to obstacles
            seek_source: true

## Question 1

The goal is reached by setting the goal to a desired position. The attracting force of the APF points in the direction of the goal until the goal is reached. Obstacles act as reflecting forces normal to the surface of the obstacle onto the robot.

### Question 1a
The robot reaches the origin ~6s after it received the first `Obstacle.msg`. Succesful examples are presented in the following.
![1a1](ros2_ws/img/1a-1.png)
![1a2](ros2_ws/img/1a-2.png)
![1a3](ros2_ws/img/1a-3.png)
![1a4](ros2_ws/img/1a-4.png)

### Question 1b
The robot is able to move between the obstacles, however when it reaches the end of the corridor, it runs in an local minima. The override kicks in and the controller drives through the obstacle.
Possible solutions for this issue:
- reduce the force generated by obstacles direct in front when driving in confined spaces
- add an routine which checks if an obstacle is in front of the robot before overriding the force vector

![1b1](ros2_ws/img/t_fail1.png)
![1b2](ros2_ws/img/t_fail2.png)
The Robot is touching is touching the wall, since the controller is right now not accounting for any dimension of the robot and assumes it as point.

The mechanism controlling the heading works nonetheless.
![1a3](ros2_ws/img/t_3.png)

## Question 2
To adapt the code of Question 1, the attracting force is replaced by the gradient of the source field. The gradient was generated using MATLAB Symbolic Toolbox. To be able to navigate in an environment with obstacles, the attraction force had to be scaled.

### Question 2a
The robot needs around 8 seconds to reach the source from any point.
![2a1](ros2_ws/img/2a-1.png)
![2a2](ros2_ws/img/2a-2.png)
![2a3](ros2_ws/img/2a-3.png)
![2a4](ros2_ws/img/2a-4.png)

### Question 2b
The goal is reached by the controller, however since the reflecting force is non-zero it does not stop completely.
![2b1](ros2_ws/img/2b-1.png)
![2b2](ros2_ws/img/2b-2.png)
![2b3](ros2_ws/img/2b-3.png)
![2b4](ros2_ws/img/2b-4.png)

Since the environment is generated randomly, it is possible, that the attraction force is to small and the robot gets stuck. This happens only rarely, when the minima routine cannot escape from the local minimum. The normal case is depicted in pic. 4.

![2b-fail](ros2_ws/img/2b-fail.png)

## Outlook

The software is also capable handling complexer environments without further adaptions as shown below.
![outlook](ros2_ws/img/complex.png)