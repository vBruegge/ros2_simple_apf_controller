cmake_minimum_required(VERSION 3.8)
project(environment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(obstacle REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)
include_directories(obstacle/include)

add_executable(map_generator_node src/map.cpp)
ament_target_dependencies(map_generator_node rclcpp Eigen3 visualization_msgs geometry_msgs obstacle tf2_ros)

install(TARGETS
  map_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch models include rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
