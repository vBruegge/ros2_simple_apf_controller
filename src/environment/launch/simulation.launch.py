import launch
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define Paths
    pkg_share = FindPackageShare(package='environment').find('environment')
    rviz_path = os.path.join(pkg_share, 'rviz/config.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    model_path = os.path.join(pkg_share, 'models/robot_1.urdf')
    #model_path = os.path.join(pkg_share, 'models/robot_2.urdf')

    model = ParameterValue(Command(['xacro ', model_path]),value_type=str)
    config = os.path.join(pkg_share, 'config', 'environment.yaml')
    source_file_name = os.path.join(pkg_share, 'maps', 'color_field.dae')
    print(source_file_name)

    use_sim_time = LaunchConfiguration('simulation')
    declare_use_sim_time = DeclareLaunchArgument(
        name="simulation",
        default_value='True',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': model}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d',rviz_path],
    )

    map_generator_node = Node(
        package="environment",
        executable="map_generator_node",
        parameters = [config, {'source_mesh': source_file_name}]
    )

    apf_controller_node = Node(
        package="controller",
        executable="apf_controller_node",
        parameters = [config]
    )

    odom_manager_node = Node(
        package="odom",
        executable="odom_manager_node",
    )
    map_server_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('environment'), 'launch'),
         '/map_server.launch.py']),
      )

    return launch.LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        rviz_node,
        #map_server_launch,
        map_generator_node,
        odom_manager_node,
        apf_controller_node,
    ])
