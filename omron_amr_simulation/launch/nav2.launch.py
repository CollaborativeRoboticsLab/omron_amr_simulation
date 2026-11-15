import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulation_share = get_package_share_directory('omron_amr_simulation')
    nav2_share = get_package_share_directory('amr_nav2')
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true', description='Start an LD250 simulator'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo graphical client'),
        DeclareLaunchArgument('world', default_value='warehouse', description='Simulation world'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X in meters'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y in meters'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw in radians'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(nav2_share, 'config', 'ld250_nav2.yaml'),
                      description='Nav2 and SLAM parameters'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(simulation_share, 'launch', 'simulation.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_sim')),
            launch_arguments={
                'model': 'ld250', 'drive': 'true', 'lidar': 'true',
                'gui': LaunchConfiguration('gui'), 'world': LaunchConfiguration('world'),
                'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'), 'yaw': LaunchConfiguration('yaw'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'ld250.launch.py')),
            launch_arguments={
                'use_sim_time': 'true', 'use_namespace': 'false', 'slam': 'true',
                'use_composition': 'false', 'params_file': LaunchConfiguration('params_file'),
            }.items(),
        ),
    ])