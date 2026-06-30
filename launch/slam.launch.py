#!/usr/bin/env python3
"""
STEP 3 - 2D SLAM with the lidar. Brings up the rover (Gazebo GUI),
SLAM Toolbox, and RViz. Drive manually with teleop to build the map.

  ros2 launch autonomous_exploration slam.launch.py
  # second terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save the finished map:
  ros2 run nav2_map_server map_saver_cli -f ~/my_room_map
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                            DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('autonomous_exploration')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='true')

    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('autonomous_exploration'), 'launch', 'spawn_only.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}], output='screen')

    slam = TimerAction(period=6.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'slam_params_file': slam_params}.items())
    ])

    return LaunchDescription([declare_sim, rover, rviz, slam])
