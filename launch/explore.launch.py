#!/usr/bin/env python3
"""
LAYER 2 - Fully autonomous frontier exploration.

rover (Gazebo) + SLAM Toolbox + Nav2 + RViz + frontier explorer.
The explorer finds unknown-space frontiers and feeds goals to Nav2
automatically. No clicking - the rover explores the whole building on
its own and stops when no frontiers remain.

  ros2 launch autonomous_exploration explore.launch.py

Startup:
  t=0   rover + RViz
  t=6   SLAM Toolbox
  t=10  Nav2
  t=16  frontier explorer (needs /map + Nav2 action server)
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
    explore_params = os.path.join(pkg, 'config', 'explore_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'nav.rviz')

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

    nav2 = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('autonomous_exploration'), 'launch', 'nav2.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items())
    ])

    explorer = TimerAction(period=16.0, actions=[
        Node(package='autonomous_exploration', executable='explore',
             name='explorer', output='screen',
             parameters=[explore_params, {'use_sim_time': use_sim_time}])
    ])

    return LaunchDescription([declare_sim, rover, rviz, slam, nav2,
                              explorer])
