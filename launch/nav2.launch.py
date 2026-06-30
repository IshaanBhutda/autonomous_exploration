#!/usr/bin/env python3
"""
Nav2 bringup WITHOUT velocity_smoother.
controller_server publishes straight to /cmd_vel (the drive plugin's topic).
behavior_server (recovery spin/backup) also publishes to /cmd_vel.

Nodes: controller, planner, behavior, bt_navigator, lifecycle_manager.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('autonomous_exploration')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='true')

    lifecycle_nodes = ['controller_server', 'planner_server',
                       'behavior_server', 'bt_navigator']

    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=[params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel')])

    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}])

    behavior = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel')])

    bt_nav = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[params, {'use_sim_time': use_sim_time}])

    lifecycle = TimerAction(period=4.0, actions=[Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}])])

    return LaunchDescription([declare_sim, controller, planner,
                              behavior, bt_nav, lifecycle])
