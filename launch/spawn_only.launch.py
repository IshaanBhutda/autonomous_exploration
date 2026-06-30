#!/usr/bin/env python3
"""
STEP 1 - Spawn only. Gazebo (GUI) + rover + robot_state_publisher.
No SLAM, no Nav2. Confirm the rover spawns and sensors publish.

Verify after launch:
  ros2 topic list                  # /scan /camera/image_raw /odom /clock /tf
  ros2 topic hz /scan
  ros2 topic hz /camera/image_raw
  ros2 run tf2_tools view_frames

Drive by hand:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('autonomous_exploration')
    xacro_file = os.path.join(pkg, 'urdf', 'rover.urdf.xacro')
    default_world = os.path.join(pkg, 'worlds', 'custom_world.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_world = DeclareLaunchArgument('world', default_value=default_world)

    robot_description = xacro.process_file(xacro_file).toxml()

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])),
        launch_arguments={'world': world, 'verbose': 'true'}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])))

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', output='screen',
               parameters=[{'use_sim_time': use_sim_time,
                            'robot_description': robot_description}])

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description', '-entity', 'rover',
                            '-x', '0.0', '-y', '0.0', '-z', '0.15'],
                 output='screen')

    return LaunchDescription([declare_sim, declare_world,
                              gzserver, gzclient, rsp, spawn])
