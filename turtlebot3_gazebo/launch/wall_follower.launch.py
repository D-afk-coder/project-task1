#!/usr/bin/env python3
#
# wall_follower.launch.py
# Starts the right wall follower (the turtlebot3_drive executable) with the
# parameters in params/wall_follower.yaml.
#
#   Real robot:  ros2 launch turtlebot3_gazebo wall_follower.launch.py
#   Gazebo:      ros2 launch turtlebot3_gazebo wall_follower.launch.py use_sim_time:=true
#
# A different parameter file can be passed with params_file:=/path/to/file.yaml,
# and namespace:=<ns> runs the node inside a namespace (so it uses
# /<ns>/scan and /<ns>/cmd_vel) if the robot's topics are namespaced.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'params', 'wall_follower.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true when running against Gazebo'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Wall follower parameter file'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the robot topics, if any'),
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_drive',
            name='turtlebot3_drive_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
