#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool, Hyungyu Kim
#
# Modified for MTRX3760 Project 1 from turtlebot3_world.launch.py.
#
# Starts the maze simulation for the right wall follower:
#   - Gazebo with worlds/turtlebot3_maze.world
#   - the TurtleBot 3 named by TURTLEBOT3_MODEL, in the south-west cell of the
#     maze, facing east with the outer wall 0.3 m to its right
#   - RViz showing the laser scan, the camera image and the path driven
#
# The wall follower itself is started separately:
#   ros2 run turtlebot3_gazebo turtlebot3_drive
#
# Launch arguments:
#   x_pose, y_pose  where the robot starts [m]
#   rviz            set to false to run without RViz

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(package_dir, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.5')
    y_pose = LaunchConfiguration('y_pose', default='0.33')
    rviz = LaunchConfiguration('rviz')

    world = os.path.join(package_dir, 'worlds', 'turtlebot3_maze.world')
    rviz_config = os.path.join(package_dir, 'rviz', 'tb3_maze.rviz')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz with the laser, camera and path displays')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz)
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(package_dir, 'models'))

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_rviz_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(set_env_vars_resources)

    return ld
