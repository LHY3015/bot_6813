#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gazebo_launch = '/opt/ros/humble/share/turtlebot3_gazebo/launch/turtlebot3_world.launch.py'
    slam_launch = '/opt/ros/humble/share/nav2_bringup/launch/slam_launch.py'
    nav2_launch = '/opt/ros/humble/share/turtlebot3_navigation2/launch/navigation2.launch.py'
    
    # define parameters
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_initial_x = DeclareLaunchArgument('initial_x', default_value='-2.0')
    declare_initial_y = DeclareLaunchArgument('initial_y', default_value='-0.5')
    declare_target_x = DeclareLaunchArgument('target_x', default_value='3.0')
    declare_target_y = DeclareLaunchArgument('target_y', default_value='3.0')


    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'x_pose': initial_x,
            'y_pose': initial_y
        }.items()
    )

    # SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # initial_alignment
    initial_alignment = Node(
        package='bot_6813',
        executable='initial_alignment',
        name='initial_alignment',
        output='screen',
        parameters=[{
            'initial_x': initial_x,
            'initial_y': initial_y,
            'use_sim_time': use_sim_time
        }]
    )

    # navigate
    navigate = Node(
        package='bot_6813',
        executable='navigate',
        name='navigate',
        output='screen',
        parameters=[{
            'target_x': target_x,
            'target_y': target_y,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        declare_initial_x,
        declare_initial_y,
        declare_target_x,
        declare_target_y,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        initial_alignment,
        navigate
    ])
