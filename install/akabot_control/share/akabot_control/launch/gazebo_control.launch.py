#!/usr/bin/env python3
"""
Launch akabot in Gazebo with pose control
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('akabot_gazebo'),
                'launch',
                'gz_launch.py'
            ])
        ])
    )
    
    # Include MoveIt RViz
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('akabot_moveit_config'),
                'launch',
                'moveit_rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'True'
        }.items()
    )
    
    # Controller node
    controller_node = Node(
        package='akabot_control',
        executable='akabot_controller',
        name='akabot_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        moveit_launch,
        controller_node,
    ])