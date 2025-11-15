#!/usr/bin/env python3
"""
Simple demo launch - Uses existing MoveIt demo launch with controller
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Include the existing MoveIt demo launch from akabot_moveit_config
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('akabot_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ])
    )
    
    # Controller node - wait a bit for MoveIt to start
    controller_node = TimerAction(
        period=5.0,  # Wait 5 seconds for MoveIt to initialize
        actions=[
            Node(
                package='akabot_control',
                executable='akabot_controller',
                name='akabot_controller',
                output='screen',
                parameters=[
                    {'use_sim_time': False}
                ]
            )
        ]
    )
    
    return LaunchDescription([
        moveit_demo_launch,
        controller_node,
    ])
