#!/usr/bin/env python3
"""
Launch file for vision-based pick and place in Gazebo
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Path to the new world file with balls
    pkg_gazebo = FindPackageShare(package='akabot_gazebo').find('akabot_gazebo')
    world_path = os.path.join(pkg_gazebo, 'worlds', 'pick_and_place_balls.world')
    
    # Include Gazebo launch with new world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Robot state publisher
    pkg_description = FindPackageShare(package='akabot_description').find('akabot_description')
    urdf_model_path = os.path.join(pkg_description, 'urdf/akabot_gz.urdf.xacro')
    
    from launch.substitutions import Command
    from launch_ros.parameter_descriptions import ParameterValue
    
    robot_description_config = Command(['xacro ', urdf_model_path])
    robot_description_str = ParameterValue(robot_description_config, value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_str,
            'use_sim_time': True
        }]
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'akabot',
            '-x', '-0.155216',
            '-y', '-0.056971',
            '-z', '1.010770',
            '-Y', '0.016798'
        ],
        output='screen'
    )
    
    # Bridge for end effector camera
    ee_camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/ee_camera/image_raw'],
        output='screen'
    )
    
    # Spawn controllers with delays
    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            )
        ]
    )
    
    arm_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'akabot_arm_controller',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            )
        ]
    )
    
    hand_controller = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'hand_controller',
                    '--controller-manager', '/controller_manager'
                ],
                output='screen'
            )
        ]
    )
    
    # MoveIt launch
    moveit_launch = TimerAction(
        period=18.0,
        actions=[
            IncludeLaunchDescription(
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
        ]
    )
    
    # Vision pick and place controller
    vision_controller = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='akabot_control',
                executable='vision_pick_place',
                name='vision_pick_place',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        ee_camera_bridge,
        joint_state_broadcaster,
        arm_controller,
        hand_controller,
        moveit_launch,
        vision_controller,
    ])

