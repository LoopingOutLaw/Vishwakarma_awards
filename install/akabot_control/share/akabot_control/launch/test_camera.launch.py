#!/usr/bin/env python3
"""
Simple launch file to test end effector camera in Gazebo
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Paths
    pkg_gazebo = FindPackageShare(package='akabot_gazebo').find('akabot_gazebo')
    world_path = os.path.join(pkg_gazebo, 'worlds', 'pick_and_place_balls.world')
    
    pkg_description = FindPackageShare(package='akabot_description').find('akabot_description')
    urdf_model_path = os.path.join(pkg_description, 'urdf/akabot_gz.urdf.xacro')
    
    # Robot description
    robot_description_config = Command(['xacro ', urdf_model_path])
    robot_description_str = ParameterValue(robot_description_config, value_type=str)
    
    # Launch Gazebo (using ros_gz_sim for newer Gazebo)
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
    
    # Bridge for end effector camera IMAGE
    # The camera publishes to /ee_camera on Gazebo side
    ee_camera_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/ee_camera',  # Gazebo topic
            '--ros-args',
            '-r', '/ee_camera:=/ee_camera/image_raw'  # Remap to ROS topic
        ],
        output='screen'
    )
    
    # Bridge for camera info
    ee_camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ee_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )
    
    # Spawn controllers
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
        period=8.0,
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
        period=11.0,
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
    
    # Camera viewer (auto-start)
    camera_viewer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='akabot_control',
                executable='test_ee_camera',
                name='test_ee_camera',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        ee_camera_image_bridge,
        ee_camera_info_bridge,
        joint_state_broadcaster,
        arm_controller,
        hand_controller,
        camera_viewer,
    ])