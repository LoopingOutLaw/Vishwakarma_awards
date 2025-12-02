#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # 1. SETUP PATHS
    pkg_gazebo = FindPackageShare(package='akabot_gazebo').find('akabot_gazebo')
    pkg_description = FindPackageShare(package='akabot_description').find('akabot_description')
    pkg_ros_ign_gazebo = FindPackageShare(package='ros_ign_gazebo').find('ros_ign_gazebo')
    pkg_akabot_control = FindPackageShare('akabot_control')
    
    world_path = os.path.join(pkg_gazebo, 'worlds', 'pick_and_place_balls.world')
    urdf_model_path = os.path.join(pkg_description, 'urdf/akabot_gz.urdf.xacro')
    bridge_config_path = os.path.join(pkg_gazebo, 'config', 'akabot_bridge.yaml')

    # 2. ROBOT DESCRIPTION
    robot_description_config = Command(['xacro ', urdf_model_path])
    robot_description_str = ParameterValue(robot_description_config, value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_str, 'use_sim_time': True}]
    )

    # 3. LAUNCH GAZEBO
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 4. SPAWN ROBOT
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'akabot',
            '-x', '-0.155216', '-y', '-0.056971', '-z', '1.05', '-Y', '0.016798'
        ],
        output='screen'
    )

    # 5. BRIDGES
    
    # Parameter Bridge (Clock, Camera Info, Scan)
    # Using config file for stability
    param_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'],
        output='screen'
    )

    # Image Bridge (High bandwidth for Image Raw)
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/tf_camera/image_raw'],
        output='screen'
    )

    # 6. SPAWN CONTROLLERS
    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])]
    )
    
    arm_controller = TimerAction(
        period=8.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['akabot_arm_controller', '--controller-manager', '/controller_manager'])]
    )
    
    hand_controller = TimerAction(
        period=12.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['hand_controller', '--controller-manager', '/controller_manager'])]
    )

    # 7. BALL DETECTOR & TF PUBLISHER NODE
    # Switched to 'publish_tf' as requested
    ball_tf_publisher = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='akabot_control',
                executable='publish_tf', # Must match setup.py entry point
                name='publish_tf',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        param_bridge,
        image_bridge,
        joint_state_broadcaster,
        arm_controller,
        hand_controller,
        ball_tf_publisher
    ])