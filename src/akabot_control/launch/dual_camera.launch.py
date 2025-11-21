#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
    
    # Use the world with balls you uploaded previously
    world_path = os.path.join(pkg_gazebo, 'worlds', 'pick_and_place_balls.world')
    urdf_model_path = os.path.join(pkg_description, 'urdf/akabot_gz.urdf.xacro')

    # 2. ROBOT DESCRIPTION
    robot_description_config = Command(['xacro ', urdf_model_path])
    robot_description_str = ParameterValue(robot_description_config, value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
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
            '-x', '-0.155216', '-y', '-0.056971', '-z', '1.010770', '-Y', '0.016798'
        ],
        output='screen'
    )

    # 5. BRIDGES (The Fix)
    
    # A. CLOCK BRIDGE (Crucial for Rviz synchronization)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # B. CAMERA IMAGE BRIDGES (For both cameras)
    # Maps Gazebo topics to ROS topics
    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/ee_camera',               # Topic for End Effector Camera
            '/tf_camera/image_raw'      # Topic for External Camera (from previous turn)
        ],
        output='screen'
    )

    # C. CAMERA INFO BRIDGES (Optional, needed for PointClouds/Depth)
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ee_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/tf_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # 6. SPAWN CONTROLLERS (Standard sequence)
    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])]
    )
    
    arm_controller = TimerAction(
        period=10.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['akabot_arm_controller', '--controller-manager', '/controller_manager'])]
    )
    
    hand_controller = TimerAction(
        period=15.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['hand_controller', '--controller-manager', '/controller_manager'])]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        clock_bridge,           # <--- This fixes the "Dropping message" error
        camera_bridge,          # <--- Bridges both cameras
        camera_info_bridge,
        joint_state_broadcaster,
        arm_controller,
        hand_controller,
    ])