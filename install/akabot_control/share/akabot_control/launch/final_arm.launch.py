#!/usr/bin/env python3
import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    
    # 1. SETUP PATHS
    pkg_gazebo = FindPackageShare('akabot_gazebo')
    pkg_description = FindPackageShare('akabot_description')
    pkg_moveit = FindPackageShare('akabot_moveit_config')
    pkg_ros_ign_gazebo = FindPackageShare('ros_ign_gazebo')


    world_path = PathJoinSubstitution([pkg_gazebo, 'worlds', 'pick_and_place_balls.world'])
    srdf_file = PathJoinSubstitution([pkg_moveit, 'srdf', 'akabot.srdf'])
    robot_description_file = PathJoinSubstitution([pkg_description, 'urdf', 'akabot_gz.urdf.xacro'])
    
    # 2. LOAD CONFIGURATIONS
    
    # Robot Description
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', robot_description_file]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # Semantic Description
    robot_description_semantic_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', srdf_file]
    )
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}
    
    # MoveIt Configs
    kinematics_config = load_yaml('akabot_moveit_config', 'config/kinematics.yaml')
    moveit_controllers = load_yaml('akabot_moveit_config', 'config/moveit_controllers.yaml')


    # 3. ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )


    # 4. LAUNCH GAZEBO
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world_path]}.items(),
    )


    # 5. SPAWN ROBOT - FIXED POSITION
    # Base plate sits on ground level (z=0)
    # The URDF definition handles lifting the arm structure to table height
    # base_plate_joint: z=0 (on table surface)
    # top_plate_joint: z=0.048 (revolute joint lifts it)
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "akabot",
            "-x",
            "-0.155216",
            "-y",
            "-0.056971",
            "-z",
            "1.010770",
            "-Y",
            "0.016798",
        ],
        output='screen'
    )


    # 6. BRIDGES (Direct Arguments - No Config File)
    # This bypasses the "YAML sequence" error completely.
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/ee_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )


    # Image Bridge
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/tf_camera/image_raw', 
            '/ee_camera/image_raw'
        ],
        output='screen'
    )


    # 7. CONTROLLERS
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


    # 8. MOVEIT
    move_group_node = TimerAction(
        period=15.0,
        actions=[Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_config,
                moveit_controllers,
                {
                    'use_sim_time': True,
                    'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                    'moveit_manage_controllers': True
                }
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )]
    )


    # 9. LOGIC NODES
    publish_tf = TimerAction(
        period=21.0,
        actions=[Node(
            package='akabot_control',
            executable='publish_tf',
            name='publish_tf',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )


    # 10. ROBUST BALL PICKER USING MOVEIT IK SERVICE (NEW)
    # Replaces analytical IK approach with proven MoveIt integration:
    # - Uses /compute_ik service (official MoveIt kinematics)
    # - Relaxes orientation constraints for 5-DOF arm
    # - Robust trajectory execution with error handling
    # - Waits 25 seconds for all systems to initialize
    # - Then executes 3-ball pickup sequence
    ball_picker_moveit = TimerAction(
        period=25.0,
        actions=[Node(
            package='akabot_control',
            executable='ball_picker_moveit',
            name='ball_picker_moveit',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )


    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        parameter_bridge,
        image_bridge,
        joint_state_broadcaster,
        arm_controller,
        hand_controller,
        move_group_node,
        publish_tf,
        ball_picker_moveit  # NEW: Robust MoveIt-based picker
    ])