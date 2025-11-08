from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the "Eyes"
        Node(
            package='arm_vla_pkg',
            executable='vision_node',
            name='vision_node'
        ),
        
        # Start the "Ears"
        Node(
            package='arm_vla_pkg',
            executable='speech_node',
            name='speech_node',
            output='screen' # So you can see "Listening..."
        ),

        # Start the "Brain"
        Node(
            package='arm_vla_pkg',
            executable='brain_node',
            name='brain_node',
            output='screen' # So you can see the AI's "thoughts"
        ),
    ])
