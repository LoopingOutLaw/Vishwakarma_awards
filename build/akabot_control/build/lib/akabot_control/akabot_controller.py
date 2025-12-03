#!/usr/bin/env python3
"""
Akabot Pose Controller - Simplified Version
Controls the akabot arm using MoveIt2 MoveGroupInterface
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
import numpy as np
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Duration


class AkabotController(Node):
    """Simple controller for akabot arm using IK service"""
    
    def __init__(self):
        super().__init__('akabot_controller')
        
        # Joint names
        self.joint_names = [
            'top_plate_joint',
            'lower_arm_joint',
            'upper_arm_joint',
            'wrist_joint',
            'claw_base_joint'
        ]
        
        # Current joint state
        self.current_joint_state = None
        self.joint_state_received = False
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # IK service client
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )
        
        # Action client for joint trajectory
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/akabot_arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Akabot Controller initialized')
        self.get_logger().info('Waiting for IK service...')
        self.ik_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('IK service available!')
        
        self.get_logger().info('Waiting for trajectory action server...')
        self.trajectory_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('Trajectory action server available!')
    
    def joint_state_callback(self, msg):
        """Callback to store current joint state"""
        self.current_joint_state = msg
        self.joint_state_received = True
    
    def create_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Create a Pose from position and Euler angles
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians
        
        Returns:
            geometry_msgs/Pose
        """
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        
        # Convert Euler to quaternion
        r = Rotation.from_euler('xyz', [roll, pitch, yaw])
        quat = r.as_quat()  # [x, y, z, w]
        
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        
        return pose
    
    def compute_ik(self, target_pose):
        """
        Compute inverse kinematics for a target pose
        Args:
            target_pose: geometry_msgs/Pose
        Returns:
            list: Joint positions if successful, None otherwise
        """
        if not self.joint_state_received:
            self.get_logger().error('No joint state received yet!')
            return None

        # Create IK request
        request = GetPositionIK.Request()

        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose

        request.ik_request.group_name = 'akabot_arm'
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.timeout = Duration(sec=5, nanosec=0)
        # REMOVE THIS LINE - 'attempts' attribute does not exist
        # request.ik_request.attempts = 10

        # Set robot state
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = self.current_joint_state

        # Call IK service
        self.get_logger().info('Computing IK...')
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('IK service call failed!')
            return None

        response = future.result()

        if response.error_code.val == 1:  # SUCCESS
            self.get_logger().info('IK solution found!')
            
            # Extract joint positions
            joint_positions = []
            for joint_name in self.joint_names:
                try:
                    idx = response.solution.joint_state.name.index(joint_name)
                    joint_positions.append(response.solution.joint_state.position[idx])
                except ValueError:
                    self.get_logger().error(f'Joint {joint_name} not found in IK solution')
                    return None
            
            return joint_positions
        else:
            self.get_logger().error(f'IK failed with error code: {response.error_code.val}')
            return None

    
    def execute_joint_trajectory(self, joint_positions, duration=5.0):
        """
        Execute a trajectory to reach target joint positions
        
        Args:
            joint_positions: List of target joint positions
            duration: Time to reach target (seconds)
            
        Returns:
            bool: True if successful
        """
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(duration), 
                                        nanosec=int((duration % 1) * 1e9))
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        # Send goal
        self.get_logger().info('Sending trajectory goal...')
        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected!')
            return False
        
        self.get_logger().info('Trajectory goal accepted, executing...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        
        if result.result.error_code == 0:  # SUCCESSFUL
            self.get_logger().info('Trajectory execution successful!')
            return True
        else:
            self.get_logger().error(f'Trajectory execution failed: {result.result.error_code}')
            return False
    
    def move_to_pose(self, target_pose, duration=5.0):
        """
        Move arm to a target pose
        
        Args:
            target_pose: geometry_msgs/Pose
            duration: Execution time in seconds
            
        Returns:
            bool: True if successful
        """
        # Compute IK
        joint_positions = self.compute_ik(target_pose)
        
        if joint_positions is None:
            return False
        
        self.get_logger().info(f'Target joint positions: {[f"{p:.3f}" for p in joint_positions]}')
        
        # Execute trajectory
        return self.execute_joint_trajectory(joint_positions, duration)
    
    def move_to_named_target(self, target_name):
        """
        Move to pre-defined named target from SRDF
        
        Available targets: home, ready, right, left, etc.
        """
        named_targets = {
            'home': [3.12, 1.5686, 0.0, 0.0, 0.0],
            'ready': [3.12, 2.182, -0.925, -1.1177, 0.0],
            'right': [1.56, 1.5169, -0.216, 0.0, 0.0],
            'left': [4.68, 1.5169, -0.216, 0.0, 0.0],
        }
        
        if target_name not in named_targets:
            self.get_logger().error(f'Unknown target: {target_name}')
            self.get_logger().info(f'Available targets: {list(named_targets.keys())}')
            return False
        
        self.get_logger().info(f'Moving to named target: {target_name}')
        return self.execute_joint_trajectory(named_targets[target_name], duration=5.0)


def main(args=None):
    rclpy.init(args=args)
    
    controller = AkabotController()
    
    # Wait for joint states
    controller.get_logger().info('Waiting for joint states...')
    while not controller.joint_state_received:
        rclpy.spin_once(controller, timeout_sec=0.1)
    
    controller.get_logger().info('Ready to accept commands!')
    
    # Example: Move to home position
    controller.get_logger().info('='*50)
    controller.get_logger().info('Moving to HOME position...')
    controller.move_to_named_target('home')
    
    # Example: Move to a specific pose
    controller.get_logger().info('='*50)
    controller.get_logger().info('Moving to target pose...')
    target_pose = controller.create_pose(
        x=0.15,   # 15cm forward
        y=0.0,    # centered
        z=0.25,   # 25cm up
        roll=0.0,
        pitch=np.pi/2,  # pointing down
        yaw=0.0
    )
    controller.move_to_pose(target_pose, duration=5.0)
    
    controller.get_logger().info('='*50)
    controller.get_logger().info('Demo complete!')
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()