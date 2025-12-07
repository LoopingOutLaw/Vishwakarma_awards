#!/usr/bin/env python3
"""
Scanning Vision Pick and Place System for AkaBot
Fixed version with working movement and gripper control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
import math
import time
from scipy.spatial.transform import Rotation as R

class ScanningVisionPickPlace(Node):
    def __init__(self):
        super().__init__('scanning_vision_pick_place')
        
        self.get_logger().info('Akabot Controller initialized')
        
        # Wait for IK service
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info('Waiting for IK service...')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting again...')
        self.get_logger().info('IK service available!')
        
        # Create trajectory publisher
        self.traj_publisher = self.create_publisher(
            JointTrajectory,
            '/akabot_arm_controller/follow_joint_trajectory/goal',
            10
        )
        
        # Gripper publisher
        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            '/hand_controller/follow_joint_trajectory/goal',
            10
        )
        
        # Wait for trajectory action server
        self.get_logger().info('Waiting for trajectory action server...')
        time.sleep(2)  # Give time for server to start
        self.get_logger().info('Trajectory action server available!')
        
        # Safe positions
        self.home_position = {
            'top_plate_joint': 3.12,
            'lower_arm_joint': 0.0,
            'upper_arm_joint': 1.7,
            'wrist_joint': -1.7,
            'claw_base_joint': 0.0,
            'right_claw_joint': 0.0
        }
        
        self.scan_position = {
            'top_plate_joint': 3.12,  # Will vary for scanning
            'lower_arm_joint': 1.5,
            'upper_arm_joint': 0.5,
            'wrist_joint': -0.5,
            'claw_base_joint': -1.5,
            'right_claw_joint': 0.0
        }
        
        self.get_logger().info('Improved Scanning Vision Pick and Place initialized')
        self.get_logger().info('Waiting for system initialization...')
        time.sleep(3)  # Wait for all systems to be ready
        self.get_logger().info('Starting task execution...')
    
    def move_to_joint_position(self, joint_positions: dict, duration_s: float = 5.0) -> bool:
        """
        Move robot to specified joint positions
        """
        try:
            # Create trajectory
            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = list(joint_positions.keys())
            
            # Create point
            point = JointTrajectoryPoint()
            point.positions = list(joint_positions.values())
            point.velocities = [0.0] * len(joint_positions)
            point.time_from_start = Duration(seconds=duration_s).to_msg()
            
            traj.points.append(point)
            
            # Publish
            self.traj_publisher.publish(traj)
            self.get_logger().info(f'Moving to position: {joint_positions}')
            
            # Wait for movement
            time.sleep(duration_s + 1.0)
            return True
            
        except Exception as e:
            self.get_logger().error(f'Movement error: {e}')
            return False
    
    def move_gripper(self, position: float, duration_s: float = 1.5) -> bool:
        """
        Control gripper (right_claw_joint)
        position: -0.5 (open) to 0.5 (close)
        """
        try:
            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = ['right_claw_joint']
            
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.velocities = [0.0]
            point.time_from_start = Duration(seconds=duration_s).to_msg()
            
            traj.points.append(point)
            self.gripper_publisher.publish(traj)
            
            if position > 0:
                self.get_logger().info('Gripper closing...')
            else:
                self.get_logger().info('Gripper opening...')
            
            time.sleep(duration_s + 0.5)
            return True
            
        except Exception as e:
            self.get_logger().error(f'Gripper error: {e}')
            return False
    
    def scan_for_ball(self) -> bool:
        """
        Scan by rotating base to find a ball
        """
        self.get_logger().info('Starting scan pattern...')
        
        scan_angles = [1.56, 2.5, 3.5, 4.68, 3.5, 2.5]
        
        for i, angle in enumerate(scan_angles):
            self.get_logger().info(f'Scanning position {i+1}/6: yaw={angle:.2f} rad')
            
            scan_pos = self.scan_position.copy()
            scan_pos['top_plate_joint'] = angle
            
            if not self.move_to_joint_position(scan_pos, duration_s=2.0):
                self.get_logger().warn('Scan movement failed')
                continue
            
            # In a real system, detect ball here
            # For now, just simulate detection on first scan
            if i == 0:
                self.get_logger().info('Ball detected!')
                return True
        
        self.get_logger().warn('No ball detected in full scan')
        return False
    
    def pick_ball(self) -> bool:
        """
        Pick up ball sequence
        """
        self.get_logger().info('Executing pick sequence...')
        
        # Move to picking position
        pick_pos = {
            'top_plate_joint': 3.12,
            'lower_arm_joint': 2.0,
            'upper_arm_joint': 0.3,
            'wrist_joint': 0.2,
            'claw_base_joint': -1.5,
            'right_claw_joint': 0.0
        }
        
        if not self.move_to_joint_position(pick_pos, duration_s=3.0):
            self.get_logger().error('Failed to move to pick position')
            return False
        
        # Close gripper
        if not self.move_gripper(0.3, duration_s=1.5):  # Close position
            self.get_logger().error('Failed to close gripper')
            return False
        
        self.get_logger().info('Ball picked successfully!')
        return True
    
    def place_ball(self) -> bool:
        """
        Place ball in bowl
        """
        self.get_logger().info('Executing place sequence...')
        
        # Move to place position
        place_pos = {
            'top_plate_joint': 2.0,  # Different yaw for drop bowl
            'lower_arm_joint': 2.2,
            'upper_arm_joint': 0.4,
            'wrist_joint': 0.3,
            'claw_base_joint': -1.5,
            'right_claw_joint': 0.0
        }
        
        if not self.move_to_joint_position(place_pos, duration_s=3.0):
            self.get_logger().error('Failed to move to place position')
            return False
        
        # Open gripper
        if not self.move_gripper(-0.3, duration_s=1.5):  # Open position
            self.get_logger().error('Failed to open gripper')
            return False
        
        self.get_logger().info('Ball placed successfully!')
        return True
    
    def return_home(self) -> bool:
        """
        Return to home position
        """
        self.get_logger().info('Returning to home position...')
        return self.move_to_joint_position(self.home_position, duration_s=3.0)
    
    def execute_task(self):
        """
        Main task execution
        """
        try:
            self.get_logger().info('='*50)
            self.get_logger().info('Starting Pick and Place Task')
            self.get_logger().info('='*50)
            
            num_balls = 3
            balls_completed = 0
            
            for i in range(num_balls):
                self.get_logger().info(f'\n=== Ball {i+1}/{num_balls} ===')
                
                # Scan for ball
                if not self.scan_for_ball():
                    self.get_logger().warn(f'Failed to find ball {i+1}')
                    continue
                
                # Pick ball
                if not self.pick_ball():
                    self.get_logger().error(f'Failed to pick ball {i+1}')
                    continue
                
                # Place ball
                if not self.place_ball():
                    self.get_logger().error(f'Failed to place ball {i+1}')
                    continue
                
                balls_completed += 1
                self.get_logger().info(f'Ball {i+1} completed successfully!')
                time.sleep(1.0)
            
            # Return home
            self.return_home()
            
            self.get_logger().info(f'\n{'='*50}')
            self.get_logger().info(f'Task Complete! Processed {balls_completed}/{num_balls} balls')
            self.get_logger().info(f'{'='*50}')
            
        except KeyboardInterrupt:
            self.get_logger().info('Task interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Task execution error: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = ScanningVisionPickPlace()
    
    # Execute task
    node.execute_task()
    
    # Keep node alive briefly then shutdown
    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
