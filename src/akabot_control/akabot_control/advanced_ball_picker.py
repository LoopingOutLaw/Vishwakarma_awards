#!/usr/bin/env python3
"""
Advanced Ball Picker with Trajectory Planning and End-Effector Camera Precision

Features:
- 5-DOF inverse kinematics
- Trajectory planning for smooth arm motion
- End-effector camera feedback for precision picking
- Three-ball sequential pickup with place in bowl
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Point

from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
from enum import Enum

from trajectory_planner import TrajectoryPlanner


class PickerState(Enum):
    """State machine for picking operations"""
    IDLE = 0
    MOVING_TO_HOVER = 1
    MOVING_TO_APPROACH = 2
    CAMERA_PRECISION = 3
    MOVING_TO_GRASP = 4
    GRASPING = 5
    MOVING_TO_PLACE = 6
    PLACING = 7
    RETURNING_HOME = 8
    COMPLETED = 9


class AdvancedBallPicker(Node):
    """Main controller for ball picking with trajectory planning"""
    
    def __init__(self):
        super().__init__('advanced_ball_picker')
        
        self.get_logger().info("Initializing Advanced Ball Picker...")
        
        # Trajectory planner
        self.planner = TrajectoryPlanner()
        
        # State machine
        self.state = PickerState.IDLE
        self.current_ball_index = 0
        self.balls_picked = 0
        
        # Target positions (from Gazebo world)
        self.ball_positions = [
            (0.15, 0.10, 0.065),   # Ball 0
            (0.12, 0.15, 0.065),   # Ball 1
            (0.13, 0.10, 0.065),   # Ball 2
        ]
        
        self.dest_bowl = (0.20, 0.00, 0.065)  # Empty bowl destination
        self.home_position = (0.15, -0.10, 0.35)  # Home/safe position
        
        # Joint trajectory action client
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/akabot_arm_controller/follow_joint_trajectory'
        )
        
        # Publishers
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/akabot_hand_controller/commands',
            10
        )
        
        # Subscribers
        self.ee_camera_sub = self.create_subscription(
            Image,
            '/ee_camera/image_raw',
            self.ee_camera_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Camera bridge
        self.bridge = CvBridge()
        self.latest_ee_frame = None
        self.current_joint_state = None
        
        # Camera parameters
        self.BALL_COLOR_LOWER = np.array([100, 100, 200])  # Blue-ish white
        self.BALL_COLOR_UPPER = np.array([255, 255, 255])  # White
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480
        self.CENTER_TOLERANCE = 30  # pixels
        
        # Timing
        self.motion_timeout = 10.0  # seconds
        self.camera_precision_timeout = 5.0
        
        self.get_logger().info("Advanced Ball Picker initialized!")
    
    def ee_camera_callback(self, msg: Image):
        """Receive end-effector camera frames"""
        try:
            self.latest_ee_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")
    
    def joint_state_callback(self, msg: JointState):
        """Receive joint state"""
        self.current_joint_state = msg
    
    def detect_ball_in_frame(self) -> Tuple[bool, float, float, float]:
        """
        Detect ball in end-effector camera frame
        Returns (found, center_x, center_y, area)
        """
        if self.latest_ee_frame is None:
            return (False, 0, 0, 0)
        
        frame = self.latest_ee_frame.copy()
        
        # Create mask for ball color
        mask = cv2.inRange(frame, self.BALL_COLOR_LOWER, self.BALL_COLOR_UPPER)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return (False, 0, 0, 0)
        
        # Get largest contour (should be the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Get centroid
        M = cv2.moments(largest_contour)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (True, cx, cy, area)
        
        return (False, 0, 0, area)
    
    def camera_precision_adjustment(self, target_x: float, target_y: float, target_z: float,
                                   timeout: float = 5.0) -> Tuple[float, float, float]:
        """
        Use end-effector camera to refine approach to ball
        Adjusts XY position based on camera feedback
        
        Returns refined (x, y, z) position
        """
        self.get_logger().info("Starting camera precision adjustment...")
        start_time = time.time()
        max_adjustments = 5
        adjustment_count = 0
        
        current_x, current_y = target_x, target_y
        
        while adjustment_count < max_adjustments and (time.time() - start_time) < timeout:
            # Get frame and detect ball
            found, cx, cy, area = self.detect_ball_in_frame()
            
            if not found:
                self.get_logger().warn("Ball not visible in camera frame, using default position")
                break
            
            # Calculate offset from center
            offset_x = cx - (self.FRAME_WIDTH / 2)
            offset_y = cy - (self.FRAME_HEIGHT / 2)
            
            # Check if centered
            if abs(offset_x) < self.CENTER_TOLERANCE and abs(offset_y) < self.CENTER_TOLERANCE:
                self.get_logger().info(f"Ball centered in camera (offset: {offset_x:.1f}, {offset_y:.1f})")
                break
            
            # Convert pixel offset to world adjustment
            # Calibration factor (adjust based on your setup)
            pixel_to_meter = 0.0003  # meters per pixel
            
            adjust_x = -offset_x * pixel_to_meter
            adjust_y = offset_y * pixel_to_meter
            
            # Update target position
            current_x += adjust_x
            current_y += adjust_y
            
            # Check if new position is reachable
            test_ik = self.planner.inverse_kinematics(current_x, current_y, target_z)
            if test_ik:
                # Move to adjusted position
                self.execute_trajectory([test_ik])
                self.get_logger().info(f"Adjustment {adjustment_count + 1}: offset({offset_x:.1f}, {offset_y:.1f}) -> position({current_x:.3f}, {current_y:.3f})")
                adjustment_count += 1
                time.sleep(0.5)
            else:
                self.get_logger().warn("Adjusted position unreachable")
                break
        
        self.get_logger().info(f"Camera precision complete after {adjustment_count} adjustments")
        return (current_x, current_y, target_z)
    
    def execute_trajectory(self, waypoints: list) -> bool:
        """
        Execute trajectory using MoveIt
        
        Args:
            waypoints: List of joint configurations
            
        Returns:
            True if successful, False otherwise
        """
        if not waypoints:
            return False
        
        # Smooth the trajectory
        smooth_waypoints = self.planner.smooth_trajectory(waypoints, step_size=3)
        
        # Build trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'top_plate_joint',
            'lower_arm_joint',
            'upper_arm_joint',
            'wrist_joint',
            'claw_base_joint'
        ]
        
        # Add trajectory points
        for i, waypoint in enumerate(smooth_waypoints):
            point = JointTrajectoryPoint()
            point.positions = [
                waypoint['top_plate_joint'],
                waypoint['lower_arm_joint'],
                waypoint['upper_arm_joint'],
                waypoint['wrist_joint'],
                waypoint['claw_base_joint']
            ]
            point.velocities = [0.0] * 5
            point.time_from_start.sec = int((i + 1) * 0.5)  # 0.5 seconds per waypoint
            point.time_from_start.nanosec = 0
            trajectory.points.append(point)
        
        try:
            # Wait for action server
            if not self.trajectory_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("Trajectory action server not available")
                return False
            
            # Send trajectory goal
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            goal.goal_time_tolerance.sec = 5
            
            future = self.trajectory_client.send_goal_async(goal)
            
            # Wait for result
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < self.motion_timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            return future.done()
            
        except Exception as e:
            self.get_logger().error(f"Trajectory execution error: {e}")
            return False
    
    def set_gripper(self, open_angle: float = 0.0) -> bool:
        """
        Control gripper opening/closing
        
        Args:
            open_angle: 0.0 = closed, 0.5 = open (in radians)
            
        Returns:
            True if successful
        """
        try:
            msg = Float64MultiArray()
            msg.data = [open_angle, open_angle * -1]  # Mirror for dual gripper
            self.gripper_pub.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Gripper control error: {e}")
            return False
    
    def pick_single_ball(self, ball_index: int) -> bool:
        """
        Pick a single ball and place in destination bowl
        
        Returns:
            True if successful, False otherwise
        """
        if ball_index >= len(self.ball_positions):
            return False
        
        ball_pos = self.ball_positions[ball_index]
        self.get_logger().info(f"\n=== Picking Ball {ball_index} ===")
        self.get_logger().info(f"Target position: {ball_pos}")
        
        # Step 1: Plan approach trajectory
        self.get_logger().info("Step 1: Planning approach trajectory...")
        approach_traj = self.planner.plan_approach_trajectory(*ball_pos, approach_distance=0.05)
        if not approach_traj:
            self.get_logger().error("Failed to plan approach trajectory")
            return False
        
        # Step 2: Execute approach
        self.get_logger().info("Step 2: Moving to hover position...")
        if not self.execute_trajectory([approach_traj[0]]):
            self.get_logger().error("Failed to move to hover")
            return False
        
        self.get_logger().info("Step 3: Moving to approach position...")
        if not self.execute_trajectory([approach_traj[1]]):
            self.get_logger().error("Failed to move to approach")
            return False
        
        # Step 4: Camera precision adjustment
        self.get_logger().info("Step 4: Refining position with end-effector camera...")
        refined_pos = self.camera_precision_adjustment(ball_pos[0], ball_pos[1], ball_pos[2])
        
        # Step 5: Move to final grasp position
        self.get_logger().info("Step 5: Moving to grasp position...")
        grasp_ik = self.planner.inverse_kinematics(refined_pos[0], refined_pos[1], refined_pos[2] - 0.03)
        if grasp_ik:
            if not self.execute_trajectory([grasp_ik]):
                self.get_logger().error("Failed to move to grasp")
                return False
        else:
            self.get_logger().error("Grasp position unreachable")
            return False
        
        # Step 6: Close gripper
        self.get_logger().info("Step 6: Closing gripper...")
        self.set_gripper(open_angle=0.0)  # Close
        time.sleep(1.5)  # Wait for gripper to close
        
        # Step 7: Lift ball
        self.get_logger().info("Step 7: Lifting ball...")
        lift_ik = self.planner.inverse_kinematics(refined_pos[0], refined_pos[1], refined_pos[2] + 0.10)
        if lift_ik:
            if not self.execute_trajectory([lift_ik]):
                self.get_logger().error("Failed to lift ball")
                return False
        
        # Step 8: Move to destination bowl
        self.get_logger().info("Step 8: Moving to destination bowl...")
        place_traj = self.planner.plan_place_trajectory(*self.dest_bowl)
        if not place_traj:
            self.get_logger().error("Failed to plan place trajectory")
            return False
        
        for waypoint_config in place_traj:
            if not self.execute_trajectory([waypoint_config]):
                self.get_logger().error("Failed to move to place position")
                return False
        
        # Step 9: Open gripper
        self.get_logger().info("Step 9: Opening gripper to release ball...")
        self.set_gripper(open_angle=0.5)  # Open
        time.sleep(1.0)
        
        # Step 10: Return to home
        self.get_logger().info("Step 10: Returning to home...")
        return_traj = self.planner.plan_return_trajectory()
        if not self.execute_trajectory(return_traj):
            self.get_logger().error("Failed to return home")
            return False
        
        self.get_logger().info(f"Ball {ball_index} picked and placed successfully! ✓")
        return True
    
    def run_sequence(self):
        """
        Main sequence: Pick all 3 balls
        """
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("Starting 3-Ball Pickup Sequence")
        self.get_logger().info("="*50)
        
        time.sleep(2)  # Wait for all nodes to initialize
        
        for ball_idx in range(len(self.ball_positions)):
            success = self.pick_single_ball(ball_idx)
            
            if success:
                self.balls_picked += 1
                self.get_logger().info(f"Status: {self.balls_picked}/{len(self.ball_positions)} balls picked")
            else:
                self.get_logger().error(f"Failed to pick ball {ball_idx}")
            
            time.sleep(1)  # Pause between balls
        
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info(f"Sequence Complete! Picked {self.balls_picked}/3 balls ✓")
        self.get_logger().info("="*50)


def main(args=None):
    rclpy.init(args=args)
    picker = AdvancedBallPicker()
    
    # Run sequence in background thread
    import threading
    picker_thread = threading.Thread(target=picker.run_sequence, daemon=True)
    picker_thread.start()
    
    # Spin node
    try:
        rclpy.spin(picker)
    except KeyboardInterrupt:
        picker.get_logger().info("Shutting down...")
    finally:
        picker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
