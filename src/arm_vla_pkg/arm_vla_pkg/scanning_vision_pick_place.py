#!/usr/bin/env python3
"""
AkaBot Scanning Vision Pick and Place System - FIXED
Uses correct trajectory controller publishers and working motion commands
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import cv2
import numpy as np
import threading
import time

# ROS messages
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
from rclpy.duration import Duration


class ScanningVisionPickPlace(Node):
    """
    Complete pick and place system with:
    - Real-time camera feed visualization
    - Ball scanning and detection from vision
    - Empty bowl detection
    - Real picking and placing with gripper control
    """

    def __init__(self):
        super().__init__('scanning_vision_pick_place')
        
        # Setup callback groups for multithreading
        self.reentrant_group = ReentrantCallbackGroup()
        self.sensor_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()
        
        self.get_logger().info("="*60)
        self.get_logger().info("AkaBot Scanning Vision Pick and Place System")
        self.get_logger().info("="*60)
        
        # Publishers for robot control - FIXED to use correct topic names
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/akabot_arm_controller/commands',  # FIXED: use commands, not follow_joint_trajectory
            10,
            callback_group=self.control_group
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/hand_controller/commands',  # FIXED: use commands, not follow_joint_trajectory
            10,
            callback_group=self.control_group
        )
        
        # Subscribers
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(
            Image,
            '/ee_camera/image_raw',
            self.camera_callback,
            10,
            callback_group=self.sensor_group
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.sensor_group
        )
        
        # State tracking
        self.current_image = None
        self.current_joint_state = {}
        self.ball_detected_pos = None
        self.bowl_detected_pos = None
        self.detection_ready = False
        
        # Joint configuration
        self.arm_joints = [
            'top_plate_joint',
            'lower_arm_joint',
            'upper_arm_joint',
            'wrist_joint',
            'claw_base_joint'
        ]
        self.gripper_joint = 'right_claw_joint'
        
        # Home position - FIXED to match actual URDF joint limits
        self.home_position = {
            'top_plate_joint': 3.12,    # Within [1.56, 4.68]
            'lower_arm_joint': 0.5,     # Within [0.0, 3.12]
            'upper_arm_joint': 0.0,     # Within [-1.7, 1.7]
            'wrist_joint': 0.0,         # Within [-1.7, 1.7]
            'claw_base_joint': 0.0,     # Within [-3.17, 0.0]
            'right_claw_joint': 0.0     # Within [-0.5, 0.0]
        }
        
        # Scanning yaw angles for 360 coverage
        self.scan_yaw_angles = [1.56, 2.2, 2.8, 3.5, 4.1, 4.7]
        
        # Vision detection parameters
        self.ball_color_lower_hsv = np.array([0, 100, 100])
        self.ball_color_upper_hsv = np.array([10, 255, 255])
        self.bowl_color_lower_hsv = np.array([10, 50, 50])
        self.bowl_color_upper_hsv = np.array([30, 255, 200])
        
        self.get_logger().info("System initialized successfully")
        self.get_logger().info("Waiting for camera feed and joint states...")
        time.sleep(2)
        self.detection_ready = True
        self.get_logger().info("READY! Camera and joints initialized")
        
    def camera_callback(self, msg):
        """Receive and display camera feed with overlays"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_image = self.current_image.copy()
            
            # Draw detected ball if found
            if self.ball_detected_pos is not None:
                x, y = int(self.ball_detected_pos[0]), int(self.ball_detected_pos[1])
                cv2.circle(display_image, (x, y), 15, (0, 255, 0), 3)
                cv2.putText(display_image, f'Ball ({x},{y})', (x-60, y-25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw detected bowl if found
            if self.bowl_detected_pos is not None:
                x, y = int(self.bowl_detected_pos[0]), int(self.bowl_detected_pos[1])
                cv2.circle(display_image, (x, y), 25, (255, 165, 0), 3)
                cv2.putText(display_image, f'Bowl ({x},{y})', (x-70, y-35),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)
            
            # Show resolution and detection status
            h, w = display_image.shape[:2]
            cv2.putText(display_image, f'Resolution: {w}x{h}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(display_image, 'AkaBot Camera Feed', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            cv2.imshow('AkaBot Camera Feed - Real-Time Detection', display_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")
    
    def joint_state_callback(self, msg):
        """Track current joint positions"""
        for name, position in zip(msg.name, msg.position):
            self.current_joint_state[name] = position
    
    def detect_balls(self, image):
        """Detect red balls using HSV color segmentation"""
        if image is None:
            return []
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Red in HSV at both ends
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        balls = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100 or area > 5000:
                continue
            
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                balls.append({
                    'position': (cx, cy),
                    'area': area,
                    'contour': contour
                })
        
        return balls
    
    def detect_bowls(self, image):
        """Detect brown/yellow bowls"""
        if image is None:
            return []
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.bowl_color_lower_hsv, self.bowl_color_upper_hsv)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        bowls = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 200 or area > 10000:
                continue
            
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                bowls.append({
                    'position': (cx, cy),
                    'area': area,
                    'contour': contour
                })
        
        return bowls
    
    def move_arm(self, joint_positions, duration=3.0):
        """Publish arm trajectory to move to position - FIXED"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = [joint_positions.get(name, self.home_position[name])
                          for name in self.arm_joints]
        # FIXED: Use Duration instead of manual sec/nanosec
        point.time_from_start = Duration(seconds=duration).to_msg()
        
        trajectory.points = [point]
        self.arm_pub.publish(trajectory)
        
        self.get_logger().info(f"Arm moving for {duration}s...")
        time.sleep(duration + 0.5)
    
    def move_gripper(self, position, duration=1.0):
        """Control gripper - FIXED"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [self.gripper_joint]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        # FIXED: Use Duration instead of manual sec/nanosec
        point.time_from_start = Duration(seconds=duration).to_msg()
        
        trajectory.points = [point]
        self.gripper_pub.publish(trajectory)
        
        status = "CLOSING" if position < 0 else "OPENING"
        self.get_logger().info(f"Gripper {status}")
        time.sleep(duration + 0.2)
    
    def scan_for_ball(self):
        """Scan by rotating base to find ball"""
        self.get_logger().info(f"\nSCAN PATTERN: Looking for ball...")
        
        for i, yaw in enumerate(self.scan_yaw_angles):
            self.get_logger().info(f"  Scan {i+1}/{len(self.scan_yaw_angles)}: yaw={yaw:.2f}rad")
            
            scan_pos = self.home_position.copy()
            scan_pos['top_plate_joint'] = yaw
            self.move_arm(scan_pos, duration=1.5)
            
            time.sleep(0.3)
            
            if self.current_image is not None:
                balls = self.detect_balls(self.current_image)
                if balls:
                    ball = max(balls, key=lambda x: x['area'])
                    self.ball_detected_pos = ball['position']
                    self.get_logger().info(f"  ✓ Ball DETECTED at {self.ball_detected_pos}")
                    return True, yaw
        
        self.get_logger().warning("  ✗ No ball found")
        return False, None
    
    def scan_for_bowl(self):
        """Scan to find empty bowl"""
        self.get_logger().info(f"\nSCAN PATTERN: Looking for bowl...")
        
        for i, yaw in enumerate(self.scan_yaw_angles):
            self.get_logger().info(f"  Scan {i+1}/{len(self.scan_yaw_angles)}: yaw={yaw:.2f}rad")
            
            scan_pos = self.home_position.copy()
            scan_pos['top_plate_joint'] = yaw
            self.move_arm(scan_pos, duration=1.5)
            
            time.sleep(0.3)
            
            if self.current_image is not None:
                bowls = self.detect_bowls(self.current_image)
                if bowls:
                    bowl = max(bowls, key=lambda x: x['area'])
                    self.bowl_detected_pos = bowl['position']
                    self.get_logger().info(f"  ✓ Bowl DETECTED at {self.bowl_detected_pos}")
                    return True, yaw
        
        self.get_logger().warning("  ✗ No bowl found")
        return False, None
    
    def pick_ball(self, yaw):
        """Execute pick sequence - FIXED joint limits"""
        self.get_logger().info("PICK SEQUENCE:")
        
        # Move to picking position - FIXED to respect joint limits
        pick_pos = {
            'top_plate_joint': yaw,     # [1.56, 4.68]
            'lower_arm_joint': 1.5,     # [0.0, 3.12]
            'upper_arm_joint': 0.0,     # [-1.7, 1.7]
            'wrist_joint': 0.0,         # [-1.7, 1.7]
            'claw_base_joint': -1.5     # [-3.17, 0.0]
        }
        self.get_logger().info("  Moving to ball...")
        self.move_arm(pick_pos, duration=2.0)
        
        # Close gripper
        self.get_logger().info("  Closing gripper...")
        self.move_gripper(-0.4, duration=1.0)  # Negative = close
        
        # Lift
        lift_pos = pick_pos.copy()
        lift_pos['upper_arm_joint'] = 1.0  # Lift arm up
        self.get_logger().info("  Lifting ball...")
        self.move_arm(lift_pos, duration=2.0)
        
        self.get_logger().info("  ✓ BALL PICKED")
        return True
    
    def place_ball(self, yaw):
        """Execute place sequence - FIXED"""
        self.get_logger().info("PLACE SEQUENCE:")
        
        # Move to place position
        place_pos = {
            'top_plate_joint': yaw,     # [1.56, 4.68]
            'lower_arm_joint': 1.5,     # [0.0, 3.12]
            'upper_arm_joint': 0.0,     # [-1.7, 1.7]
            'wrist_joint': 0.0,         # [-1.7, 1.7]
            'claw_base_joint': -1.5     # [-3.17, 0.0]
        }
        self.get_logger().info("  Moving to bowl...")
        self.move_arm(place_pos, duration=2.0)
        
        # Open gripper
        self.get_logger().info("  Opening gripper...")
        self.move_gripper(0.3, duration=1.0)  # Positive = open
        
        # Retract
        retract_pos = place_pos.copy()
        retract_pos['upper_arm_joint'] = 1.0
        self.get_logger().info("  Retracting...")
        self.move_arm(retract_pos, duration=2.0)
        
        self.get_logger().info("  ✓ BALL PLACED")
        return True
    
    def execute_task(self):
        """Execute full automated task"""
        self.get_logger().info("\n" + "#"*60)
        self.get_logger().info("# STARTING AUTOMATED TASK")
        self.get_logger().info("#"*60)
        
        num_balls = 3
        success_count = 0
        
        for ball_num in range(1, num_balls + 1):
            self.get_logger().info(f"\n{"-"*60}")
            self.get_logger().info(f"BALL {ball_num}/{num_balls}")
            self.get_logger().info(f"{"-"*60}")
            
            # Scan and detect ball
            found, yaw = self.scan_for_ball()
            if not found:
                self.get_logger().warning(f"Skipping ball {ball_num}")
                continue
            
            # Pick ball
            if not self.pick_ball(yaw):
                self.get_logger().warning(f"Pick failed for ball {ball_num}")
                continue
            
            # Scan and detect bowl
            found, yaw = self.scan_for_bowl()
            if not found:
                self.get_logger().warning(f"No bowl found for ball {ball_num}")
                continue
            
            # Place ball
            if not self.place_ball(yaw):
                self.get_logger().warning(f"Place failed for ball {ball_num}")
                continue
            
            success_count += 1
            self.get_logger().info(f"✓ BALL {ball_num} COMPLETE")
            time.sleep(1)
        
        # Return home
        self.get_logger().info(f"\nReturning to HOME...")
        self.move_arm(self.home_position, duration=2.0)
        
        self.get_logger().info("\n" + "#"*60)
        self.get_logger().info(f"# TASK COMPLETE: {success_count}/{num_balls} balls")
        self.get_logger().info("#"*60)


def main(args=None):
    rclpy.init(args=args)
    
    node = ScanningVisionPickPlace()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    # Run task in separate thread
    task_thread = threading.Thread(target=node.execute_task, daemon=True)
    task_thread.start()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
