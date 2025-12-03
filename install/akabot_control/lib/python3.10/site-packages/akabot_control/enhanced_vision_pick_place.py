#!/usr/bin/env python3
"""
Enhanced Vision-based Pick and Place for Akabot
Complete system with robust detection, transforms, and motion planning
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from akabot_control.akabot_controller import AkabotController
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time
from enum import Enum
from collections import deque


class State(Enum):
    """State machine states"""
    INIT = 0
    MOVE_TO_SCAN = 1
    SCANNING = 2
    BALL_DETECTED = 3
    APPROACH_BALL = 4
    PICK_BALL = 5
    LIFT_BALL = 6
    MOVE_TO_DROP = 7
    DROP_BALL = 8
    RETURN_HOME = 9
    DONE = 10
    ERROR = 11


class BallDetector:
    """Enhanced ball detection with filtering and tracking"""
    
    def __init__(self, logger):
        self.logger = logger
        self.detection_history = deque(maxlen=5)
        
        # HSV color ranges for white thermocol balls
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 50, 255])
        
        # Detection parameters
        self.min_area = 200
        self.max_area = 8000
        self.min_circularity = 0.7
        self.min_radius = 8
        
    def detect_balls(self, image):
        """
        Detect white balls in image with filtering
        Returns list of ball detections with confidence scores
        """
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for white objects
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        
        # Morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area
            if self.min_area < area < self.max_area:
                # Fit circle
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                
                if radius > self.min_radius:
                    # Check circularity
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter > 0:
                        circularity = 4 * np.pi * area / (perimeter * perimeter)
                        
                        if circularity > self.min_circularity:
                            # Calculate confidence based on circularity and area
                            confidence = circularity * min(area / 1000.0, 1.0)
                            
                            detections.append({
                                'center': (int(x), int(y)),
                                'radius': radius,
                                'area': area,
                                'circularity': circularity,
                                'confidence': confidence
                            })
        
        # Sort by confidence
        detections.sort(key=lambda d: d['confidence'], reverse=True)
        
        return detections, mask
    
    def get_best_detection(self, detections):
        """Get the most confident detection with temporal filtering"""
        if not detections:
            return None
        
        # Add to history
        self.detection_history.append(detections[0] if detections else None)
        
        # Check if we have consistent detections
        valid_detections = [d for d in self.detection_history if d is not None]
        
        if len(valid_detections) >= 3:
            # Average the positions for stability
            avg_x = np.mean([d['center'][0] for d in valid_detections])
            avg_y = np.mean([d['center'][1] for d in valid_detections])
            avg_radius = np.mean([d['radius'] for d in valid_detections])
            
            return {
                'center': (int(avg_x), int(avg_y)),
                'radius': avg_radius,
                'confidence': detections[0]['confidence']
            }
        
        return None
    
    def clear_history(self):
        """Clear detection history"""
        self.detection_history.clear()


class EnhancedVisionPickPlace(Node):
    """Enhanced vision-based pick and place with robust error handling"""
    
    def __init__(self):
        super().__init__('enhanced_vision_pick_place')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Ball detector
        self.ball_detector = BallDetector(self.get_logger())
        
        # Camera subscribers
        self.image_sub = self.create_subscription(
            Image, '/ee_camera/image_raw', self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/ee_camera/camera_info', self.camera_info_callback, 10
        )
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None
        
        # Robot controller
        self.controller = AkabotController()
        
        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd')
        
        # State machine
        self.state = State.INIT
        self.balls_transferred = 0
        self.target_balls = 3
        self.max_retries = 3
        self.retry_count = 0
        
        # Current detection
        self.current_ball_pixel = None
        self.current_ball_3d = None
        
        # Positions (in base_link frame)
        self.scan_position = {
            'x': 0.15, 'y': 0.0, 'z': 0.25,
            'roll': 0.0, 'pitch': np.pi/2, 'yaw': 0.0
        }
        
        self.source_bowl_center = np.array([0.15, 0.10, 1.03])
        self.target_bowl_center = np.array([0.15, -0.10, 1.03])
        
        # Motion parameters
        self.pick_offset_z = 0.01
        self.approach_offset_z = 0.08
        self.place_offset_z = 0.03
        self.lift_height = 0.12
        
        # Table height for depth estimation
        self.table_height = 1.03
        
        # Visualization
        self.show_debug = True
        
        self.get_logger().info('Enhanced Vision Pick and Place initialized')
        
    def camera_info_callback(self, msg):
        """Store camera calibration"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration received')
    
    def image_callback(self, msg):
        """Process camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image.copy()
            
            # Detect balls during scanning
            if self.state in [State.SCANNING, State.MOVE_TO_SCAN]:
                self.process_image(cv_image)
                
        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')
    
    def process_image(self, image):
        """Process image and detect balls"""
        detections, mask = self.ball_detector.detect_balls(image)
        best_ball = self.ball_detector.get_best_detection(detections)
        
        if best_ball:
            self.current_ball_pixel = best_ball['center']
            self.get_logger().info(
                f"Ball detected at pixel {self.current_ball_pixel} "
                f"with confidence {best_ball['confidence']:.2f}"
            )
        
        # Visualization
        if self.show_debug:
            self.visualize_detections(image, detections, best_ball, mask)
    
    def visualize_detections(self, image, detections, best_ball, mask):
        """Visualize detections for debugging"""
        debug_img = image.copy()
        
        # Draw all detections
        for det in detections:
            x, y = det['center']
            r = int(det['radius'])
            color = (0, 255, 255)  # Yellow for all detections
            cv2.circle(debug_img, (x, y), r, color, 2)
            cv2.putText(
                debug_img, f"{det['confidence']:.2f}",
                (x - 30, y - r - 5), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 2
            )
        
        # Draw best detection
        if best_ball:
            x, y = best_ball['center']
            r = int(best_ball['radius'])
            cv2.circle(debug_img, (x, y), r, (0, 255, 0), 3)  # Green
            cv2.circle(debug_img, (x, y), 3, (0, 0, 255), -1)  # Red center
            cv2.putText(
                debug_img, "TARGET",
                (x - 30, y + r + 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2
            )
        
        # State info
        state_text = f'State: {self.state.name} | Balls: {self.balls_transferred}/{self.target_balls}'
        cv2.putText(debug_img, state_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Ball Detection', debug_img)
        cv2.imshow('White Mask', mask)
        cv2.waitKey(1)
    
    def estimate_ball_3d_position(self):
        """Estimate 3D position of detected ball using camera transforms"""
        if self.current_ball_pixel is None or self.camera_matrix is None:
            return None
        
        try:
            # Get camera to base_link transform
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'ee_camera_optical_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Camera position
            cam_z = transform.transform.translation.z
            
            # Estimate depth to table
            depth = abs(cam_z - (self.table_height + 0.015))  # Ball radius
            
            # Unproject pixel to 3D
            px, py = self.current_ball_pixel
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            
            # 3D point in camera frame
            x_cam = (px - cx) * depth / fx
            y_cam = (py - cy) * depth / fy
            z_cam = depth
            
            # Create pose in camera frame
            point_cam = PoseStamped()
            point_cam.header.frame_id = 'ee_camera_optical_link'
            point_cam.pose.position.x = float(x_cam)
            point_cam.pose.position.y = float(y_cam)
            point_cam.pose.position.z = float(z_cam)
            point_cam.pose.orientation.w = 1.0
            
            # Transform to base_link
            point_base = tf2_geometry_msgs.do_transform_pose(point_cam, transform)
            
            ball_3d = np.array([
                point_base.pose.position.x,
                point_base.pose.position.y,
                self.table_height + 0.015  # Fixed height at ball center
            ])
            
            self.get_logger().info(
                f'Ball 3D position: x={ball_3d[0]:.3f}, y={ball_3d[1]:.3f}, z={ball_3d[2]:.3f}'
            )
            
            return ball_3d
            
        except Exception as e:
            self.get_logger().error(f'Failed to estimate 3D position: {str(e)}')
            return None
    
    def move_to_scan_position(self):
        """Move to scanning position above source bowl"""
        self.get_logger().info('Moving to scan position...')
        
        scan_pose = self.controller.create_pose(**self.scan_position)
        success = self.controller.move_to_pose(scan_pose, duration=4.0)
        
        if success:
            time.sleep(1.0)  # Wait for image to stabilize
            self.ball_detector.clear_history()
        
        return success
    
    def open_gripper(self):
        """Open the gripper"""
        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        goal.command.max_effort = 10.0
        
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.0)
        self.get_logger().info('Gripper opened')
    
    def close_gripper(self):
        """Close the gripper"""
        goal = GripperCommand.Goal()
        goal.command.position = -0.4
        goal.command.max_effort = 10.0
        
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.0)
        self.get_logger().info('Gripper closed')
    
    def pick_sequence(self, ball_pos):
        """Execute pick sequence"""
        self.get_logger().info(f'Executing pick sequence at {ball_pos}')
        
        # 1. Approach from above
        approach_pose = self.controller.create_pose(
            x=ball_pos[0], y=ball_pos[1],
            z=ball_pos[2] + self.approach_offset_z,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        if not self.controller.move_to_pose(approach_pose, duration=3.0):
            return False
        time.sleep(0.5)
        
        # 2. Move down to pick
        pick_pose = self.controller.create_pose(
            x=ball_pos[0], y=ball_pos[1],
            z=ball_pos[2] + self.pick_offset_z,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        if not self.controller.move_to_pose(pick_pose, duration=2.0):
            return False
        time.sleep(0.5)
        
        # 3. Close gripper
        self.close_gripper()
        
        # 4. Lift
        lift_pose = self.controller.create_pose(
            x=ball_pos[0], y=ball_pos[1],
            z=ball_pos[2] + self.lift_height,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        return self.controller.move_to_pose(lift_pose, duration=2.0)
    
    def place_sequence(self, place_pos):
        """Execute place sequence"""
        self.get_logger().info(f'Executing place sequence at {place_pos}')
        
        # 1. Approach target bowl
        approach_pose = self.controller.create_pose(
            x=place_pos[0], y=place_pos[1],
            z=place_pos[2] + self.approach_offset_z,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        if not self.controller.move_to_pose(approach_pose, duration=3.0):
            return False
        time.sleep(0.5)
        
        # 2. Move down to place
        place_pose = self.controller.create_pose(
            x=place_pos[0], y=place_pos[1],
            z=place_pos[2] + self.place_offset_z,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        if not self.controller.move_to_pose(place_pose, duration=2.0):
            return False
        time.sleep(0.5)
        
        # 3. Open gripper
        self.open_gripper()
        
        # 4. Move up
        lift_pose = self.controller.create_pose(
            x=place_pos[0], y=place_pos[1],
            z=place_pos[2] + self.lift_height,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        return self.controller.move_to_pose(lift_pose, duration=2.0)
    
    def run_state_machine(self):
        """Main state machine loop"""
        rate = self.create_rate(10)
        
        while rclpy.ok() and self.state != State.DONE:
            
            if self.state == State.INIT:
                self.get_logger().info('=== Starting Enhanced Pick and Place ===')
                self.open_gripper()
                self.state = State.MOVE_TO_SCAN
                
            elif self.state == State.MOVE_TO_SCAN:
                if self.move_to_scan_position():
                    self.state = State.SCANNING
                else:
                    self.get_logger().error('Failed to move to scan position')
                    self.state = State.ERROR
                    
            elif self.state == State.SCANNING:
                # Wait for stable detection
                time.sleep(2.0)
                
                if self.current_ball_pixel is not None:
                    self.get_logger().info('Ball detected! Estimating 3D position...')
                    self.state = State.BALL_DETECTED
                else:
                    self.get_logger().warn('No ball detected')
                    if self.retry_count < self.max_retries:
                        self.retry_count += 1
                        self.state = State.MOVE_TO_SCAN
                    else:
                        self.get_logger().error('Max retries reached')
                        self.state = State.DONE
                        
            elif self.state == State.BALL_DETECTED:
                ball_3d = self.estimate_ball_3d_position()
                
                if ball_3d is not None:
                    self.current_ball_3d = ball_3d
                    self.state = State.APPROACH_BALL
                else:
                    self.get_logger().error('Failed to estimate position')
                    self.state = State.MOVE_TO_SCAN
                    
            elif self.state == State.APPROACH_BALL:
                self.state = State.PICK_BALL
                
            elif self.state == State.PICK_BALL:
                if self.pick_sequence(self.current_ball_3d):
                    self.get_logger().info('Pick successful!')
                    self.state = State.MOVE_TO_DROP
                else:
                    self.get_logger().error('Pick failed')
                    if self.retry_count < self.max_retries:
                        self.retry_count += 1
                        self.open_gripper()
                        self.state = State.MOVE_TO_SCAN
                    else:
                        self.state = State.ERROR
                        
            elif self.state == State.MOVE_TO_DROP:
                self.state = State.DROP_BALL
                
            elif self.state == State.DROP_BALL:
                place_pos = self.target_bowl_center.copy()
                place_pos[2] += 0.05
                
                if self.place_sequence(place_pos):
                    self.balls_transferred += 1
                    self.retry_count = 0
                    self.get_logger().info(
                        f'Ball placed! Progress: {self.balls_transferred}/{self.target_balls}'
                    )
                    
                    if self.balls_transferred >= self.target_balls:
                        self.state = State.RETURN_HOME
                    else:
                        self.current_ball_pixel = None
                        self.ball_detector.clear_history()
                        self.state = State.MOVE_TO_SCAN
                else:
                    self.get_logger().error('Place failed')
                    self.state = State.ERROR
                    
            elif self.state == State.RETURN_HOME:
                self.get_logger().info('Returning to home position...')
                self.controller.move_to_named_target('home')
                self.state = State.DONE
                
            elif self.state == State.ERROR:
                self.get_logger().error('Error state reached. Returning home.')
                self.controller.move_to_named_target('home')
                self.state = State.DONE
            
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()
        
        self.get_logger().info('=== Mission Complete! ===')
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    node = EnhancedVisionPickPlace()
    
    # Wait for initialization
    time.sleep(2.0)
    
    # Wait for camera and controller ready
    while not node.controller.joint_state_received or node.camera_matrix is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)
    
    node.get_logger().info('System ready! Starting pick and place...')
    
    try:
        node.run_state_machine()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()