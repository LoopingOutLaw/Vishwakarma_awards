#!/usr/bin/env python3
"""
Vision-based Pick and Place with Scanning
The robot scans the workspace by rotating, detects balls, and picks them up
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


class State(Enum):
    """State machine states"""
    INIT = 0
    SCANNING = 1
    BALL_DETECTED = 2
    MOVING_TO_PICK = 3
    PICKING = 4
    MOVING_TO_PLACE = 5
    PLACING = 6
    DONE = 7


class ScanningVisionPickPlace(Node):
    """Vision-based pick and place with scanning capability"""
    
    def __init__(self):
        super().__init__('scanning_vision_pick_place')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to end effector camera
        self.image_sub = self.create_subscription(
            Image,
            '/ee_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/ee_camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None
        
        # Akabot controller
        self.controller = AkabotController()
        
        # Gripper action client
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/hand_controller/gripper_cmd'
        )
        
        # State machine
        self.state = State.INIT
        self.balls_picked = 0
        self.target_balls = 3
        
        # Scanning parameters
        self.scan_positions = [
            3.12,   # Center
            2.50,   # Right
            3.70,   # Left
            2.00,   # Far right
            4.20,   # Far left
        ]
        self.current_scan_index = 0
        self.scanning_height = 0.20  # Height to scan from
        
        # Detection parameters
        self.detected_ball_pixel = None
        self.detected_ball_3d = None
        self.ball_detection_threshold = 5  # Frames to confirm detection
        self.ball_detection_count = 0
        
        # Pick and place positions
        self.pick_height = 0.05  # Height to pick balls (relative to table)
        self.place_height = 0.05  # Height to place balls
        self.approach_offset = 0.08  # Approach from above
        
        # Bowl positions (approximate, will be refined by vision)
        self.source_bowl_center = np.array([0.15, 0.10, 1.03])
        self.target_bowl_center = np.array([0.15, -0.10, 1.03])
        
        # Visualization
        self.show_debug = True
        
        self.get_logger().info('Scanning Vision Pick and Place initialized')
        
    def camera_info_callback(self, msg):
        """Store camera calibration parameters"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera parameters received')
    
    def image_callback(self, msg):
        """Process camera images to detect balls"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image.copy()
            
            # Only detect during scanning or when looking for balls
            if self.state in [State.SCANNING, State.BALL_DETECTED]:
                self.detect_and_track_ball(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
    
    def detect_and_track_ball(self, image):
        """Detect white balls and track the best candidate"""
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # White color range (adjusted for better detection)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        
        # Create mask
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_ball = None
        max_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area
            if 200 < area < 8000:
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                
                # Check circularity
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    if circularity > 0.7 and radius > 8:
                        # Keep the largest ball
                        if area > max_area:
                            max_area = area
                            best_ball = {
                                'center': (int(x), int(y)),
                                'radius': radius,
                                'area': area
                            }
        
        # Update detection state
        if best_ball is not None:
            self.detected_ball_pixel = best_ball['center']
            self.ball_detection_count += 1
            
            if self.ball_detection_count >= self.ball_detection_threshold:
                if self.state == State.SCANNING:
                    self.state = State.BALL_DETECTED
                    self.get_logger().info(f'Ball confirmed at pixel {self.detected_ball_pixel}')
        else:
            self.ball_detection_count = 0
            self.detected_ball_pixel = None
        
        # Visualization
        if self.show_debug:
            debug_img = image.copy()
            
            # Draw all valid contours
            for contour in contours:
                area = cv2.contourArea(contour)
                if 200 < area < 8000:
                    ((x, y), radius) = cv2.minEnclosingCircle(contour)
                    cv2.circle(debug_img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            
            # Draw best ball
            if best_ball:
                x, y = best_ball['center']
                r = int(best_ball['radius'])
                cv2.circle(debug_img, (x, y), r, (0, 255, 0), 3)
                cv2.circle(debug_img, (x, y), 3, (0, 0, 255), -1)
                cv2.putText(debug_img, f'Ball: {int(best_ball["area"])}px', 
                           (x-50, y-int(r)-10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (0, 255, 0), 2)
            
            # Show state
            state_text = f'State: {self.state.name} | Balls: {self.balls_picked}/{self.target_balls}'
            cv2.putText(debug_img, state_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('Ball Detection', debug_img)
            cv2.imshow('White Mask', mask)
            cv2.waitKey(1)
    
    def move_to_scan_position(self, scan_angle):
        """Move arm to scanning position with given base angle"""
        self.get_logger().info(f'Moving to scan position: {scan_angle:.2f} rad')
        
        # Create scan pose
        scan_pose = self.controller.create_pose(
            x=0.15,
            y=0.0,
            z=self.scanning_height,
            roll=0.0,
            pitch=np.pi/2,  # Point down
            yaw=0.0
        )
        
        # Get joint positions for this pose
        joint_positions = self.controller.compute_ik(scan_pose)
        
        if joint_positions is None:
            self.get_logger().warn('IK failed for scan position, using direct joint control')
            # Fallback: directly set base angle
            joint_positions = [
                scan_angle,  # top_plate_joint
                1.8,         # lower_arm_joint
                -0.5,        # upper_arm_joint
                -0.8,        # wrist_joint
                0.0          # claw_base_joint
            ]
        else:
            # Override base angle
            joint_positions[0] = scan_angle
        
        return self.controller.execute_joint_trajectory(joint_positions, duration=3.0)
    
    def estimate_ball_3d_position(self):
        """Estimate 3D position of detected ball"""
        if self.detected_ball_pixel is None or self.camera_matrix is None:
            return None
        
        try:
            # Get current end effector pose
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'ee_camera_optical_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Camera position in base_link
            cam_x = transform.transform.translation.x
            cam_y = transform.transform.translation.y
            cam_z = transform.transform.translation.z
            
            # Pixel coordinates
            px, py = self.detected_ball_pixel
            
            # Camera intrinsics
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            
            # Estimate depth (distance from camera to table)
            # Camera is at height cam_z, table is at ~1.03m
            table_height = 1.03
            depth_to_table = abs(cam_z - table_height)
            
            # Back-project to 3D in camera frame
            x_cam = (px - cx) * depth_to_table / fx
            y_cam = (py - cy) * depth_to_table / fy
            z_cam = depth_to_table
            
            # Transform to base_link
            point_cam = PoseStamped()
            point_cam.header.frame_id = 'ee_camera_optical_link'
            point_cam.pose.position.x = float(x_cam)
            point_cam.pose.position.y = float(y_cam)
            point_cam.pose.position.z = float(z_cam)
            point_cam.pose.orientation.w = 1.0
            
            point_base = tf2_geometry_msgs.do_transform_pose(point_cam, transform)
            
            ball_3d = np.array([
                point_base.pose.position.x,
                point_base.pose.position.y,
                table_height + 0.015  # Ball center height (table + radius)
            ])
            
            self.get_logger().info(f'Estimated ball position: ({ball_3d[0]:.3f}, {ball_3d[1]:.3f}, {ball_3d[2]:.3f})')
            return ball_3d
            
        except Exception as e:
            self.get_logger().error(f'Failed to estimate 3D position: {str(e)}')
            return None
    
    def open_gripper(self):
        """Open the gripper"""
        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        goal.command.max_effort = 10.0
        
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.0)
    
    def close_gripper(self):
        """Close the gripper"""
        goal = GripperCommand.Goal()
        goal.command.position = -0.4
        goal.command.max_effort = 10.0
        
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.0)
    
    def pick_ball(self, ball_position):
        """Pick up a ball at given position"""
        self.get_logger().info(f'Picking ball at {ball_position}')
        
        # Open gripper
        self.open_gripper()
        
        # Approach from above
        approach_pose = self.controller.create_pose(
            x=ball_position[0],
            y=ball_position[1],
            z=ball_position[2] + self.approach_offset,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        if not self.controller.move_to_pose(approach_pose, duration=3.0):
            return False
        
        time.sleep(0.5)
        
        # Move down to pick
        pick_pose = self.controller.create_pose(
            x=ball_position[0],
            y=ball_position[1],
            z=ball_position[2] + 0.01,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        if not self.controller.move_to_pose(pick_pose, duration=2.0):
            return False
        
        time.sleep(0.5)
        
        # Close gripper
        self.close_gripper()
        
        # Lift
        lift_pose = self.controller.create_pose(
            x=ball_position[0],
            y=ball_position[1],
            z=ball_position[2] + 0.12,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        return self.controller.move_to_pose(lift_pose, duration=2.0)
    
    def place_ball(self, place_position):
        """Place ball at target position"""
        self.get_logger().info(f'Placing ball at {place_position}')
        
        # Approach target bowl
        approach_pose = self.controller.create_pose(
            x=place_position[0],
            y=place_position[1],
            z=place_position[2] + self.approach_offset,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        if not self.controller.move_to_pose(approach_pose, duration=3.0):
            return False
        
        time.sleep(0.5)
        
        # Move down to place
        place_pose = self.controller.create_pose(
            x=place_position[0],
            y=place_position[1],
            z=place_position[2] + 0.03,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        if not self.controller.move_to_pose(place_pose, duration=2.0):
            return False
        
        time.sleep(0.5)
        
        # Release ball
        self.open_gripper()
        
        # Move away
        lift_pose = self.controller.create_pose(
            x=place_position[0],
            y=place_position[1],
            z=place_position[2] + 0.12,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        return self.controller.move_to_pose(lift_pose, duration=2.0)
    
    def run_state_machine(self):
        """Main state machine loop"""
        rate = self.create_rate(10)  # 10 Hz
        
        while rclpy.ok() and self.state != State.DONE:
            
            if self.state == State.INIT:
                self.get_logger().info('=== Starting Scanning Pick and Place ===')
                self.open_gripper()
                self.current_scan_index = 0
                self.state = State.SCANNING
                
            elif self.state == State.SCANNING:
                # Move to next scan position
                if self.current_scan_index < len(self.scan_positions):
                    scan_angle = self.scan_positions[self.current_scan_index]
                    self.move_to_scan_position(scan_angle)
                    time.sleep(2.0)  # Wait for detection
                    
                    if self.state == State.BALL_DETECTED:
                        continue  # Ball found, skip to next state
                    
                    self.current_scan_index += 1
                else:
                    # Completed full scan
                    self.get_logger().warn('Scan complete, no balls found!')
                    self.state = State.DONE
                    
            elif self.state == State.BALL_DETECTED:
                self.get_logger().info('Ball detected! Estimating position...')
                
                # Estimate 3D position
                ball_3d = self.estimate_ball_3d_position()
                
                if ball_3d is not None:
                    self.detected_ball_3d = ball_3d
                    self.state = State.MOVING_TO_PICK
                else:
                    self.get_logger().error('Failed to estimate ball position')
                    self.state = State.SCANNING
                    
            elif self.state == State.MOVING_TO_PICK:
                self.state = State.PICKING
                
            elif self.state == State.PICKING:
                success = self.pick_ball(self.detected_ball_3d)
                
                if success:
                    self.get_logger().info('Ball picked successfully!')
                    self.state = State.MOVING_TO_PLACE
                else:
                    self.get_logger().error('Failed to pick ball')
                    self.state = State.SCANNING
                    
            elif self.state == State.MOVING_TO_PLACE:
                self.state = State.PLACING
                
            elif self.state == State.PLACING:
                place_pos = self.target_bowl_center.copy()
                place_pos[2] = self.target_bowl_center[2] + self.place_height
                
                success = self.place_ball(place_pos)
                
                if success:
                    self.balls_picked += 1
                    self.get_logger().info(f'Ball placed! Total: {self.balls_picked}/{self.target_balls}')
                    
                    if self.balls_picked >= self.target_balls:
                        self.state = State.DONE
                    else:
                        # Continue scanning for next ball
                        self.current_scan_index = 0
                        self.ball_detection_count = 0
                        self.detected_ball_pixel = None
                        self.state = State.SCANNING
                else:
                    self.get_logger().error('Failed to place ball')
                    self.state = State.SCANNING
            
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()
        
        # Mission complete
        self.get_logger().info('=== Mission Complete! ===')
        self.controller.move_to_named_target('home')
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    node = ScanningVisionPickPlace()
    
    # Wait for initialization
    time.sleep(2.0)
    
    # Wait for camera and joint states
    while not node.controller.joint_state_received or node.camera_matrix is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)
    
    node.get_logger().info('System ready!')
    
    try:
        node.run_state_machine()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()