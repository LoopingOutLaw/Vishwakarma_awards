#!/usr/bin/env python3
"""
Enhanced Vision-based Pick and Place with Scanning
The robot actively scans for blue (source) and red (target) bowls
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, JointState
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
    SCAN_FOR_SOURCE_BOWL = 1
    SOURCE_BOWL_FOUND = 2
    MOVE_TO_SCAN_SOURCE = 3
    DETECT_BALL = 4
    PICK_BALL = 5
    SCAN_FOR_TARGET_BOWL = 6
    TARGET_BOWL_FOUND = 7
    PLACE_BALL = 8
    RETURN_TO_SCAN = 9
    DONE = 10
    ERROR = 11


class BowlDetector:
    """Detector for colored bowls"""
    
    def __init__(self, logger):
        self.logger = logger
        
        # HSV ranges for blue bowl (source)
        self.lower_blue = np.array([100, 100, 50])
        self.upper_blue = np.array([130, 255, 255])
        
        # HSV ranges for red bowl (target)
        self.lower_red1 = np.array([0, 100, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 50])
        self.upper_red2 = np.array([180, 255, 255])
        
        self.min_bowl_area = 1000
    
    def detect_blue_bowl(self, image):
        """Detect blue bowl (source)"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_bowl_area:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return {'center': (cx, cy), 'area': area, 'contour': contour}
        
        return None
    
    def detect_red_bowl(self, image):
        """Detect red bowl (target)"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_bowl_area:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return {'center': (cx, cy), 'area': area, 'contour': contour}
        
        return None


class BallDetector:
    """Enhanced ball detection"""
    
    def __init__(self, logger):
        self.logger = logger
        self.detection_history = deque(maxlen=5)
        
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 50, 255])
        
        self.min_area = 200
        self.max_area = 8000
        self.min_circularity = 0.7
        self.min_radius = 8
        
    def detect_balls(self, image):
        """Detect white balls"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if self.min_area < area < self.max_area:
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                
                if radius > self.min_radius:
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter > 0:
                        circularity = 4 * np.pi * area / (perimeter * perimeter)
                        
                        if circularity > self.min_circularity:
                            confidence = circularity * min(area / 1000.0, 1.0)
                            
                            detections.append({
                                'center': (int(x), int(y)),
                                'radius': radius,
                                'area': area,
                                'circularity': circularity,
                                'confidence': confidence
                            })
        
        detections.sort(key=lambda d: d['confidence'], reverse=True)
        return detections, mask
    
    def get_best_detection(self, detections):
        """Get best detection with temporal filtering"""
        if not detections:
            return None
        
        self.detection_history.append(detections[0] if detections else None)
        
        valid_detections = [d for d in self.detection_history if d is not None]
        
        if len(valid_detections) >= 3:
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
    """Enhanced vision-based pick and place with scanning"""
    
    def __init__(self):
        super().__init__('enhanced_vision_pick_place')
        
        self.bridge = CvBridge()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.bowl_detector = BowlDetector(self.get_logger())
        self.ball_detector = BallDetector(self.get_logger())
        
        self.image_sub = self.create_subscription(
            Image, '/ee_camera/image_raw', self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/ee_camera/camera_info', self.camera_info_callback, 10
        )
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None
        
        self.controller = AkabotController()
        
        self.gripper_client = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd')
        
        self.state = State.INIT
        self.balls_transferred = 0
        self.target_balls = 3
        
        self.current_ball_pixel = None
        self.current_ball_3d = None
        self.source_bowl_angle = None
        self.target_bowl_angle = None
        
        self.table_height = 1.04
        
        # Scanning parameters
        self.scan_angles = np.linspace(1.56, 4.68, 20)  # Full range of base joint
        self.current_scan_index = 0
        self.bowl_found_count = 0
        self.bowl_found_threshold = 3
        
        # Motion parameters
        self.pick_offset_z = 0.01
        self.approach_offset_z = 0.08
        self.place_offset_z = 0.03
        self.lift_height = 0.12
        
        self.show_debug = True
        
        self.get_logger().info('Enhanced Vision Pick and Place with Scanning initialized')
        
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
            
            if self.state in [State.SCAN_FOR_SOURCE_BOWL, State.SCAN_FOR_TARGET_BOWL]:
                self.process_bowl_detection(cv_image)
            elif self.state == State.DETECT_BALL:
                self.process_ball_detection(cv_image)
                
        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')
    
    def process_bowl_detection(self, image):
        """Process image for bowl detection"""
        if self.state == State.SCAN_FOR_SOURCE_BOWL:
            blue_bowl = self.bowl_detector.detect_blue_bowl(image)
            if blue_bowl:
                self.bowl_found_count += 1
                self.get_logger().info(f"Blue bowl detected! Count: {self.bowl_found_count}")
                
                if self.bowl_found_count >= self.bowl_found_threshold:
                    self.get_logger().info("Blue bowl confirmed!")
                    self.state = State.SOURCE_BOWL_FOUND
            else:
                self.bowl_found_count = 0
                
        elif self.state == State.SCAN_FOR_TARGET_BOWL:
            red_bowl = self.bowl_detector.detect_red_bowl(image)
            if red_bowl:
                self.bowl_found_count += 1
                self.get_logger().info(f"Red bowl detected! Count: {self.bowl_found_count}")
                
                if self.bowl_found_count >= self.bowl_found_threshold:
                    self.get_logger().info("Red bowl confirmed!")
                    self.state = State.TARGET_BOWL_FOUND
            else:
                self.bowl_found_count = 0
        
        if self.show_debug:
            self.visualize_bowl_detection(image)
    
    def process_ball_detection(self, image):
        """Process image for ball detection"""
        detections, mask = self.ball_detector.detect_balls(image)
        best_ball = self.ball_detector.get_best_detection(detections)
        
        if best_ball:
            self.current_ball_pixel = best_ball['center']
            self.get_logger().info(
                f"Ball detected at pixel {self.current_ball_pixel} "
                f"with confidence {best_ball['confidence']:.2f}"
            )
        
        if self.show_debug:
            self.visualize_ball_detection(image, detections, best_ball, mask)
    
    def visualize_bowl_detection(self, image):
        """Visualize bowl detection"""
        debug_img = image.copy()
        
        blue_bowl = self.bowl_detector.detect_blue_bowl(image)
        red_bowl = self.bowl_detector.detect_red_bowl(image)
        
        if blue_bowl:
            cv2.drawContours(debug_img, [blue_bowl['contour']], -1, (255, 0, 0), 3)
            cx, cy = blue_bowl['center']
            cv2.circle(debug_img, (cx, cy), 10, (255, 0, 0), -1)
            cv2.putText(debug_img, "SOURCE", (cx-40, cy-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        if red_bowl:
            cv2.drawContours(debug_img, [red_bowl['contour']], -1, (0, 0, 255), 3)
            cx, cy = red_bowl['center']
            cv2.circle(debug_img, (cx, cy), 10, (0, 0, 255), -1)
            cv2.putText(debug_img, "TARGET", (cx-40, cy-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        state_text = f'State: {self.state.name} | Balls: {self.balls_transferred}/{self.target_balls}'
        cv2.putText(debug_img, state_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Bowl Detection', debug_img)
        cv2.waitKey(1)
    
    def visualize_ball_detection(self, image, detections, best_ball, mask):
        """Visualize ball detection"""
        debug_img = image.copy()
        
        for det in detections:
            x, y = det['center']
            r = int(det['radius'])
            cv2.circle(debug_img, (x, y), r, (0, 255, 255), 2)
        
        if best_ball:
            x, y = best_ball['center']
            r = int(best_ball['radius'])
            cv2.circle(debug_img, (x, y), r, (0, 255, 0), 3)
            cv2.circle(debug_img, (x, y), 3, (0, 0, 255), -1)
            cv2.putText(debug_img, "TARGET BALL", (x-50, y+r+20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        state_text = f'State: {self.state.name} | Balls: {self.balls_transferred}/{self.target_balls}'
        cv2.putText(debug_img, state_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Ball Detection', debug_img)
        cv2.imshow('White Mask', mask)
        cv2.waitKey(1)
    
    def scan_for_bowl(self, target_state):
        """Scan by rotating base joint to find bowl"""
        if self.current_scan_index >= len(self.scan_angles):
            self.current_scan_index = 0
            self.get_logger().warn("Completed full scan, no bowl found. Restarting scan...")
        
        scan_angle = self.scan_angles[self.current_scan_index]
        self.get_logger().info(f"Scanning at angle: {scan_angle:.2f} rad ({scan_angle*180/np.pi:.1f} deg)")
        
        # Move to scan position
        joint_positions = [
            scan_angle,  # top_plate_joint
            1.8,         # lower_arm_joint - high position
            -0.5,        # upper_arm_joint
            -0.8,        # wrist_joint
            0.0          # claw_base_joint
        ]
        
        success = self.controller.execute_joint_trajectory(joint_positions, duration=2.0)
        
        if success:
            time.sleep(1.0)  # Wait for image to stabilize
            self.current_scan_index += 1
        
        return success
    
    def estimate_ball_3d_position(self):
        """Estimate 3D position of detected ball"""
        if self.current_ball_pixel is None or self.camera_matrix is None:
            return None
        
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'ee_camera_optical_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            cam_z = transform.transform.translation.z
            depth = abs(cam_z - (self.table_height + 0.015))
            
            px, py = self.current_ball_pixel
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            
            x_cam = (px - cx) * depth / fx
            y_cam = (py - cy) * depth / fy
            z_cam = depth
            
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
                self.table_height + 0.015
            ])
            
            self.get_logger().info(
                f'Ball 3D position: x={ball_3d[0]:.3f}, y={ball_3d[1]:.3f}, z={ball_3d[2]:.3f}'
            )
            
            return ball_3d
            
        except Exception as e:
            self.get_logger().error(f'Failed to estimate 3D position: {str(e)}')
            return None
    
    def move_above_bowl(self, bowl_type='source'):
        """Move camera above detected bowl"""
        self.get_logger().info(f'Moving above {bowl_type} bowl...')
        
        # Get current base angle (where bowl was detected)
        current_angle = self.scan_angles[max(0, self.current_scan_index - 1)]
        
        if bowl_type == 'source':
            self.source_bowl_angle = current_angle
        else:
            self.target_bowl_angle = current_angle
        
        # Move to position above bowl
        joint_positions = [
            current_angle,  # top_plate_joint
            2.0,           # lower_arm_joint
            -0.7,          # upper_arm_joint
            -0.9,          # wrist_joint
            0.0            # claw_base_joint
        ]
        
        success = self.controller.execute_joint_trajectory(joint_positions, duration=3.0)
        
        if success:
            time.sleep(1.0)
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
        
        approach_pose = self.controller.create_pose(
            x=ball_pos[0], y=ball_pos[1],
            z=ball_pos[2] + self.approach_offset_z,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        if not self.controller.move_to_pose(approach_pose, duration=3.0):
            return False
        time.sleep(0.5)
        
        pick_pose = self.controller.create_pose(
            x=ball_pos[0], y=ball_pos[1],
            z=ball_pos[2] + self.pick_offset_z,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        if not self.controller.move_to_pose(pick_pose, duration=2.0):
            return False
        time.sleep(0.5)
        
        self.close_gripper()
        
        lift_pose = self.controller.create_pose(
            x=ball_pos[0], y=ball_pos[1],
            z=ball_pos[2] + self.lift_height,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        return self.controller.move_to_pose(lift_pose, duration=2.0)
    
    def place_sequence(self, place_pos):
        """Execute place sequence"""
        self.get_logger().info(f'Executing place sequence at {place_pos}')
        
        approach_pose = self.controller.create_pose(
            x=place_pos[0], y=place_pos[1],
            z=place_pos[2] + self.approach_offset_z,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        if not self.controller.move_to_pose(approach_pose, duration=3.0):
            return False
        time.sleep(0.5)
        
        place_pose = self.controller.create_pose(
            x=place_pos[0], y=place_pos[1],
            z=place_pos[2] + self.place_offset_z,
            roll=0.0, pitch=np.pi/2, yaw=0.0
        )
        
        if not self.controller.move_to_pose(place_pose, duration=2.0):
            return False
        time.sleep(0.5)
        
        self.open_gripper()
        
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
                self.get_logger().info('=== Starting Enhanced Pick and Place with Scanning ===')
                self.open_gripper()
                self.current_scan_index = 0
                self.bowl_found_count = 0
                self.state = State.SCAN_FOR_SOURCE_BOWL
                
            elif self.state == State.SCAN_FOR_SOURCE_BOWL:
                self.get_logger().info("Scanning for BLUE source bowl...")
                self.scan_for_bowl(State.SOURCE_BOWL_FOUND)
                
            elif self.state == State.SOURCE_BOWL_FOUND:
                self.get_logger().info("Source bowl found! Moving above it...")
                self.bowl_found_count = 0
                self.state = State.MOVE_TO_SCAN_SOURCE
                
            elif self.state == State.MOVE_TO_SCAN_SOURCE:
                if self.move_above_bowl('source'):
                    self.state = State.DETECT_BALL
                else:
                    self.get_logger().error("Failed to move above source bowl")
                    self.state = State.ERROR
                    
            elif self.state == State.DETECT_BALL:
                time.sleep(2.0)  # Wait for stable detection
                
                if self.current_ball_pixel is not None:
                    self.get_logger().info('Ball detected! Estimating 3D position...')
                    ball_3d = self.estimate_ball_3d_position()
                    
                    if ball_3d is not None:
                        self.current_ball_3d = ball_3d
                        self.state = State.PICK_BALL
                    else:
                        self.get_logger().error('Failed to estimate ball position')
                        self.current_scan_index = 0
                        self.state = State.SCAN_FOR_SOURCE_BOWL
                else:
                    self.get_logger().warn('No ball detected')
                    self.current_scan_index = 0
                    self.state = State.SCAN_FOR_SOURCE_BOWL
                    
            elif self.state == State.PICK_BALL:
                if self.pick_sequence(self.current_ball_3d):
                    self.get_logger().info('Pick successful!')
                    self.current_scan_index = 0
                    self.bowl_found_count = 0
                    self.state = State.SCAN_FOR_TARGET_BOWL
                else:
                    self.get_logger().error('Pick failed')
                    self.open_gripper()
                    self.current_scan_index = 0
                    self.state = State.SCAN_FOR_SOURCE_BOWL
                    
            elif self.state == State.SCAN_FOR_TARGET_BOWL:
                self.get_logger().info("Scanning for RED target bowl...")
                self.scan_for_bowl(State.TARGET_BOWL_FOUND)
                
            elif self.state == State.TARGET_BOWL_FOUND:
                self.get_logger().info("Target bowl found! Moving to place ball...")
                self.bowl_found_count = 0
                
                # Calculate place position based on detected target bowl angle
                current_angle = self.scan_angles[max(0, self.current_scan_index - 1)]
                self.target_bowl_angle = current_angle
                
                # Estimate target position (simplified)
                place_pos = np.array([0.15, -0.10, self.table_height + 0.05])
                
                self.state = State.PLACE_BALL
                
            elif self.state == State.PLACE_BALL:
                place_pos = np.array([0.15, -0.10, self.table_height + 0.05])
                
                if self.place_sequence(place_pos):
                    self.balls_transferred += 1
                    self.get_logger().info(
                        f'Ball placed! Progress: {self.balls_transferred}/{self.target_balls}'
                    )
                    
                    if self.balls_transferred >= self.target_balls:
                        self.state = State.DONE
                    else:
                        self.current_ball_pixel = None
                        self.ball_detector.clear_history()
                        self.current_scan_index = 0
                        self.bowl_found_count = 0
                        self.state = State.SCAN_FOR_SOURCE_BOWL
                else:
                    self.get_logger().error('Place failed')
                    self.state = State.ERROR
                    
            elif self.state == State.ERROR:
                self.get_logger().error('Error state reached. Returning home.')
                self.controller.move_to_named_target('home')
                self.state = State.DONE
            
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()
        
        self.get_logger().info('=== Mission Complete! ===')
        self.controller.move_to_named_target('home')
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    node = EnhancedVisionPickPlace()
    
    time.sleep(2.0)
    
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