#!/usr/bin/env python3
"""
Vision-based Pick and Place Controller for Akabot
Uses end effector camera to detect and pick thermocol balls
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


class VisionPickPlace(Node):
    """Vision-based pick and place controller"""
    
    def __init__(self):
        super().__init__('vision_pick_place')
        
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
        
        # Detection parameters
        self.ball_positions = []
        self.pick_height = 1.06  # Height to pick balls
        self.place_height = 1.06  # Height to place balls
        self.approach_height = 1.15  # Height to approach from above
        
        # Bowl positions (in base_link frame)
        self.source_bowl = np.array([0.15, 0.10, 1.03])
        self.target_bowl = np.array([0.15, -0.10, 1.03])
        
        self.get_logger().info('Vision Pick and Place Controller initialized')
        
    def camera_info_callback(self, msg):
        """Store camera calibration parameters"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera parameters received')
    
    def image_callback(self, msg):
        """Process camera images to detect balls"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image.copy()
            
            # Detect white balls
            self.detect_balls(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
    
    def detect_balls(self, image):
        """Detect white thermocol balls in the image"""
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for white color
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 30, 255])
        
        # Create mask for white objects
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_balls = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area (adjust based on distance)
            if 100 < area < 5000:
                # Fit circle to contour
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                
                if radius > 5:  # Minimum radius threshold
                    detected_balls.append({
                        'center': (int(x), int(y)),
                        'radius': radius,
                        'area': area
                    })
        
        self.ball_positions = detected_balls
        
        # Visualize detections (optional, for debugging)
        if len(detected_balls) > 0:
            self.get_logger().info(f'Detected {len(detected_balls)} balls')
    
    def pixel_to_3d(self, pixel_x, pixel_y, depth=0.10):
        """
        Convert pixel coordinates to 3D position in camera frame
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            depth: Estimated depth (distance from camera)
        
        Returns:
            3D position in camera optical frame
        """
        if self.camera_matrix is None:
            return None
        
        # Camera intrinsics
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Convert to 3D coordinates in camera frame
        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth
        
        return np.array([x, y, z])
    
    def transform_to_base_link(self, point_camera_frame):
        """Transform point from camera frame to base_link frame"""
        try:
            # Get transform from camera optical frame to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'ee_camera_optical_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Create PoseStamped in camera frame
            pose_camera = PoseStamped()
            pose_camera.header.frame_id = 'ee_camera_optical_link'
            pose_camera.pose.position.x = float(point_camera_frame[0])
            pose_camera.pose.position.y = float(point_camera_frame[1])
            pose_camera.pose.position.z = float(point_camera_frame[2])
            pose_camera.pose.orientation.w = 1.0
            
            # Transform to base_link frame
            pose_base = tf2_geometry_msgs.do_transform_pose(pose_camera, transform)
            
            return np.array([
                pose_base.pose.position.x,
                pose_base.pose.position.y,
                pose_base.pose.position.z
            ])
            
        except Exception as e:
            self.get_logger().error(f'Transform error: {str(e)}')
            return None
    
    def open_gripper(self):
        """Open the gripper"""
        goal = GripperCommand.Goal()
        goal.command.position = 0.0  # Open position
        goal.command.max_effort = 10.0
        
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        time.sleep(1.0)
        self.get_logger().info('Gripper opened')
    
    def close_gripper(self):
        """Close the gripper"""
        goal = GripperCommand.Goal()
        goal.command.position = -0.4  # Closed position (grip ball)
        goal.command.max_effort = 10.0
        
        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        time.sleep(1.0)
        self.get_logger().info('Gripper closed')
    
    def move_to_scan_position(self):
        """Move arm to position for scanning the source bowl"""
        self.get_logger().info('Moving to scan position...')
        
        # Position above source bowl, looking down
        scan_pose = self.controller.create_pose(
            x=0.15,
            y=0.10,
            z=0.20,  # 20cm above bowl
            roll=0.0,
            pitch=np.pi/2,  # Point down
            yaw=0.0
        )
        
        return self.controller.move_to_pose(scan_pose, duration=4.0)
    
    def pick_ball(self, ball_position):
        """
        Pick a ball at the given position
        
        Args:
            ball_position: 3D position of ball in base_link frame
        """
        self.get_logger().info(f'Picking ball at position: {ball_position}')
        
        # Open gripper first
        self.open_gripper()
        
        # Move to approach position (above ball)
        approach_pose = self.controller.create_pose(
            x=ball_position[0],
            y=ball_position[1],
            z=ball_position[2] + 0.08,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        if not self.controller.move_to_pose(approach_pose, duration=3.0):
            self.get_logger().error('Failed to reach approach position')
            return False
        
        # Move down to pick position
        pick_pose = self.controller.create_pose(
            x=ball_position[0],
            y=ball_position[1],
            z=ball_position[2] + 0.02,  # Just above ball
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        if not self.controller.move_to_pose(pick_pose, duration=2.0):
            self.get_logger().error('Failed to reach pick position')
            return False
        
        # Close gripper to grasp ball
        self.close_gripper()
        
        # Lift ball
        lift_pose = self.controller.create_pose(
            x=ball_position[0],
            y=ball_position[1],
            z=ball_position[2] + 0.10,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        return self.controller.move_to_pose(lift_pose, duration=2.0)
    
    def place_ball(self, place_position):
        """
        Place ball at the given position
        
        Args:
            place_position: 3D position to place ball in base_link frame
        """
        self.get_logger().info(f'Placing ball at position: {place_position}')
        
        # Move to approach position above target bowl
        approach_pose = self.controller.create_pose(
            x=place_position[0],
            y=place_position[1],
            z=place_position[2] + 0.08,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        if not self.controller.move_to_pose(approach_pose, duration=3.0):
            self.get_logger().error('Failed to reach place approach position')
            return False
        
        # Move down to place position
        place_pose = self.controller.create_pose(
            x=place_position[0],
            y=place_position[1],
            z=place_position[2] + 0.04,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        if not self.controller.move_to_pose(place_pose, duration=2.0):
            self.get_logger().error('Failed to reach place position')
            return False
        
        # Open gripper to release ball
        self.open_gripper()
        
        # Move up after placing
        lift_pose = self.controller.create_pose(
            x=place_position[0],
            y=place_position[1],
            z=place_position[2] + 0.10,
            roll=0.0,
            pitch=np.pi/2,
            yaw=0.0
        )
        
        return self.controller.move_to_pose(lift_pose, duration=2.0)
    
    def run_pick_and_place(self, num_balls=3):
        """
        Main pick and place routine
        
        Args:
            num_balls: Number of balls to transfer
        """
        self.get_logger().info('='*50)
        self.get_logger().info('Starting Vision-Based Pick and Place')
        self.get_logger().info('='*50)
        
        # Wait for joint states
        while not self.controller.joint_state_received:
            rclpy.spin_once(self.controller, timeout_sec=0.1)
        
        for i in range(num_balls):
            self.get_logger().info(f'\n--- Transferring Ball {i+1}/{num_balls} ---')
            
            # Move to scan position above source bowl
            if not self.move_to_scan_position():
                self.get_logger().error('Failed to move to scan position')
                continue
            
            # Wait a bit for image processing
            time.sleep(1.0)
            
            # Check if balls detected
            if len(self.ball_positions) == 0:
                self.get_logger().warn('No balls detected! Trying manual position...')
                # Use approximate position
                ball_pos = np.array([0.15, 0.10, self.pick_height])
            else:
                # Pick the first detected ball
                ball_pixel = self.ball_positions[0]['center']
                self.get_logger().info(f'Ball detected at pixel: {ball_pixel}')
                
                # Convert to 3D (estimate depth based on known ball height)
                ball_camera = self.pixel_to_3d(ball_pixel[0], ball_pixel[1], depth=0.10)
                
                # Transform to base_link
                ball_pos = self.transform_to_base_link(ball_camera)
                
                if ball_pos is None:
                    self.get_logger().error('Failed to transform ball position')
                    continue
            
            # Pick the ball
            if not self.pick_ball(ball_pos):
                self.get_logger().error('Failed to pick ball')
                continue
            
            # Place the ball in target bowl
            place_pos = self.target_bowl.copy()
            place_pos[2] = self.place_height
            
            if not self.place_ball(place_pos):
                self.get_logger().error('Failed to place ball')
                continue
            
            self.get_logger().info(f'Ball {i+1} transferred successfully!')
        
        # Return to home position
        self.get_logger().info('\nReturning to home position...')
        self.controller.move_to_named_target('home')
        
        self.get_logger().info('='*50)
        self.get_logger().info('Pick and Place Complete!')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    
    vision_controller = VisionPickPlace()
    
    # Wait a bit for everything to initialize
    time.sleep(2.0)
    
    try:
        # Run pick and place for 3 balls
        vision_controller.run_pick_and_place(num_balls=3)
    except KeyboardInterrupt:
        vision_controller.get_logger().info('Interrupted by user')
    finally:
        vision_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()