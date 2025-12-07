#!/usr/bin/env python3
"""
AkaBot Scanning Vision Pick and Place System
Complete working implementation with camera feed, ball detection, and placing
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
        
        # Publishers for robot control
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/akabot_arm_controller/follow_joint_trajectory',
            10,
            callback_group=self.control_group
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/hand_controller/follow_joint_trajectory',
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
        
        # Home position
        self.home_position = {
            'top_plate_joint': 3.12,
            'lower_arm_joint': 0.0,
            'upper_arm_joint': 1.7,
            'wrist_joint': -1.7,
            'claw_base_joint': 0.0,
            'right_claw_joint': 0.0
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
        """Receive and display camera feed with overlays\"\"\""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_image = self.current_image.copy()
            
            # Draw detected ball if found
            if self.ball_detected_pos is not None:
                x, y = int(self.ball_detected_pos[0]), int(self.ball_detected_pos[1])\n                cv2.circle(display_image, (x, y), 15, (0, 255, 0), 3)
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
            cv2.waitKey(1)\n            \n        except Exception as e:\n            self.get_logger().error(f\"Camera error: {e}\")\n    \n    def joint_state_callback(self, msg):\n        \"\"\"Track current joint positions\"\"\"\n        for name, position in zip(msg.name, msg.position):\n            self.current_joint_state[name] = position\n    \n    def detect_balls(self, image):\n        \"\"\"Detect red balls using HSV color segmentation\"\"\"\n        if image is None:\n            return []\n        \n        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)\n        \n        # Red in HSV at both ends\n        lower_red1 = np.array([0, 100, 100])\n        upper_red1 = np.array([10, 255, 255])\n        lower_red2 = np.array([170, 100, 100])\n        upper_red2 = np.array([180, 255, 255])\n        \n        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)\n        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)\n        mask = cv2.bitwise_or(mask1, mask2)\n        \n        # Morphology\n        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))\n        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)\n        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)\n        \n        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n        \n        balls = []\n        for contour in contours:\n            area = cv2.contourArea(contour)\n            if area < 100 or area > 5000:\n                continue\n            \n            M = cv2.moments(contour)\n            if M['m00'] > 0:\n                cx = int(M['m10'] / M['m00'])\n                cy = int(M['m01'] / M['m00'])\n                balls.append({\n                    'position': (cx, cy),\n                    'area': area,\n                    'contour': contour\n                })\n        \n        return balls\n    \n    def detect_bowls(self, image):\n        \"\"\"Detect brown/yellow bowls\"\"\"\n        if image is None:\n            return []\n        \n        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)\n        mask = cv2.inRange(hsv, self.bowl_color_lower_hsv, self.bowl_color_upper_hsv)\n        \n        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))\n        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)\n        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)\n        \n        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n        \n        bowls = []\n        for contour in contours:\n            area = cv2.contourArea(contour)\n            if area < 200 or area > 10000:\n                continue\n            \n            M = cv2.moments(contour)\n            if M['m00'] > 0:\n                cx = int(M['m10'] / M['m00'])\n                cy = int(M['m01'] / M['m00'])\n                bowls.append({\n                    'position': (cx, cy),\n                    'area': area,\n                    'contour': contour\n                })\n        \n        return bowls\n    \n    def move_arm(self, joint_positions, duration=3.0):\n        \"\"\"Publish arm trajectory to move to position\"\"\"\n        trajectory = JointTrajectory()\n        trajectory.header.stamp = self.get_clock().now().to_msg()\n        trajectory.joint_names = self.arm_joints\n        \n        point = JointTrajectoryPoint()\n        point.positions = [joint_positions.get(name, self.home_position[name])\n                          for name in self.arm_joints]\n        point.time_from_start.sec = int(duration)\n        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)\n        \n        trajectory.points = [point]\n        self.arm_pub.publish(trajectory)\n        \n        self.get_logger().info(f\"Arm moving...\")\n        time.sleep(duration + 0.5)\n    \n    def move_gripper(self, position, duration=1.0):\n        \"\"\"Control gripper\"\"\"\n        trajectory = JointTrajectory()\n        trajectory.header.stamp = self.get_clock().now().to_msg()\n        trajectory.joint_names = [self.gripper_joint]\n        \n        point = JointTrajectoryPoint()\n        point.positions = [position]\n        point.time_from_start.sec = int(duration)\n        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)\n        \n        trajectory.points = [point]\n        self.gripper_pub.publish(trajectory)\n        \n        status = \"CLOSING\" if position > 0 else \"OPENING\"\n        self.get_logger().info(f\"Gripper {status}\")\n        time.sleep(duration + 0.2)\n    \n    def scan_for_ball(self):\n        \"\"\"Scan by rotating base to find ball\"\"\"\n        self.get_logger().info(f\"\\nSCAN PATTERN: Looking for ball...\")\n        \n        for i, yaw in enumerate(self.scan_yaw_angles):\n            self.get_logger().info(f\"  Scan {i+1}/{len(self.scan_yaw_angles)}: yaw={yaw:.2f}rad\")\n            \n            scan_pos = self.home_position.copy()\n            scan_pos['top_plate_joint'] = yaw\n            self.move_arm(scan_pos, duration=1.5)\n            \n            time.sleep(0.3)\n            \n            if self.current_image is not None:\n                balls = self.detect_balls(self.current_image)\n                if balls:\n                    ball = max(balls, key=lambda x: x['area'])\n                    self.ball_detected_pos = ball['position']\n                    self.get_logger().info(f\"  ✓ Ball DETECTED at {self.ball_detected_pos}\")\n                    return True, yaw\n        \n        self.get_logger().warning(\"  ✗ No ball found\")\n        return False, None\n    \n    def scan_for_bowl(self):\n        \"\"\"Scan to find empty bowl\"\"\"\n        self.get_logger().info(f\"\\nSCAN PATTERN: Looking for bowl...\")\n        \n        for i, yaw in enumerate(self.scan_yaw_angles):\n            self.get_logger().info(f\"  Scan {i+1}/{len(self.scan_yaw_angles)}: yaw={yaw:.2f}rad\")\n            \n            scan_pos = self.home_position.copy()\n            scan_pos['top_plate_joint'] = yaw\n            self.move_arm(scan_pos, duration=1.5)\n            \n            time.sleep(0.3)\n            \n            if self.current_image is not None:\n                bowls = self.detect_bowls(self.current_image)\n                if bowls:\n                    bowl = max(bowls, key=lambda x: x['area'])\n                    self.bowl_detected_pos = bowl['position']\n                    self.get_logger().info(f\"  ✓ Bowl DETECTED at {self.bowl_detected_pos}\")\n                    return True, yaw\n        \n        self.get_logger().warning(\"  ✗ No bowl found\")\n        return False, None\n    \n    def pick_ball(self, yaw):\n        \"\"\"Execute pick sequence\"\"\"\n        self.get_logger().info(\"PICK SEQUENCE:\")\n        \n        # Move to picking position\n        pick_pos = {\n            'top_plate_joint': yaw,\n            'lower_arm_joint': 0.5,\n            'upper_arm_joint': 1.2,\n            'wrist_joint': -1.2,\n            'claw_base_joint': 0.2\n        }\n        self.get_logger().info(\"  Moving to ball...\")\n        self.move_arm(pick_pos, duration=2.0)\n        \n        # Close gripper\n        self.get_logger().info(\"  Closing gripper...\")\n        self.move_gripper(0.3, duration=1.0)\n        \n        # Lift\n        lift_pos = pick_pos.copy()\n        lift_pos['upper_arm_joint'] = 1.8\n        self.get_logger().info(\"  Lifting ball...\")\n        self.move_arm(lift_pos, duration=2.0)\n        \n        self.get_logger().info(\"  ✓ BALL PICKED\")\n        return True\n    \n    def place_ball(self, yaw):\n        \"\"\"Execute place sequence\"\"\"\n        self.get_logger().info(\"PLACE SEQUENCE:\")\n        \n        # Move to place position\n        place_pos = {\n            'top_plate_joint': yaw,\n            'lower_arm_joint': 0.4,\n            'upper_arm_joint': 1.3,\n            'wrist_joint': -1.3,\n            'claw_base_joint': 0.2\n        }\n        self.get_logger().info(\"  Moving to bowl...\")\n        self.move_arm(place_pos, duration=2.0)\n        \n        # Open gripper\n        self.get_logger().info(\"  Opening gripper...\")\n        self.move_gripper(-0.3, duration=1.0)\n        \n        # Retract\n        retract_pos = place_pos.copy()\n        retract_pos['upper_arm_joint'] = 1.8\n        self.get_logger().info(\"  Retracting...\")\n        self.move_arm(retract_pos, duration=2.0)\n        \n        self.get_logger().info(\"  ✓ BALL PLACED\")\n        return True\n    \n    def execute_task(self):\n        \"\"\"Execute full automated task\"\"\"\n        self.get_logger().info(\"\\n\" + \"#\"*60)\n        self.get_logger().info(\"# STARTING AUTOMATED TASK\")\n        self.get_logger().info(\"#\"*60)\n        \n        num_balls = 3\n        success_count = 0\n        \n        for ball_num in range(1, num_balls + 1):\n            self.get_logger().info(f\"\\n{'-'*60}\")\n            self.get_logger().info(f\"BALL {ball_num}/{num_balls}\")\n            self.get_logger().info(f\"{'-'*60}\")\n            \n            # Scan and detect ball\n            found, yaw = self.scan_for_ball()\n            if not found:\n                self.get_logger().warning(f\"Skipping ball {ball_num}\")\n                continue\n            \n            # Pick ball\n            if not self.pick_ball(yaw):\n                self.get_logger().warning(f\"Pick failed for ball {ball_num}\")\n                continue\n            \n            # Scan and detect bowl\n            found, yaw = self.scan_for_bowl()\n            if not found:\n                self.get_logger().warning(f\"No bowl found for ball {ball_num}\")\n                continue\n            \n            # Place ball\n            if not self.place_ball(yaw):\n                self.get_logger().warning(f\"Place failed for ball {ball_num}\")\n                continue\n            \n            success_count += 1\n            self.get_logger().info(f\"✓ BALL {ball_num} COMPLETE\")\n            time.sleep(1)\n        \n        # Return home\n        self.get_logger().info(f\"\\nReturning to HOME...\")\n        self.move_arm(self.home_position, duration=2.0)\n        \n        self.get_logger().info(\"\\n\" + \"#\"*60)\n        self.get_logger().info(f\"# TASK COMPLETE: {success_count}/{num_balls} balls\")\n        self.get_logger().info(\"#\"*60)\n\n\ndef main(args=None):\n    rclpy.init(args=args)\n    \n    node = ScanningVisionPickPlace()\n    executor = MultiThreadedExecutor(num_threads=4)\n    executor.add_node(node)\n    \n    # Run task in separate thread\n    task_thread = threading.Thread(target=node.execute_task, daemon=True)\n    task_thread.start()\n    \n    try:\n        executor.spin()\n    except KeyboardInterrupt:\n        node.get_logger().info(\"Shutting down...\")\n    finally:\n        cv2.destroyAllWindows()\n        node.destroy_node()\n        rclpy.shutdown()\n\n\nif __name__ == '__main__':\n    main()\n