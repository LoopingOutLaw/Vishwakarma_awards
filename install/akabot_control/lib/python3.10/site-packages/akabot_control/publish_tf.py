#!/usr/bin/env python3
"""
Ball TF Publisher Node
1. Detects white balls on a blue plate (using verified logic).
2. Converts 2D pixels to 3D world coordinates using Ray-Plane intersection.
3. Publishes coordinate transforms (TF) for each ball.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

class BallTFPublisher(Node):
    def __init__(self):
        super().__init__('publish_tf')
        
        self.bridge = CvBridge()
        
        # --- TF Infrastructure ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # --- Camera Subscribers ---
        self.camera_model = None
        self.create_subscription(CameraInfo, '/tf_camera/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/tf_camera/image_raw', self.image_callback, 10)
        
        # --- Configuration ---
        self.start_time = self.get_clock().now()
        self.warmup_duration = 7.0  # Seconds to wait before publishing
        
        # KNOWN WORLD CONSTANT: Height of the ball center.
        # Table (~1.01m) + Ball Radius (~0.02m) = 1.03m
        self.ball_z_height = 1.03 
        
        self.get_logger().info(f'Ball TF Publisher initialized. Waiting {self.warmup_duration}s for sim stability...')

    def info_callback(self, msg):
        """Store camera intrinsic parameters"""
        self.camera_model = msg

    def image_callback(self, msg):
        # 1. Check 7-second delay
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.warmup_duration:
            return

        try:
            # 2. Convert Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 3. Detect Balls (Your verified logic)
            processed_img, balls_2d = self.detect_balls_on_plate(cv_image)
            
            # 4. Publish TF for each ball
            # We sort balls by their X-coordinate (u) to assign consistent IDs (ball_0 = left-most)
            balls_2d.sort(key=lambda b: b[0])
            
            if self.camera_model is not None:
                for i, (u, v, radius) in enumerate(balls_2d):
                    self.publish_transform(u, v, i, msg.header)
            else:
                self.get_logger().warn('Waiting for Camera Info to perform 3D projection...', throttle_duration_sec=2.0)

            # 5. Show Visualization
            cv2.imshow("Ball TF Publisher", processed_img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'CV Error: {str(e)}')

    def detect_balls_on_plate(self, image):
        """
        Your specific detection logic: White Circle INSIDE Blue Plate
        """
        detected_balls = [] # List of (u, v, radius)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # --- Step A: Find Blue Plate ---
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])
        mask_plate = cv2.inRange(hsv, lower_blue, upper_blue)
        
        kernel = np.ones((5, 5), np.uint8)
        mask_plate = cv2.morphologyEx(mask_plate, cv2.MORPH_OPEN, kernel)
        mask_plate = cv2.morphologyEx(mask_plate, cv2.MORPH_CLOSE, kernel)

        plate_contours, _ = cv2.findContours(mask_plate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_plate_contour = None
        if plate_contours:
            largest_plate_contour = max(plate_contours, key=cv2.contourArea)
            cv2.drawContours(image, [largest_plate_contour], -1, (255, 0, 0), 2) # Blue outline

        # --- Step B: Find White Objects ---
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)

        white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if white_contours and largest_plate_contour is not None:
            for contour in white_contours:
                area = cv2.contourArea(contour)
                
                # Filter small noise
                if area > 50:
                    ((x, y), radius) = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    
                    # --- Step C: Check Intersection ---
                    # Is the center of the white object inside the blue plate?
                    dist = cv2.pointPolygonTest(largest_plate_contour, center, False)
                    
                    if dist >= 0:
                        # --- Step D: Check Circularity ---
                        perimeter = cv2.arcLength(contour, True)
                        if perimeter == 0: continue
                        circularity = 4 * np.pi * (area / (perimeter * perimeter))
                        
                        if circularity > 0.7:
                            detected_balls.append((int(x), int(y), radius))
                            
                            # Visuals
                            cv2.circle(image, center, int(radius), (0, 255, 0), 2)
                            cv2.circle(image, center, 3, (0, 0, 255), -1)
                            cv2.putText(image, f"Ball", (center[0]+10, center[1]), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image, detected_balls

    def publish_transform(self, u, v, ball_id, header):
        """
        Math: Projects 2D pixel to 3D ray, intersects with table plane (Z=1.03), publishes TF.
        """
        # 1. Camera Intrinsics
        k = self.camera_model.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        
        # 2. 3D Ray in Camera Frame
        ray_x = (u - cx) / fx
        ray_y = (v - cy) / fy
        ray_z = 1.0
        ray_camera = np.array([ray_x, ray_y, ray_z])

        try:
            # 3. Get Camera Pose (Camera -> Base Link)
            # We fallback to 'base_link' if 'world' is not available
            target_frame = 'base_link' 
            source_frame = header.frame_id 
            
            if not self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):
                return

            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            
            # Position
            t_vec = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            
            # Rotation (Quaternion to Matrix)
            q = trans.transform.rotation
            # Manual conversion to avoid scipy dependency if not installed
            q_w, q_x, q_y, q_z = q.w, q.x, q.y, q.z
            R = np.array([
                [1 - 2*(q_y**2 + q_z**2), 2*(q_x*q_y - q_z*q_w), 2*(q_x*q_z + q_y*q_w)],
                [2*(q_x*q_y + q_z*q_w), 1 - 2*(q_x**2 + q_z**2), 2*(q_y*q_z - q_x*q_w)],
                [2*(q_x*q_z - q_y*q_w), 2*(q_y*q_z + q_x*q_w), 1 - 2*(q_x**2 + q_y**2)]
            ])
            
            # 4. Transform Ray to Target Frame
            ray_world = np.dot(R, ray_camera)
            
            # 5. Ray-Plane Intersection
            # We want to find 'alpha' such that: (CameraPos + alpha * Ray).z = ball_z_height
            if abs(ray_world[2]) < 1e-6: return # Parallel
            
            alpha = (self.ball_z_height - t_vec[2]) / ray_world[2]
            
            if alpha < 0: return # Behind camera
            
            intersection = t_vec + (alpha * ray_world)
            
            # 6. Broadcast
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = target_frame
            t.child_frame_id = f'ball_{ball_id}'
            
            t.transform.translation.x = intersection[0]
            t.transform.translation.y = intersection[1]
            t.transform.translation.z = intersection[2]
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = BallTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()