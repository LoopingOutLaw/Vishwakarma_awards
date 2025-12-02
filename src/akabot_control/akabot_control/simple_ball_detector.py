#!/usr/bin/env python3
"""
Simple Ball Detector Node with GUI
Detects white balls on a blue plate in the tf_camera feed and displays them in a window.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SimpleBallDetector(Node):
    def __init__(self):
        super().__init__('simple_ball_detector')
        
        self.bridge = CvBridge()
        
        # Subscribe to the external camera feed
        self.image_sub = self.create_subscription(
            Image,
            '/tf_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Ball Detector started. Waiting for images...')

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. Detect Balls on Plate
            processed_image, mask_combined = self.detect_balls_on_plate(cv_image)
            
            # 3. Show Live Feed Windows
            # Resize mask to 3 channels so we can stack it for visualization
            if len(mask_combined.shape) == 2:
                mask_bgr = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)
            else:
                mask_bgr = mask_combined

            # Stack original (with drawing) and mask side-by-side
            combined_view = np.hstack((processed_image, mask_bgr))
            
            # Resize if too big for screen
            if combined_view.shape[1] > 1920:
                scale_percent = 50
                width = int(combined_view.shape[1] * scale_percent / 100)
                height = int(combined_view.shape[0] * scale_percent / 100)
                dim = (width, height)
                combined_view = cv2.resize(combined_view, dim, interpolation=cv2.INTER_AREA)

            cv2.imshow("Camera Feed | Detection Mask", combined_view)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'CV Error: {str(e)}')

    def detect_balls_on_plate(self, image):
        """
        Detects white balls ONLY if they are inside the blue plate area.
        """
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # --- STEP 1: Detect the Blue Plate ---
        # Adjust these ranges if the blue plate color is different in Gazebo
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])
        
        mask_plate = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Clean up plate mask
        kernel = np.ones((5, 5), np.uint8)
        mask_plate = cv2.morphologyEx(mask_plate, cv2.MORPH_OPEN, kernel)
        mask_plate = cv2.morphologyEx(mask_plate, cv2.MORPH_CLOSE, kernel)

        # Find the largest blue contour (the plate)
        plate_contours, _ = cv2.findContours(mask_plate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_plate_contour = None
        if plate_contours:
            largest_plate_contour = max(plate_contours, key=cv2.contourArea)
            
            # Optional: Draw the detected plate boundary in Blue
            cv2.drawContours(image, [largest_plate_contour], -1, (255, 0, 0), 2)

        # --- STEP 2: Detect White Objects ---
        # Lower Saturation to detect "white"
        lower_white = np.array([0, 0, 200])      
        upper_white = np.array([180, 50, 255])  
        
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)

        # --- STEP 3: Combine Logic (White INSIDE Blue) ---
        # We only care about white pixels that are INSIDE the blue plate contour.
        # Create a mask that is 0 everywhere except inside the plate
        mask_roi = np.zeros_like(mask_white)
        if largest_plate_contour is not None:
            cv2.drawContours(mask_roi, [largest_plate_contour], -1, 255, thickness=cv2.FILLED)
        
        # Alternative approach: Find all white contours, check if their center is inside the plate contour.
        final_mask_for_display = mask_white.copy() # For visualization
        
        white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if white_contours and largest_plate_contour is not None:
            for contour in white_contours:
                area = cv2.contourArea(contour)
                
                # Filter small noise
                if area > 50:
                    # Get enclosing circle
                    ((x, y), radius) = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    
                    # Check 1: Is the center of the ball inside the Blue Plate contour?
                    # pointPolygonTest returns positive if inside, negative if outside, 0 if on edge
                    dist = cv2.pointPolygonTest(largest_plate_contour, center, False)
                    
                    if dist >= 0:
                        # Check 2: Circularity (Optional but good for filtering squares)
                        perimeter = cv2.arcLength(contour, True)
                        if perimeter == 0: continue
                        circularity = 4 * np.pi * (area / (perimeter * perimeter))
                        
                        # A perfect circle has circularity ~1.0. Allow some tolerance (e.g., > 0.7)
                        if circularity > 0.7:
                            # It's a ball on the plate!
                            cv2.circle(image, center, int(radius), (0, 255, 0), 2)
                            cv2.circle(image, center, 3, (0, 0, 255), -1)
                            cv2.putText(image, f"Ball", (center[0]+10, center[1]), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            # self.get_logger().info(f'Valid Ball found at: {center}')

        return image, final_mask_for_display

def main(args=None):
    rclpy.init(args=args)
    node = SimpleBallDetector()
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