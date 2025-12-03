#!/usr/bin/env python3
"""
Test script to verify end effector camera feed
Displays camera image with ball detection overlay
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraViewer(Node):
    """Simple camera viewer for testing"""
    
    def __init__(self):
        super().__init__('camera_viewer')
        
        self.bridge = CvBridge()
        
        # Subscribe to end effector camera
        self.image_sub = self.create_subscription(
            Image,
            '/ee_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Camera Viewer started. Press "q" to quit.')
    
    def image_callback(self, msg):
        """Display camera image with ball detection"""
        try:
            # Convert to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect white balls
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # White color range
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([180, 30, 255])
            
            mask = cv2.inRange(hsv, lower_white, upper_white)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw detections
            for contour in contours:
                area = cv2.contourArea(contour)
                
                if 100 < area < 5000:
                    ((x, y), radius) = cv2.minEnclosingCircle(contour)
                    
                    if radius > 5:
                        # Draw circle around detected ball
                        cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                        cv2.circle(cv_image, (int(x), int(y)), 3, (0, 0, 255), -1)
                        
                        # Add text
                        cv2.putText(cv_image, f'Ball', (int(x)-20, int(y)-20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Display image
            cv2.imshow('End Effector Camera - Ball Detection', cv_image)
            cv2.imshow('White Mask', mask)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Quitting...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    viewer = CameraViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()