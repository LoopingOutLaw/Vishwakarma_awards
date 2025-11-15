#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # ROS2 standard Image message
from std_msgs.msg import String     # ROS2 standard String message
from cv_bridge import CvBridge    # Package to convert ROS images to OpenCV
import cv2
import numpy as np

# --- (1) CONFIGURE YOUR VISION HERE ---

# HSV color range for a white ball. You MUST tune these values.
# You can use a script to find these.
WHITE_LOWER = np.array([0, 0, 200])
WHITE_UPPER = np.array([255, 50, 255])

# This is the (X,Y) pixel coordinate of your basket in the camera view
# You MUST update this by running the camera and finding the pixel.
BASKET_PIXEL_X = 500
BASKET_PIXEL_Y = 300

# -------------------------------------

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('Vision Node (Eyes) is running...')
        
        # ROS2 "Bridge" to convert Image messages to OpenCV frames
        self.bridge = CvBridge()

        # Subscribe to the laptop's camera feed
        # (This assumes a default camera node is publishing to /camera/image_raw)
        # You may need to start your camera node first!
        # `ros2 run camera_ros camera_node`
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # This is a common topic for cameras
            self.image_callback,
            10)

        # Publish the state of the world as a string
        self.world_state_pub = self.create_publisher(String, '/world_state', 10)

        # --- (2) FAKE CALIBRATION ---
        # This is your MOST IMPORTANT task. You must replace this
        # with a real cv2.getPerspectiveTransform or similar.
        # This fake transform just pretends pixels are centimeters.
        self.get_logger().warn('USING FAKE PIXEL-TO-CM CALIBRATION!')
        # ----------------------------

    def transform_pixel_to_world(self, px, py):
        # --- !!! FAKE CALIBRATION !!! ---
        # This is a DANGEROUSLY simple placeholder.
        # It assumes (0,0) pixel is (0,0) cm and scales.
        # You MUST replace this with a real calibration matrix.
        # For example, it doesn't account for height (Z).
        
        real_x_cm = round(px / 10.0, 1) # Example: 320px -> 32.0cm
        real_y_cm = round(py / 10.0, 1)
        real_z_cm = 0.0 # It assumes the ball is on the table
        
        return (real_x_cm, real_y_cm, real_z_cm)
        # ---------------------------------

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV frame
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Find the ball
        ball_pos_str = "ball_at:unknown"
        
        # --- OpenCV Ball Finding Logic ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Find the largest contour (assume it's the ball)
            c = max(contours, key=cv2.contourArea)
            ((px, py), radius) = cv2.minEnclosingCircle(c)
            
            if radius > 10: # Only count if ball is big enough
                # Convert pixel (x,y) to real-world (X,Y,Z)
                (rx, ry, rz) = self.transform_pixel_to_world(px, py)
                ball_pos_str = f"ball_at:({rx},{ry},{rz})"
                
                # Draw on the frame (for debugging)
                cv2.circle(frame, (int(px), int(py)), int(radius), (0, 255, 0), 2)

        # --- Get Basket Position ---
        (bx, by, bz) = self.transform_pixel_to_world(BASKET_PIXEL_X, BASKET_PIXEL_Y)
        basket_pos_str = f"basket_at:({bx},{by},{bz})"
        cv2.circle(frame, (BASKET_PIXEL_X, BASKET_PIXEL_Y), 10, (0, 0, 255), 3) # Draw basket

        # --- Publish the World State ---
        state_msg = String()
        state_msg.data = f"{ball_pos_str}, {basket_pos_str}, gripper:unknown"
        self.world_state_pub.publish(state_msg)

        # Show the debug window
        cv2.imshow("Vision Node (Eyes)", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()