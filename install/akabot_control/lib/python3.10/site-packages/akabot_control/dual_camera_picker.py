#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from akabot_control.akabot_controller import AkabotController
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time


class DualCameraPicker(Node):
    def __init__(self):
        super().__init__('dual_camera_picker')

        # Controller node (runs in background spinner)
        self.controller = AkabotController()

        # Action client for the hand - attach to controller node so its spinner serves it
        self.hand_client = ActionClient(self.controller, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory')

        # Heights (ADJUSTED for realistic arm reach and precision)
        self.HOVER_Z = 0.28       # High hover, above table
        self.APPROACH_Z = 0.22    # Intermediate approach
        self.PICK_Z = 0.155       # Get close to ball (end-effector camera engagement)
        self.GRAB_Z = 0.125       # Final grip point
        self.PLATE_HOVER_Z = 0.28
        self.PLATE_PLACE_Z = 0.165

        # Ball detection parameters
        self.BALL_COLOR_LOWER = np.array([0, 0, 200])      # White ball (high red/green, high blue)
        self.BALL_COLOR_UPPER = np.array([100, 100, 255])
        self.MIN_BALL_AREA = 50
        self.MAX_BALL_AREA = 5000

        # Camera feed from end-effector
        self.ee_camera_sub = None
        self.ee_camera_image = None
        self.ee_camera_lock = threading.Lock()
        self.bridge = CvBridge()
        self.ee_camera_active = False

        # TF listener (frame names in your URDF)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.controller)

        # Ball tracking
        self.balls_to_pick = ['ball_0', 'ball_1', 'ball_2']  # All 3 balls
        self.picked_balls = 0
        self.pink_plate_pose = None

        self.get_logger().info("Dual Camera Picker Initialized.")

    # -----------------------
    # Subscribe to EE Camera
    # -----------------------
    def subscribe_to_ee_camera(self):
        """Subscribe to end-effector camera for precision alignment."""
        self.ee_camera_sub = self.controller.create_subscription(
            Image,
            '/ee_camera/image_raw',
            self.ee_camera_callback,
            10
        )
        self.ee_camera_active = True
        self.get_logger().info("Subscribed to end-effector camera")

    def ee_camera_callback(self, msg):
        """Receive camera images from end-effector."""
        with self.ee_camera_lock:
            try:
                self.ee_camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().warn(f"Camera image conversion error: {e}")

    def get_ee_camera_feed(self, timeout=2.0):
        """Get latest frame from end-effector camera."""
        start = time.time()
        while time.time() - start < timeout:
            with self.ee_camera_lock:
                if self.ee_camera_image is not None:
                    return self.ee_camera_image.copy()
            time.sleep(0.05)
        return None

    # -----------------------
    # Ball detection from EE camera
    # -----------------------
    def detect_ball_in_frame(self, frame):
        """Detect white ball in the frame using color detection.
        Returns: (x_pixel, y_pixel, area) or None if not found.
        """
        if frame is None:
            return None

        # Create mask for white color
        mask = cv2.inRange(frame, self.BALL_COLOR_LOWER, self.BALL_COLOR_UPPER)
        
        # Morphological operations to clean up
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None

        # Find largest contour (most likely the ball)
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < self.MIN_BALL_AREA or area > self.MAX_BALL_AREA:
            return None

        # Get center
        M = cv2.moments(largest)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (cx, cy, area)
        
        return None

    def refine_approach_with_ee_camera(self, base_x, base_y, target_z):
        """Use EE camera feed to refine the final approach to ball.
        Moves toward ball center in camera frame before gripper engages.
        """
        self.get_logger().info(f"Starting EE camera-guided approach at z={target_z:.3f}")
        
        max_refinement_steps = 5
        pixel_tolerance = 30  # pixels from center
        frame_center_x = 320  # Assuming 640x480 image
        frame_center_y = 240

        for step in range(max_refinement_steps):
            frame = self.get_ee_camera_feed(timeout=1.0)
            if frame is None:
                self.get_logger().warn(f"No camera feed at refinement step {step}")
                return False

            detection = self.detect_ball_in_frame(frame)
            if detection is None:
                self.get_logger().warn(f"Ball not visible in camera at step {step}")
                # Continue with last known position
                break

            cx, cy, area = detection
            self.get_logger().info(f"Step {step}: Ball at ({cx}, {cy}), area={area}")

            # Check if centered
            dx = cx - frame_center_x
            dy = cy - frame_center_y
            distance_from_center = (dx**2 + dy**2)**0.5

            if distance_from_center < pixel_tolerance:
                self.get_logger().info(f"Ball centered in camera frame after {step+1} steps")
                return True

            # Adjust position slightly (fine tuning)
            # Convert pixel error to world adjustments (rough calibration)
            pixel_to_world = 0.0002  # Empirical conversion, adjust based on your setup
            adj_x = base_x - (dx * pixel_to_world)
            adj_y = base_y - (dy * pixel_to_world)

            self.get_logger().info(f"Refining to ({adj_x:.4f}, {adj_y:.4f}, {target_z:.3f})")
            if not self.try_reach_pose_with_fallback(adj_x, adj_y, target_z, max_attempts=2):
                self.get_logger().warn(f"Failed to reach refined position at step {step}")
                break

            time.sleep(0.3)  # Brief pause for stabilization

        return True

    # -----------------------
    # gripper control
    # -----------------------
    def set_gripper(self, val, wait_for_result=True, timeout=5.0):
        """val: position for right_claw_joint. left_claw_joint is mimic; send both for controllers that expect both.
        val=0.0 means fully closed. val=-0.5 means fully open.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['right_claw_joint', 'left_claw_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [float(val), float(-float(val))]  # left claw is mimic with multiplier -1 in URDF
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]

        if not self.hand_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().warn("Hand action server not available; sending goal anyway.")

        send_fut = self.hand_client.send_goal_async(goal)
        start = time.time()
        while not send_fut.done():
            if time.time() - start > timeout:
                self.get_logger().error("Timed out sending hand goal")
                break
            time.sleep(0.02)

        if wait_for_result:
            time.sleep(1.5)  # Give gripper time to move
        else:
            time.sleep(0.1)

        # Optional verification via joint_states
        js = self.controller.current_joint_state
        if js and 'right_claw_joint' in js.name:
            idx = js.name.index('right_claw_joint')
            pos = js.position[idx]
            self.get_logger().info(f"Gripper now at position {pos:.4f}")
        else:
            self.get_logger().warn("Could not verify gripper via joint_states")

    # -----------------------
    # TF helper
    # -----------------------
    def get_transform(self, target_frame):
        try:
            if self.tf_buffer.can_transform('base_link', target_frame, rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform('base_link', target_frame, rclpy.time.Time())
                return t.transform.translation
        except Exception:
            pass
        return None

    # -----------------------
    # IK fallback tries
    # -----------------------
    def try_reach_pose_with_fallback(self, x, y, z, yaw=0.0, max_attempts=6):
        """Try multiple orientations to reach pose."""
        pitch_list = [0.0, 0.15, 0.30, 0.45, 0.60, -0.15][:max_attempts]
        
        for pitch in pitch_list:
            self.get_logger().info(f"Trying IK with pitch={pitch:.2f} rad at z={z:.3f}...")
            pose = self.controller.create_pose(x=x, y=y, z=z, roll=0.0, pitch=pitch, yaw=yaw)
            if self.controller.move_to_pose(pose, duration=6.0):
                self.get_logger().info(f"Reached pose at pitch={pitch:.2f}")
                return True
            else:
                self.get_logger().info(f"Attempt failed for pitch={pitch:.2f}")
        
        self.get_logger().error("IK FAILED for all fallback angles")
        return False

    def is_within_workspace(self, x, y, z):
        """Check if target is roughly in reachable workspace."""
        max_reach = 0.45  # meters from base
        distance = (x**2 + y**2)**0.5
        
        if distance > max_reach:
            self.get_logger().error(f"Target ({x:.3f}, {y:.3f}) exceeds reach ({max_reach}m)")
            return False
        
        if z < 0.10 or z > 0.35:
            self.get_logger().error(f"Z={z:.3f}m is outside safe range [0.10, 0.35]")
            return False
        
        return True
    
    # -----------------------
    # main sequence for one ball
    # -----------------------
    def pick_and_place_single_ball(self, ball_name, plate_name='pink_plate'):
        """Pick a single ball and place it in the plate.
        Returns: True if successful, False otherwise.
        """
        self.get_logger().info(f"\n=== PICKING {ball_name} ===")

        # Detect ball transform
        self.get_logger().info(f"Looking for {ball_name}...")
        ball_loc = None
        for attempt in range(20):
            ball_loc = self.get_transform(ball_name)
            if ball_loc:
                break
            time.sleep(0.5)
        
        if not ball_loc:
            self.get_logger().error(f"Could not find {ball_name}! Skipping.")
            return False

        self.get_logger().info(f"Ball found at X={ball_loc.x:.3f}, Y={ball_loc.y:.3f}")
        
        # Workspace check
        if not self.is_within_workspace(ball_loc.x, ball_loc.y, self.HOVER_Z):
            self.get_logger().error(f"{ball_name} is outside reachable workspace!")
            return False

        # Open gripper
        self.set_gripper(-0.5, wait_for_result=True)
        time.sleep(0.5)

        # Hover above ball
        self.get_logger().info(f"Moving to hover position above {ball_name}...")
        if not self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.HOVER_Z):
            self.get_logger().error(f"Failed to reach hover position for {ball_name}!")
            return False

        # Intermediate approach
        self.get_logger().info(f"Intermediate approach for {ball_name}...")
        if not self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.APPROACH_Z):
            self.get_logger().warn(f"Intermediate approach failed for {ball_name}; trying higher")
            if not self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.APPROACH_Z + 0.05):
                self.get_logger().error(f"Failed to reach approach position for {ball_name}!")
                return False

        # PRECISION PHASE: Use end-effector camera to fine-tune approach
        self.get_logger().info(f"Engaging end-effector camera for precision approach to {ball_name}...")
        if not self.refine_approach_with_ee_camera(ball_loc.x, ball_loc.y, self.PICK_Z):
            self.get_logger().warn(f"EE camera refinement incomplete for {ball_name}; proceeding with last known position")

        time.sleep(0.5)  # Stabilize before final descent

        # Final descend & grab (best-effort, arm should be very close now)
        self.get_logger().info(f"Final descent to grab point for {ball_name}...")
        if self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.GRAB_Z, max_attempts=3):
            self.get_logger().info(f"Gripper engaged for {ball_name}")
            self.set_gripper(0.0, wait_for_result=True, timeout=3.0)  # Fully close
            grabbed = True
        else:
            self.get_logger().warn(f"Final descend unreachable for {ball_name}; closing gripper at approach height")
            self.set_gripper(0.0, wait_for_result=True, timeout=3.0)
            grabbed = False

        time.sleep(0.5)  # Let gripper settle

        # Lift to hover
        self.get_logger().info(f"Lifting {ball_name} to hover...")
        if not self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.HOVER_Z, max_attempts=3):
            self.get_logger().warn(f"Lift failed for {ball_name}; continuing anyway")

        # Find plate
        self.get_logger().info(f"Looking for pink plate...")
        plate_loc = None
        for attempt in range(20):
            plate_loc = self.get_transform(plate_name)
            if plate_loc:
                break
            time.sleep(0.5)

        if not plate_loc:
            self.get_logger().warn("Plate not found — using fallback offset")
            plate_loc = ball_loc
            plate_loc.y -= 0.20  # Move to approximately where empty bowl is

        self.get_logger().info(f"Placing {ball_name} at plate: X={plate_loc.x:.3f}, Y={plate_loc.y:.3f}")

        # Move to plate hover
        if not self.try_reach_pose_with_fallback(plate_loc.x, plate_loc.y, self.PLATE_HOVER_Z, max_attempts=3):
            self.get_logger().warn(f"Failed to reach plate hover for {ball_name}")
            return False

        # Lower onto plate
        self.get_logger().info(f"Lowering {ball_name} onto plate...")
        if not self.try_reach_pose_with_fallback(plate_loc.x, plate_loc.y, self.PLATE_PLACE_Z, max_attempts=2):
            self.get_logger().warn(f"Lower failed for {ball_name}; releasing anyway")

        # Release
        self.get_logger().info(f"Releasing {ball_name}...")
        self.set_gripper(-0.5, wait_for_result=True, timeout=3.0)
        time.sleep(0.5)

        # Lift and return to hover
        self.get_logger().info(f"Lifting away from plate...")
        self.try_reach_pose_with_fallback(plate_loc.x, plate_loc.y, self.HOVER_Z, max_attempts=2)
        time.sleep(0.5)

        self.get_logger().info(f"✓ {ball_name} pick-and-place complete!")
        return True

    # -----------------------
    # main logic
    # -----------------------
    def run_logic(self):
        # Wait for TF tree
        self.get_logger().info("Waiting for TF Tree...")
        start = time.time()
        while rclpy.ok():
            if self.get_transform('ee_camera_optical_link'):
                self.get_logger().info("TF Tree Ready!")
                break
            if time.time() - start > 15.0:
                self.get_logger().warn("TF not ready after 15s; continuing...")
                break
            time.sleep(0.5)

        # Subscribe to EE camera
        self.subscribe_to_ee_camera()
        time.sleep(2.0)  # Wait for first images

        # Move home
        self.get_logger().info("\n=== MOVING HOME ===")
        ok = self.controller.move_to_named_target('home')
        if not ok:
            self.get_logger().warn("move_to_named_target('home') did not report success.")
        time.sleep(1.0)

        # Pick and place all 3 balls
        for ball_name in self.balls_to_pick:
            success = self.pick_and_place_single_ball(ball_name)
            if success:
                self.picked_balls += 1
            time.sleep(1.0)  # Pause between balls

        # Return home
        self.get_logger().info("\n=== RETURNING HOME ===")
        self.controller.move_to_named_target('home')
        time.sleep(1.0)

        self.get_logger().info(f"\n✓✓✓ MISSION COMPLETE! Picked {self.picked_balls}/{len(self.balls_to_pick)} balls ✓✓✓")


def main(args=None):
    rclpy.init(args=args)
    picker = DualCameraPicker()
    spinner = threading.Thread(target=rclpy.spin, args=(picker.controller,), daemon=True)
    spinner.start()
    try:
        picker.run_logic()
    except KeyboardInterrupt:
        picker.get_logger().info("Interrupted by user")
    finally:
        picker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
