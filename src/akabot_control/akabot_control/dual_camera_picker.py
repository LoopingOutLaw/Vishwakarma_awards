#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from akabot_control.akabot_controller import AkabotController
import threading
import time


class DualCameraPicker(Node):
    def __init__(self):
        super().__init__('dual_camera_picker')

        # Controller node (runs in background spinner)
        self.controller = AkabotController()

        # Action client for the hand - attach to controller node so its spinner serves it
        self.hand_client = ActionClient(self.controller, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory')

        # Heights (tweak these)
        # Heights (tweak these) - INCREASED Z values
        self.HOVER_Z = 0.25      # was 0.15 — too low, unreachable
        self.PICK_Z = 0.20       # was 0.12
        self.GRAB_Z = 0.10       # was 0.035 — very aggressive
        self.PLATE_HOVER_Z = 0.25
        self.PLATE_PLACE_Z = 0.15

        # TF listener (frame names in your URDF)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.controller)

        self.get_logger().info("Dual Camera Picker Initialized.")

    # -----------------------
    # gripper (send both joints to match controller)
    # -----------------------
    def set_gripper(self, val, wait_for_result=True, timeout=5.0):
        """val: position for right_claw_joint. left_claw_joint is mimic; send both for controllers that expect both."""
        goal = FollowJointTrajectory.Goal()
        # send both joints (must match moveit_simple_controller_manager hand_controller joint list)
        goal.trajectory.joint_names = ['right_claw_joint', 'left_claw_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [float(val), float(-float(val))]  # left claw is mimic with multiplier -1 in URDF
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]

        if not self.hand_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().warn("Hand action server not available within timeout; sending goal anyway.")

        send_fut = self.hand_client.send_goal_async(goal)
        start = time.time()
        while not send_fut.done():
            if time.time() - start > timeout:
                self.get_logger().error("Timed out sending hand goal")
                break
            time.sleep(0.02)

        if wait_for_result:
            time.sleep(1.0)  # short wait for motion and joint_state update
        else:
            time.sleep(0.1)

        # Optional verification via joint_states
        js = self.controller.current_joint_state
        if js and 'right_claw_joint' in js.name:
            idx = js.name.index('right_claw_joint')
            pos = js.position[idx]
            self.get_logger().info(f"right_claw_joint now at {pos:.4f}")
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
    def try_reach_pose_with_fallback(self, x, y, z, yaw=0.0):
        """Try multiple orientations to reach pose."""
        # Try Z-first (hover higher, then descend in smaller steps)
        pitch_list = [0.0, 0.15, 0.30, 0.45, 0.60, -0.15]  # wider range, include negative pitch
        
        for pitch in pitch_list:
            self.get_logger().info(f"Trying IK with pitch={pitch:.2f} rad and z={z:.3f}...")
            pose = self.controller.create_pose(x=x, y=y, z=z, roll=0.0, pitch=pitch, yaw=yaw)
            if self.controller.move_to_pose(pose, duration=6.0):
                self.get_logger().info(f"Reached pose at pitch={pitch:.2f}, z={z:.3f}")
                return True
            else:
                self.get_logger().info(f"Attempt failed for pitch={pitch:.2f} at z={z:.3f}")
        
        self.get_logger().error("IK FAILED for all fallback angles at this Z")
        return False


    def is_within_workspace(self, x, y, z):
        """Check if target is roughly in reachable workspace."""
        # Adjust these bounds based on your robot's actual reach
        # From home position [3.12, 1.5686, 0.0, 0.0, 0.0], estimate reach
        max_reach = 0.45  # meters from base
        distance = (x**2 + y**2)**0.5
        
        if distance > max_reach:
            self.get_logger().error(f"Target ({x:.3f}, {y:.3f}) exceeds reach ({max_reach}m)")
            return False
        
        if z < 0.08 or z > 0.35:
            self.get_logger().error(f"Z={z:.3f}m is outside safe range [0.08, 0.35]")
            return False
        
        return True
    
    # -----------------------
    # main sequence
    # -----------------------
    def run_logic(self):
        # Wait for TF tree to include the camera/objects
        self.get_logger().info("Waiting for TF Tree...")
        start = time.time()
        while rclpy.ok():
            if self.get_transform('ee_camera_optical_link'):
                self.get_logger().info("TF Tree Ready!")
                break
            if time.time() - start > 15.0:
                self.get_logger().warn("TF not ready after 15s; continuing and hoping transforms appear.")
                break
            time.sleep(0.5)

        # Move home
        self.get_logger().info("Moving Home...")
        ok = self.controller.move_to_named_target('home')
        if not ok:
            self.get_logger().warn("move_to_named_target('home') did not report success.")

        # Open gripper
        self.set_gripper(-0.5)

        # Detect ball transform
        self.get_logger().info("Looking for ball_0...")
        ball_loc = None
        for _ in range(20):
            ball_loc = self.get_transform('ball_0')
            if ball_loc:
                break
            time.sleep(0.5)
        if not ball_loc:
            self.get_logger().error("Could not find ball_0! Aborting.")
            return

        self.get_logger().info(f"Picking ball at X={ball_loc.x:.3f}, Y={ball_loc.y:.3f}")
 
        if not ball_loc:
            self.get_logger().error("Could not find ball_0! Aborting.")
            return

        self.get_logger().info(f"Picking ball at X={ball_loc.x:.3f}, Y={ball_loc.y:.3f}")
        
        # ADD THIS CHECK
        if not self.is_within_workspace(ball_loc.x, ball_loc.y, self.HOVER_Z):
            self.get_logger().error("Ball is outside reachable workspace!")
            return

        # Hover above ball (use conservative hover)
        if not self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.HOVER_Z):
            self.get_logger().error("Failed to reach hover position!")
            return

        # Intermediate approach
        if not self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.PICK_Z):
            self.get_logger().warn("Intermediate approach failed; trying slightly higher approach")
            if not self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.PICK_Z + 0.05):
                self.get_logger().error("Failed to reach pick approach position!")
                return

        # Final descend & grab (best-effort)
        if self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.GRAB_Z):
            self.set_gripper(0.0)
            grabbed = True
        else:
            self.get_logger().warn("Final descend unreachable; closing gripper at approach height.")
            self.set_gripper(0.0)
            grabbed = False

        # Lift to hover
        self.try_reach_pose_with_fallback(ball_loc.x, ball_loc.y, self.HOVER_Z)

        # Find plate
        self.get_logger().info("Looking for pink_plate...")
        plate_loc = None
        for _ in range(20):
            plate_loc = self.get_transform('pink_plate')
            if plate_loc:
                break
            time.sleep(0.5)

        if not plate_loc:
            self.get_logger().warn("Plate not found — using fallback")
            plate_loc = ball_loc
            plate_loc.y += 0.12

        self.get_logger().info(f"Placing ball at X={plate_loc.x:.3f}, Y={plate_loc.y:.3f}")

        # Place hover
        self.try_reach_pose_with_fallback(plate_loc.x, plate_loc.y, self.PLATE_HOVER_Z)
        # lower
        self.try_reach_pose_with_fallback(plate_loc.x, plate_loc.y, self.PLATE_PLACE_Z)
        # release
        self.set_gripper(-0.5)
        # lift & home
        self.try_reach_pose_with_fallback(plate_loc.x, plate_loc.y, self.HOVER_Z)
        # self.controller.move_to_named_target('home')
        self.get_logger().info("Mission Complete!")


def main(args=None):
    rclpy.init(args=args)
    picker = DualCameraPicker()
    spinner = threading.Thread(target=rclpy.spin, args=(picker.controller,), daemon=True)
    spinner.start()
    try:
        picker.run_logic()
    except KeyboardInterrupt:
        pass
    finally:
        picker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
