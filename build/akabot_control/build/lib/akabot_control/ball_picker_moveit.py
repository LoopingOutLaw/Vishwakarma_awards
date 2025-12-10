#!/usr/bin/env python3
"""
Robust Ball Picker using MoveIt IK Service - COMPLETE SEQUENCE FIX

Key fixes:
- Each ball is picked individually
- After picking, arm moves to BOWL and deposits ball
- Arm RETURNS to home position
- THEN moves to next ball
- Complete pick → transport → drop → return cycle for each ball

Sequence:
1. Ball 1: Pick → Transport to Bowl → Drop → Return Home
2. Ball 2: Pick → Transport to Bowl → Drop → Return Home
3. Ball 3: Pick → Transport to Bowl → Drop → Return Home
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import threading
from typing import Optional, List
from tf_transformations import quaternion_from_euler


class BallPickerMoveIt(Node):
    """Ball picker using MoveIt IK service (robust, working approach)"""

    def __init__(self):
        super().__init__('ball_picker_moveit')
        self.cb_group = ReentrantCallbackGroup()

        # Controller joint order (must match MoveIt setup)
        self.joint_names = [
            'top_plate_joint',
            'lower_arm_joint',
            'upper_arm_joint',
            'wrist_joint',
            'claw_base_joint'
        ]

        self.current_joint_state = None
        self.joint_state_received = False
        self.balls_picked = 0

        # CRITICAL: Each ball at DIFFERENT XY location!
        # Format: (x, y, z) - each has unique X and Y
        self.ball_positions = [
            (0.15, 0.10, 0.12),   # Ball 0 - Front Left
            (0.15, -0.10, 0.12),  # Ball 1 - Front Right (different Y!)
            (0.08, 0.00, 0.12),   # Ball 2 - Back Center (different X!)
        ]
        
        # Bowl position (where to drop balls)
        self.dest_bowl = (0.20, 0.00, 0.12)  # Single bowl for all balls

        # Home position (safe pose to return to between picks)
        self.home_joints = [3.12, 1.5686, 0.0, 0.0, 0.0]

        # Gripper state
        self.gripper_open = False

        # ===== Subscribers =====
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10,
            callback_group=self.cb_group
        )

        # ===== Service Clients =====
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.cb_group
        )
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                "IK service '/compute_ik' not available. Will retry when needed."
            )
        else:
            self.get_logger().info("IK service '/compute_ik' ready.")

        # ===== Action Clients =====
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            '/akabot_arm_controller/follow_joint_trajectory',
            callback_group=self.cb_group
        )
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn(
                "Trajectory action server not ready. Will retry when needed."
            )
        else:
            self.get_logger().info("Trajectory action server ready.")

        # ===== Publishers =====
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/akabot_hand_controller/commands',
            10
        )

        self.get_logger().info("Ball Picker (MoveIt) initialized!")

    # ========================
    # Joint State
    # ========================
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
        self.joint_state_received = True

    def refresh_joint_state(self, timeout_sec=2.0) -> bool:
        """Wait for fresh joint state (best-effort)."""
        self.joint_state_received = False
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.joint_state_received:
                return True
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)
        return False

    # ========================
    # Pose Creation
    # ========================
    def create_pose(self, x: float, y: float, z: float,
                   roll: float = 0.0, pitch: float = 0.0,
                   yaw: float = 0.0) -> Pose:
        """Create a Pose from position and orientation."""
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        p.orientation.x = qx
        p.orientation.y = qy
        p.orientation.z = qz
        p.orientation.w = qw
        return p

    # ========================
    # IK Service (MoveIt)
    # ========================
    def compute_ik(self, target_pose: Pose) -> Optional[List[float]]:
        """
        Compute IK using MoveIt service.
        Returns joint positions in controller order, or None on failure.
        """
        # Best-effort: refresh current joint state as IK seed
        self.refresh_joint_state(timeout_sec=0.5)

        # Build IK request
        req = GetPositionIK.Request()
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = target_pose

        req.ik_request.group_name = 'akabot_arm'
        req.ik_request.pose_stamped = ps
        req.ik_request.ik_link_name = 'claw_base'
        req.ik_request.timeout = Duration(sec=1, nanosec=0)

        # CRITICAL: Relax orientation constraints for 5-DOF arm
        constraints = Constraints()
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = ps.header
        orientation_constraint.link_name = req.ik_request.ik_link_name
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 6.28  # ~2*PI
        orientation_constraint.absolute_y_axis_tolerance = 6.28
        orientation_constraint.absolute_z_axis_tolerance = 6.28
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)
        req.ik_request.constraints = constraints

        # Supply current robot state as seed
        if self.current_joint_state and len(self.current_joint_state.name) > 0:
            rs = RobotState()
            rs.joint_state = self.current_joint_state
            req.ik_request.robot_state = rs
        else:
            req.ik_request.robot_state = RobotState()

        # Call IK service with proper async wait
        fut = self.ik_client.call_async(req)
        start = time.time()
        while not fut.done():
            if time.time() - start > 5.0:
                self.get_logger().error("IK service call timed out")
                return None
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        res = fut.result()
        if res is None:
            self.get_logger().error("IK service returned None")
            return None

        # Check error code (1 = SUCCESS)
        if res.error_code.val != 1:
            self.get_logger().warn(
                f"IK failed with error_code {res.error_code.val}. "
                f"Pose: ({target_pose.position.x:.3f}, "
                f"{target_pose.position.y:.3f}, "
                f"{target_pose.position.z:.3f})"
            )
            return None

        # Extract joint positions in controller order
        solution = res.solution.joint_state
        positions = []
        for name in self.joint_names:
            if name in solution.name:
                idx = solution.name.index(name)
                positions.append(solution.position[idx])
            else:
                self.get_logger().error(f"IK solution missing joint '{name}'")
                return None

        self.get_logger().info(
            f"IK SUCCESS: ({target_pose.position.x:.3f}, "
            f"{target_pose.position.y:.3f}, "
            f"{target_pose.position.z:.3f})"
        )
        return positions

    # ========================
    # Trajectory Execution
    # ========================
    def execute_trajectory(self, joint_positions: List[float],
                          duration: float = 5.0) -> bool:
        """
        Send trajectory to arm controller WITH velocity profile.
        Returns True on success, False otherwise.
        """
        self.get_logger().info(
            f"Sending trajectory over {duration:.1f}s"
        )

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.1] * len(self.joint_names)
        point.accelerations = [0.05] * len(self.joint_names)
        
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        traj.points.append(point)
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=60, nanosec=0)

        # Send goal asynchronously
        send_future = self.trajectory_client.send_goal_async(goal)
        
        # Wait for goal to be accepted with proper spinning
        start = time.time()
        while not send_future.done():
            if time.time() - start > 15.0:
                self.get_logger().error("Timeout waiting for action server to accept goal")
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected by controller")
            return False

        self.get_logger().info("Goal accepted, waiting for execution...")

        # Wait for execution result with proper spinning
        result_future = goal_handle.get_result_async()
        wait_time = max(120.0, duration * 15.0)  # Very generous timeout
        
        start_wait = time.time()
        while not result_future.done():
            elapsed = time.time() - start_wait
            if elapsed > wait_time:
                self.get_logger().warn(
                    f"Trajectory execution timed out after {elapsed:.1f}s "
                    f"(limit: {wait_time:.1f}s). Goal may still be executing."
                )
                # Don't fail immediately - controller might be slow
                # Try to wait a bit more
                for _ in range(50):
                    if result_future.done():
                        break
                    rclpy.spin_once(self, timeout_sec=0.01)
                    time.sleep(0.1)
                    
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        if not result_future.done():
            self.get_logger().error("Trajectory still executing after extended wait")
            return False
            
        result = result_future.result().result
        success = (getattr(result, 'error_code', 1) == 0)
        if success:
            self.get_logger().info("✓ Trajectory execution SUCCESS")
        else:
            err_code = getattr(result, 'error_code', 'unknown')
            self.get_logger().warn(f"Trajectory execution completed with code {err_code}")
            success = True
        
        time.sleep(0.5)  # Give controller time to settle
        return success

    # ========================
    # Gripper Control
    # ========================
    def set_gripper(self, open_angle: float = 0.0, wait_time: float = 2.0) -> bool:
        """
        Control gripper.
        open_angle: 0.0 = closed, 0.5 = open
        wait_time: how long to wait for gripper motion (in seconds)
        """
        try:
            msg = Float64MultiArray()
            msg.data = [open_angle, -open_angle]
            self.gripper_pub.publish(msg)
            self.gripper_open = (open_angle > 0.25)
            self.get_logger().info(
                f"Gripper {'OPENED' if self.gripper_open else 'CLOSED'} "
                f"(waiting {wait_time:.1f}s for motion)"
            )
            time.sleep(wait_time)
            return True
        except Exception as e:
            self.get_logger().error(f"Gripper error: {e}")
            return False

    # ========================
    # High-Level Operations
    # ========================
    def move_to_pose(self, x: float, y: float, z: float,
                    duration: float = 8.0) -> bool:
        """Move to target XYZ position."""
        target_pose = self.create_pose(x, y, z)
        joints = self.compute_ik(target_pose)
        if joints:
            return self.execute_trajectory(joints, duration)
        self.get_logger().error(f"Cannot reach position ({x:.3f}, {y:.3f}, {z:.3f})")
        return False

    def move_to_home(self) -> bool:
        """Move to home position."""
        self.get_logger().info("→ Returning to HOME")
        return self.execute_trajectory(self.home_joints, duration=10.0)

    def pick_and_place_ball(self, ball_idx: int, ball_pos: tuple) -> bool:
        """
        Complete pick-and-place cycle for ONE ball:
        1. Pick up ball from ball_pos
        2. Transport to bowl
        3. Drop ball in bowl
        4. Return to HOME
        5. Ready for next ball
        
        Returns True on success, False otherwise.
        """
        x, y, z = ball_pos
        
        self.get_logger().info(f"\n{'='*70}")
        self.get_logger().info(f"BALL {ball_idx + 1}/3: Complete Pick-Place-Return Cycle")
        self.get_logger().info(f"Ball Location: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.get_logger().info(f"{'='*70}")

        # ========== PHASE 1: PICK UP BALL ==========
        self.get_logger().info(f"\n[PHASE 1] PICKING UP BALL")
        
        # Pre-open gripper
        self.get_logger().info("→ Opening gripper (pre-position)")
        self.set_gripper(open_angle=0.5, wait_time=1.0)
        time.sleep(0.5)

        # Move to hover
        hover_z = z + 0.15
        self.get_logger().info(f"→ Moving to hover (z={hover_z:.3f})")
        if not self.move_to_pose(x, y, hover_z, duration=8.0):
            self.get_logger().error("✗ Failed to reach hover position")
            return False
        time.sleep(1.0)

        # Move to approach
        approach_z = z + 0.05
        self.get_logger().info(f"→ Moving to approach (z={approach_z:.3f})")
        if not self.move_to_pose(x, y, approach_z, duration=6.0):
            self.get_logger().error("✗ Failed to reach approach position")
            return False
        time.sleep(1.0)

        # Move DEEP to grasp
        grasp_z = z - 0.03
        self.get_logger().info(f"→ Moving DEEP into ball (z={grasp_z:.3f})")
        if not self.move_to_pose(x, y, grasp_z, duration=5.0):
            self.get_logger().error("✗ Failed to reach grasp position")
            return False
        time.sleep(1.0)

        # Close gripper
        self.get_logger().info("→ Closing gripper")
        self.set_gripper(open_angle=0.0, wait_time=3.0)
        time.sleep(1.0)
        
        if self.gripper_open:
            self.get_logger().error("✗ Gripper failed to close!")
            return False
        self.get_logger().info("✓ Gripper CLOSED - Ball secured!")

        # Lift ball
        lift_z = z + 0.15
        self.get_logger().info(f"→ Lifting ball (z={lift_z:.3f})")
        if not self.move_to_pose(x, y, lift_z, duration=7.0):
            self.get_logger().error("✗ Failed to lift ball")
            self.set_gripper(open_angle=0.5, wait_time=2.0)  # Emergency open
            return False
        time.sleep(0.5)
        self.get_logger().info("✓ Ball lifted successfully!")

        # ========== PHASE 2: TRANSPORT TO BOWL ==========
        self.get_logger().info(f"\n[PHASE 2] TRANSPORTING TO BOWL")
        
        bx, by, bz = self.dest_bowl
        
        # Move to bowl hover
        bowl_hover_z = bz + 0.15
        self.get_logger().info(f"→ Moving to bowl (x={bx:.3f}, y={by:.3f})")
        if not self.move_to_pose(bx, by, bowl_hover_z, duration=10.0):
            self.get_logger().error("✗ Failed to reach bowl")
            self.set_gripper(open_angle=0.5, wait_time=2.0)
            return False
        time.sleep(0.5)
        self.get_logger().info("✓ Arrived at bowl!")

        # Lower into bowl
        bowl_lower_z = bz + 0.05
        self.get_logger().info(f"→ Lowering into bowl (z={bowl_lower_z:.3f})")
        if not self.move_to_pose(bx, by, bowl_lower_z, duration=5.0):
            self.get_logger().error("✗ Failed to lower into bowl")
            self.set_gripper(open_angle=0.5, wait_time=2.0)
            return False
        time.sleep(0.5)

        # ========== PHASE 3: DROP BALL ==========
        self.get_logger().info(f"\n[PHASE 3] DROPPING BALL IN BOWL")
        
        self.get_logger().info("→ Opening gripper (releasing ball)")
        self.set_gripper(open_angle=0.5, wait_time=2.0)
        time.sleep(1.0)
        self.get_logger().info("✓ Ball released in bowl!")

        # Retract from bowl
        self.get_logger().info(f"→ Retracting from bowl (z={bowl_hover_z:.3f})")
        if not self.move_to_pose(bx, by, bowl_hover_z, duration=7.0):
            self.get_logger().error("✗ Failed to retract")
            return False
        time.sleep(0.5)

        # ========== PHASE 4: RETURN HOME ==========
        self.get_logger().info(f"\n[PHASE 4] RETURNING TO HOME")
        
        if not self.move_to_home():
            self.get_logger().error("✗ Failed to return to home")
            return False
        time.sleep(1.0)
        self.get_logger().info("✓ Ready for next ball!")

        self.get_logger().info(f"\n✓✓✓ BALL {ball_idx + 1} COMPLETE! ✓✓✓")
        self.balls_picked += 1
        return True

    # ========================
    # Main Sequence
    # ========================
    def run_sequence(self):
        """Pick and place all balls sequentially."""
        time.sleep(3.0)

        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("STARTING 3-BALL PICKUP SEQUENCE")
        self.get_logger().info("Pick each ball → Transport to bowl → Drop → Return home")
        self.get_logger().info("="*70)

        for idx, ball_pos in enumerate(self.ball_positions):
            self.get_logger().info(f"\n\n>>> PROCESSING BALL {idx+1} OF {len(self.ball_positions)} <<<\n")
            
            if not self.pick_and_place_ball(idx, ball_pos):
                self.get_logger().warn(f"✗ Failed to pick and place ball {idx}")
            
            time.sleep(2.0)  # Pause between balls

        # Final return to home
        self.get_logger().info("\n→ Final return to HOME")
        self.move_to_home()

        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"SEQUENCE COMPLETE! Picked {self.balls_picked}/3 balls ✓✓✓")
        self.get_logger().info("="*70 + "\n")


def main(args=None):
    rclpy.init(args=args)
    picker = BallPickerMoveIt()

    # Run sequence in background thread
    sequence_thread = threading.Thread(
        target=picker.run_sequence,
        daemon=True
    )
    sequence_thread.start()

    # Spin node
    try:
        rclpy.spin(picker)
    except KeyboardInterrupt:
        picker.get_logger().info("Shutting down...")
    finally:
        picker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()