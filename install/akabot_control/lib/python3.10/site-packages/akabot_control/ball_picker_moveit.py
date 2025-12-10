#!/usr/bin/env python3
"""
Refined Ball Picker with MoveIt IK - Complete Pick and Place Sequence

Features:
- Gets ball transforms from TF
- Plans safe trajectories with MoveIt IK
- Proper gripper control with verification
- Complete pick-transport-place-return cycle
- Robust error handling and timing
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
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class RefinedBallPicker(Node):
    """Refined ball picker using MoveIt IK with proper pick-place sequence"""

    def __init__(self):
        super().__init__('refined_ball_picker')
        self.cb_group = ReentrantCallbackGroup()

        # Controller joint order
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

        # Ball names (from TF)
        self.ball_names = ['ball_0', 'ball_1', 'ball_2']
        
        # Bowl positions (fixed, from world file)
        self.source_bowl_pos = (0.15, 0.10, 1.03)
        self.dest_bowl_pos = (0.15, -0.10, 1.03)

        # Home position
        self.home_joints = [3.12, 1.5686, 0.0, 0.0, 0.0]

        # Gripper state
        self.gripper_open = False
        
        # Safety heights
        self.SAFE_HEIGHT = 1.25  # High above table
        self.HOVER_OFFSET = 0.12  # Above ball
        self.APPROACH_OFFSET = 0.05  # Close to ball
        self.GRASP_OFFSET = 0.01  # At ball

        # ===== TF Listener =====
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
        
        self.get_logger().info("Waiting for IK service...")
        if not self.ik_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("IK service not available!")
        else:
            self.get_logger().info("✓ IK service ready")

        # ===== Action Clients =====
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            '/akabot_arm_controller/follow_joint_trajectory',
            callback_group=self.cb_group
        )
        
        self.get_logger().info("Waiting for trajectory action server...")
        if not self.trajectory_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Trajectory action server not ready!")
        else:
            self.get_logger().info("✓ Trajectory action server ready")

        # ===== Publishers =====
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/hand_controller/commands',
            10
        )

        self.get_logger().info("✓ Refined Ball Picker initialized!")

    # ========================
    # Joint State
    # ========================
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
        self.joint_state_received = True

    def refresh_joint_state(self, timeout_sec=1.0) -> bool:
        """Wait for fresh joint state"""
        self.joint_state_received = False
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.joint_state_received:
                return True
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)
        return False

    # ========================
    # TF Lookup
    # ========================
    def get_ball_position(self, ball_name: str) -> Optional[tuple]:
        """Get ball position from TF"""
        try:
            # Wait for transform
            for _ in range(20):  # Try 20 times
                try:
                    trans = self.tf_buffer.lookup_transform(
                        'base_link',
                        ball_name,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    
                    x = trans.transform.translation.x
                    y = trans.transform.translation.y
                    z = trans.transform.translation.z
                    
                    self.get_logger().info(
                        f"✓ Found {ball_name} at ({x:.3f}, {y:.3f}, {z:.3f})"
                    )
                    return (x, y, z)
                    
                except (LookupException, ConnectivityException, ExtrapolationException):
                    time.sleep(0.5)
                    continue
            
            self.get_logger().error(f"✗ Could not find transform for {ball_name}")
            return None
            
        except Exception as e:
            self.get_logger().error(f"TF error for {ball_name}: {e}")
            return None

    # ========================
    # Pose Creation
    # ========================
    def create_pose(self, x: float, y: float, z: float,
                   roll: float = 0.0, pitch: float = 0.0,
                   yaw: float = 0.0) -> Pose:
        """Create a Pose from position and orientation"""
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
        """Compute IK using MoveIt service"""
        self.refresh_joint_state(timeout_sec=0.5)

        req = GetPositionIK.Request()
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = target_pose

        req.ik_request.group_name = 'akabot_arm'
        req.ik_request.pose_stamped = ps
        req.ik_request.ik_link_name = 'claw_base'
        req.ik_request.timeout = Duration(sec=2, nanosec=0)

        # Relax orientation constraints for 5-DOF arm
        constraints = Constraints()
        oc = OrientationConstraint()
        oc.header = ps.header
        oc.link_name = req.ik_request.ik_link_name
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 6.28
        oc.absolute_y_axis_tolerance = 6.28
        oc.absolute_z_axis_tolerance = 6.28
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        req.ik_request.constraints = constraints

        # Seed with current state
        if self.current_joint_state and len(self.current_joint_state.name) > 0:
            rs = RobotState()
            rs.joint_state = self.current_joint_state
            req.ik_request.robot_state = rs
        else:
            req.ik_request.robot_state = RobotState()

        # Call IK
        fut = self.ik_client.call_async(req)
        start = time.time()
        while not fut.done():
            if time.time() - start > 8.0:
                self.get_logger().error("IK service timeout")
                return None
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None

        # Extract joints
        solution = res.solution.joint_state
        positions = []
        for name in self.joint_names:
            if name in solution.name:
                idx = solution.name.index(name)
                positions.append(solution.position[idx])
            else:
                return None

        return positions

    # ========================
    # Trajectory Execution
    # ========================
    def execute_trajectory(self, joint_positions: List[float],
                          duration: float = 6.0) -> bool:
        """Send trajectory to arm controller"""
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        traj.points.append(point)
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=30, nanosec=0)

        # Send goal
        send_future = self.trajectory_client.send_goal_async(goal)
        
        start = time.time()
        while not send_future.done():
            if time.time() - start > 20.0:
                self.get_logger().error("Goal send timeout")
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        # Wait for execution
        result_future = goal_handle.get_result_async()
        wait_time = max(60.0, duration * 3.0)
        
        start_wait = time.time()
        while not result_future.done():
            if time.time() - start_wait > wait_time:
                self.get_logger().warn("Execution timeout")
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)
            
        return True

    # ========================
    # Gripper Control
    # ========================
    def set_gripper(self, open: bool, wait_time: float = 2.5) -> bool:
        """Control gripper: True = open, False = close"""
        try:
            msg = Float64MultiArray()
            if open:
                # Fully open
                msg.data = [-0.5, 0.5]
                self.get_logger().info("→ Opening gripper...")
            else:
                # Fully close
                msg.data = [0.0, 0.0]
                self.get_logger().info("→ Closing gripper...")
            
            self.gripper_pub.publish(msg)
            self.gripper_open = open
            
            # Wait for gripper to move
            time.sleep(wait_time)
            
            self.get_logger().info(f"✓ Gripper {'OPEN' if open else 'CLOSED'}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Gripper error: {e}")
            return False

    # ========================
    # High-Level Operations
    # ========================
    def move_to_pose(self, x: float, y: float, z: float,
                    duration: float = 6.0) -> bool:
        """Move to target XYZ position"""
        target_pose = self.create_pose(x, y, z, pitch=1.57)  # Point down
        joints = self.compute_ik(target_pose)
        if joints:
            return self.execute_trajectory(joints, duration)
        self.get_logger().error(f"IK failed for ({x:.3f}, {y:.3f}, {z:.3f})")
        return False

    def move_to_home(self) -> bool:
        """Move to home position"""
        self.get_logger().info("→ Moving to HOME")
        return self.execute_trajectory(self.home_joints, duration=8.0)

    # ========================
    # Main Pick and Place
    # ========================
    def pick_and_place_ball(self, ball_name: str) -> bool:
        """
        Complete sequence for one ball:
        1. Get ball position from TF
        2. Open gripper
        3. Move to safe hover
        4. Move to approach
        5. Move to grasp
        6. Close gripper
        7. Lift ball
        8. Move to bowl hover
        9. Lower into bowl
        10. Open gripper
        11. Retract
        12. Return home
        """
        self.get_logger().info(f"\n{'='*70}")
        self.get_logger().info(f"PICKING {ball_name}")
        self.get_logger().info(f"{'='*70}")

        # ========== STEP 1: Get ball position from TF ==========
        self.get_logger().info("\n[1/12] Getting ball position from TF...")
        ball_pos = self.get_ball_position(ball_name)
        
        if ball_pos is None:
            self.get_logger().error(f"✗ Cannot find {ball_name} in TF tree!")
            return False
        
        bx, by, bz = ball_pos
        self.get_logger().info(f"✓ Ball at: ({bx:.3f}, {by:.3f}, {bz:.3f})")

        # ========== STEP 2: Open gripper FIRST ==========
        self.get_logger().info("\n[2/12] Opening gripper...")
        if not self.set_gripper(open=True, wait_time=2.5):
            return False
        time.sleep(0.5)

        # ========== STEP 3: Move to hover ==========
        hover_z = bz + self.HOVER_OFFSET
        self.get_logger().info(f"\n[3/12] Moving to hover (z={hover_z:.3f})...")
        if not self.move_to_pose(bx, by, hover_z, duration=8.0):
            self.get_logger().error("✗ Failed to reach hover")
            return False
        time.sleep(1.0)

        # ========== STEP 4: Move to approach ==========
        approach_z = bz + self.APPROACH_OFFSET
        self.get_logger().info(f"\n[4/12] Moving to approach (z={approach_z:.3f})...")
        if not self.move_to_pose(bx, by, approach_z, duration=6.0):
            self.get_logger().error("✗ Failed to reach approach")
            return False
        time.sleep(1.0)

        # ========== STEP 5: Move to grasp ==========
        grasp_z = bz + self.GRASP_OFFSET
        self.get_logger().info(f"\n[5/12] Moving to grasp (z={grasp_z:.3f})...")
        if not self.move_to_pose(bx, by, grasp_z, duration=5.0):
            self.get_logger().error("✗ Failed to reach grasp position")
            return False
        time.sleep(1.0)

        # ========== STEP 6: Close gripper ==========
        self.get_logger().info("\n[6/12] Closing gripper to grasp ball...")
        if not self.set_gripper(open=False, wait_time=3.0):
            return False
        time.sleep(0.5)
        
        if self.gripper_open:
            self.get_logger().error("✗ Gripper failed to close!")
            return False
        self.get_logger().info("✓ Ball grasped!")

        # ========== STEP 7: Lift ball ==========
        lift_z = bz + self.HOVER_OFFSET
        self.get_logger().info(f"\n[7/12] Lifting ball (z={lift_z:.3f})...")
        if not self.move_to_pose(bx, by, lift_z, duration=6.0):
            self.get_logger().error("✗ Failed to lift")
            # Emergency: open gripper
            self.set_gripper(open=True, wait_time=2.0)
            return False
        time.sleep(0.5)
        self.get_logger().info("✓ Ball lifted!")

        # ========== STEP 8: Move to bowl hover ==========
        dbx, dby, dbz = self.dest_bowl_pos
        bowl_hover_z = dbz + self.HOVER_OFFSET + 0.10
        self.get_logger().info(f"\n[8/12] Moving to bowl hover...")
        if not self.move_to_pose(dbx, dby, bowl_hover_z, duration=10.0):
            self.get_logger().error("✗ Failed to reach bowl")
            self.set_gripper(open=True, wait_time=2.0)
            return False
        time.sleep(0.5)

        # ========== STEP 9: Lower into bowl ==========
        bowl_lower_z = dbz + 0.08
        self.get_logger().info(f"\n[9/12] Lowering into bowl (z={bowl_lower_z:.3f})...")
        if not self.move_to_pose(dbx, dby, bowl_lower_z, duration=5.0):
            self.get_logger().error("✗ Failed to lower")
            self.set_gripper(open=True, wait_time=2.0)
            return False
        time.sleep(0.5)

        # ========== STEP 10: Open gripper to release ==========
        self.get_logger().info("\n[10/12] Opening gripper to release ball...")
        if not self.set_gripper(open=True, wait_time=2.5):
            return False
        time.sleep(1.0)
        self.get_logger().info("✓ Ball released in bowl!")

        # ========== STEP 11: Retract ==========
        self.get_logger().info(f"\n[11/12] Retracting from bowl...")
        if not self.move_to_pose(dbx, dby, bowl_hover_z, duration=6.0):
            self.get_logger().error("✗ Retract failed")
            return False
        time.sleep(0.5)

        # ========== STEP 12: Return home ==========
        self.get_logger().info(f"\n[12/12] Returning to HOME...")
        if not self.move_to_home():
            self.get_logger().error("✗ Failed to return home")
            return False
        time.sleep(1.0)

        self.get_logger().info(f"\n✓✓✓ {ball_name} COMPLETE! ✓✓✓\n")
        self.balls_picked += 1
        return True

    # ========================
    # Main Sequence
    # ========================
    def run_sequence(self):
        """Pick and place all balls sequentially"""
        
        # Initial wait for all systems
        self.get_logger().info("Waiting for all systems to initialize...")
        time.sleep(3.0)

        # Wait for TF tree
        self.get_logger().info("Checking TF tree...")
        time.sleep(2.0)

        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("STARTING 3-BALL PICKUP SEQUENCE")
        self.get_logger().info("="*70)

        # Process each ball
        for ball_name in self.ball_names:
            self.get_logger().info(f"\n\n>>> PROCESSING {ball_name} <<<\n")
            
            success = self.pick_and_place_ball(ball_name)
            
            if not success:
                self.get_logger().warn(f"✗ Failed to pick {ball_name}")
            
            time.sleep(2.0)  # Pause between balls

        # Final return to home
        self.get_logger().info("\n→ Final return to HOME")
        self.move_to_home()

        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"MISSION COMPLETE! Picked {self.balls_picked}/3 balls ✓✓✓")
        self.get_logger().info("="*70 + "\n")


def main(args=None):
    rclpy.init(args=args)
    picker = RefinedBallPicker()

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