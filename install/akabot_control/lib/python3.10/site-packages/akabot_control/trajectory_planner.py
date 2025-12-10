#!/usr/bin/env python3
"""
akabot_controller_cartesian.py

ROS2 rclpy node: MoveIt IK + Cartesian straight-line planner + FollowJointTrajectory executor.

Features:
 - compute_ik() uses MoveIt '/compute_ik' service (with optional robot_state seed)
 - plan_cartesian_path() samples a straight line in Cartesian space and resolves IK for every sample
 - build_joint_trajectory() packs a joint-space JointTrajectory (multiple points) and sends via action
 - joint continuity checks + angle wrap-around shortest-path handling
 - debug logging that prints failing sample and numeric reasons for quick iteration
"""

import math
import time
from typing import List, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient

# Quaternion helper (safe)
from tf_transformations import quaternion_from_euler


def shortest_angle_diff(a: float, b: float) -> float:
    d = (b - a + math.pi) % (2 * math.pi) - math.pi
    return d


def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class AkabotControllerCartesian(Node):
    def __init__(self):
        super().__init__('akabot_controller_cartesian')
        self.cb_group = ReentrantCallbackGroup()

        # expected joint order for controller & MoveIt
        self.joint_names = [
            'top_plate_joint',
            'lower_arm_joint',
            'upper_arm_joint',
            'wrist_joint',
            'claw_base_joint'
        ]

        # tunables
        self.cartesian_step = 0.01        # meters between cartesian samples (smaller => straighter path)
        self.max_joint_step = 0.35        # radians max allowed change per joint between consecutive points
        self.time_per_segment = 0.20      # seconds per cartesian segment when building trajectory points
        self.ik_timeout_sec = 5.0         # wait timeout for IK service call

        # state
        self.current_joint_state: Optional[JointState] = None
        self.joint_state_received = False

        # Subscribers / clients / action clients
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10, callback_group=self.cb_group
        )

        # MoveIt IK service
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("IK service '/compute_ik' not available after 10s. IK will fail until available.")
        else:
            self.get_logger().info("IK service '/compute_ik' is available.")

        # Trajectory action client
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/akabot_arm_controller/follow_joint_trajectory'
        )
        if not self.trajectory_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().warn("Trajectory action server not available after 20s — will retry when sending goals.")
        else:
            self.get_logger().info("Trajectory action server is ready.")

        self.get_logger().info("AkabotControllerCartesian initialized")

    # ------------------------
    # Joint State handling
    # ------------------------
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
        self.joint_state_received = True

    def refresh_joint_state(self, timeout_sec: float = 1.0) -> bool:
        """Wait best-effort for a fresh joint_state message."""
        self.joint_state_received = False
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.joint_state_received:
                return True
            time.sleep(0.01)
        self.get_logger().debug("No fresh joint_state within %.2fs" % timeout_sec)
        return False

    # ------------------------
    # IK computation (MoveIt)
    # ------------------------
    def compute_ik(self, pose: Pose, seed_robot_state: Optional[RobotState] = None) -> Optional[List[float]]:
        """
        Call MoveIt's /compute_ik and return joint positions in self.joint_names order or None.
        Adds an orientation constraint with huge tolerances to effectively relax orientation.
        """
        # ensure service exists
        if not self.ik_client.service_is_ready():
            self.get_logger().error("IK service not ready")
            return None

        # refresh joint state to provide a good seed if available
        self.refresh_joint_state(timeout_sec=0.8)

        req = GetPositionIK.Request()
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose

        req.ik_request.group_name = 'akabot_arm'
        req.ik_request.pose_stamped = ps
        req.ik_request.ik_link_name = 'claw_base'
        req.ik_request.timeout = Duration(sec=1, nanosec=0)

        # relax orientation: create an orientation constraint with very large tolerances
        constraints = Constraints()
        oc = OrientationConstraint()
        oc.header = ps.header
        oc.link_name = req.ik_request.ik_link_name
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 6.28
        oc.absolute_y_axis_tolerance = 6.28
        oc.absolute_z_axis_tolerance = 6.28
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        req.ik_request.constraints = constraints

        # seed robot state
        if self.current_joint_state is not None and len(self.current_joint_state.name) > 0:
            rs = RobotState()
            rs.joint_state = self.current_joint_state
            req.ik_request.robot_state = rs
        else:
            req.ik_request.robot_state = RobotState()

        fut = self.ik_client.call_async(req)
        # wait for result
        start = time.time()
        while not fut.done():
            if time.time() - start > self.ik_timeout_sec:
                self.get_logger().error("IK service call timed out after %.2fs" % self.ik_timeout_sec)
                return None
            time.sleep(0.01)

        res = fut.result()
        if res is None:
            self.get_logger().error("IK service returned None")
            return None

        if res.error_code.val != 1:
            self.get_logger().warn(f"IK failed (error_code={res.error_code.val})")
            # try to log returned candidate for debugging
            try:
                sol = res.solution.joint_state
                self.get_logger().debug(f"IK candidate names: {sol.name}")
                self.get_logger().debug(f"IK candidate positions: {sol.position}")
            except Exception:
                pass
            return None

        sol = res.solution.joint_state
        # Map to self.joint_names order
        positions: List[float] = []
        for n in self.joint_names:
            if n in sol.name:
                idx = sol.name.index(n)
                positions.append(sol.position[idx])
            else:
                self.get_logger().error(f"IK solution missing joint '{n}'")
                return None

        self.get_logger().debug(f"IK success: {positions}")
        return positions

    # ------------------------
    # Cartesian path planning
    # ------------------------
    def plan_cartesian_line(self,
                            start_xyz: Tuple[float, float, float],
                            target_xyz: Tuple[float, float, float],
                            start_joint_positions: Optional[List[float]] = None
                            ) -> Optional[List[List[float]]]:
        """
        Generate a straight-line in Cartesian space from start_xyz -> target_xyz,
        sample points spaced by self.cartesian_step and solve IK for each.
        Returns list of joint position lists, or None if any sample fails.
        """

        sx, sy, sz = start_xyz
        tx, ty, tz = target_xyz
        dx = tx - sx
        dy = ty - sy
        dz = tz - sz
        dist = math.hypot(math.hypot(dx, dy), dz)
        if dist < 1e-6:
            self.get_logger().warn("Requested cartesian path with nearly zero distance")
            return None

        steps = max(2, int(dist / self.cartesian_step))
        self.get_logger().info(f"Planning Cartesian line: dist={dist:.3f}m steps={steps}")

        joint_traj: List[List[float]] = []
        prev_joints = None
        if start_joint_positions:
            prev_joints = start_joint_positions.copy()
            # normalize top_plate to -pi..pi for consistency
            prev_joints[0] = normalize_angle(prev_joints[0])

        for i in range(1, steps + 1):
            t = i / steps
            x = sx + dx * t
            y = sy + dy * t
            z = sz + dz * t

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            # keep neutral orientation (oriented downwards as needed)
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            ik = self.compute_ik(pose)
            if ik is None:
                # log helpful debug info about failed sample
                self.get_logger().error(f"IK failed for cartesian sample {i}/{steps} at (x={x:.3f}, y={y:.3f}, z={z:.3f})")
                return None

            # normalize base angle to be closest to prev_joints (prevent wrap jumps)
            ik[0] = normalize_angle(ik[0])
            if prev_joints is not None:
                # choose base equivalent (±2π) that minimizes the joint delta
                candidates = [ik[0] + k * 2 * math.pi for k in (-1, 0, 1)]
                best_base = candidates[0]
                best_diff = abs(shortest_angle_diff(prev_joints[0], candidates[0]))
                for c in candidates[1:]:
                    d = abs(shortest_angle_diff(prev_joints[0], c))
                    if d < best_diff:
                        best_diff = d
                        best_base = c
                ik[0] = best_base

                # joint continuity check
                too_big = False
                for j_idx in range(len(ik)):
                    diff = abs(shortest_angle_diff(prev_joints[j_idx], ik[j_idx]))
                    if diff > self.max_joint_step:
                        too_big = True
                        break
                if too_big:
                    self.get_logger().warn(f"Joint step too large at sample {i}/{steps}. Largest diff > {self.max_joint_step:.3f} rad")
                    return None

            joint_traj.append(ik)
            prev_joints = ik

        self.get_logger().info(f"Cartesian line produced {len(joint_traj)} joint waypoints")
        return joint_traj

    # ------------------------
    # Build & execute joint trajectory
    # ------------------------
    def build_joint_trajectory(self, joint_waypoints: List[List[float]], start_time_offset: float = 0.2) -> JointTrajectory:
        """
        Convert list of joint positions into a JointTrajectory with time_from_start.
        Each waypoint is spaced by self.time_per_segment seconds.
        """
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for i, jpos in enumerate(joint_waypoints):
            p = JointTrajectoryPoint()
            p.positions = jpos
            t = start_time_offset + i * self.time_per_segment
            # set time_from_start using builtin_interfaces Duration via float -> sec/nsec
            sec = int(math.floor(t))
            nsec = int(round((t - sec) * 1e9))
            p.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(p)

        return traj

    def execute_joint_trajectory(self, traj: JointTrajectory, wait_timeout: float = 30.0) -> bool:
        """
        Send a multi-point JointTrajectory as a single goal to the follow_joint_trajectory action server.
        Blocks until result or timeout.
        """
        if not self.trajectory_client.server_is_ready():
            self.get_logger().warn("Trajectory action server not ready, cannot send goal")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_fut = self.trajectory_client.send_goal_async(goal)
        # wait for accept
        start = time.time()
        while not send_fut.done():
            if time.time() - start > 10.0:
                self.get_logger().error("Timed out waiting for action server to accept goal")
                return False
            time.sleep(0.01)
        gh = send_fut.result()
        if not gh.accepted:
            self.get_logger().error("Trajectory goal rejected by server")
            return False

        self.get_logger().info("Goal accepted — waiting for execution result")
        res_fut = gh.get_result_async()
        # wait execution (allow generous time)
        start = time.time()
        timeout = max(wait_timeout, len(traj.points) * self.time_per_segment * 3.0)
        while not res_fut.done():
            if time.time() - start > timeout:
                self.get_logger().warn("Action execution timed out")
                return False
            time.sleep(0.02)
        res = res_fut.result().result
        # follow_joint_trajectory: error_code == 0 indicates success
        success = getattr(res, 'error_code', 1) == 0
        if success:
            self.get_logger().info("Trajectory Execution SUCCESS")
        else:
            self.get_logger().error(f"Trajectory Execution FAILED (error_code={getattr(res, 'error_code', 'unknown')})")
        return success

    # ------------------------
    # High-level helpers combining cartesian planning + execution
    # ------------------------
    def move_cartesian_to(self,
                          hover_xyz: Tuple[float, float, float],
                          final_xyz: Tuple[float, float, float],
                          use_current_as_start: bool = True
                          ) -> bool:
        """
        Plan: hover_xyz -> straight-line -> final_xyz. Build joint trajectory containing:
           - optionally a starting waypoint from the current robot (if available)
           - hover waypoint (single IK)
           - sampled cartesian approach waypoints
           - final IK waypoint

        Returns True on successful execution.
        """
        # seed: starting joint either current or built from IK of hover
        start_joint = None
        if use_current_as_start and self.current_joint_state is not None:
            # map current joint state to controller order
            try:
                mapping = {n: i for i, n in enumerate(self.current_joint_state.name)}
                start_joint = []
                for n in self.joint_names:
                    if n in mapping:
                        start_joint.append(self.current_joint_state.position[mapping[n]])
                    else:
                        start_joint.append(0.0)
                start_joint[0] = normalize_angle(start_joint[0])
            except Exception:
                start_joint = None

        # IK for hover
        hover_pose = Pose()
        hover_pose.position.x = float(hover_xyz[0])
        hover_pose.position.y = float(hover_xyz[1])
        hover_pose.position.z = float(hover_xyz[2])
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
        hover_pose.orientation.x = qx
        hover_pose.orientation.y = qy
        hover_pose.orientation.z = qz
        hover_pose.orientation.w = qw

        hover_joints = self.compute_ik(hover_pose)
        if hover_joints is None:
            self.get_logger().error("Hover IK failed — cannot plan approach")
            return False

        # IK for final exact pose
        final_pose = Pose()
        final_pose.position.x = float(final_xyz[0])
        final_pose.position.y = float(final_xyz[1])
        final_pose.position.z = float(final_xyz[2])
        final_pose.orientation.x = qx
        final_pose.orientation.y = qy
        final_pose.orientation.z = qz
        final_pose.orientation.w = qw

        final_joints = self.compute_ik(final_pose)
        if final_joints is None:
            self.get_logger().warn("Final exact IK failed — will attempt approach to a near target")
            # fall back: try to approach slightly above final
            final_joints = None

        # Plan cartesian from hover -> final_approach_z
        approach_xyz = (final_xyz[0], final_xyz[1], max(final_xyz[2] + 0.02, hover_xyz[2] - 0.05))
        cart_joints_seq = self.plan_cartesian_line(hover_xyz, approach_xyz, start_joint_positions=hover_joints)
        if cart_joints_seq is None:
            self.get_logger().warn("Cartesian sampling failed — trying single-step joint approach (hover->approach->final)")
            # fallback: do hover -> approach IK -> final IK
            approach_pose = Pose()
            approach_pose.position.x = approach_xyz[0]
            approach_pose.position.y = approach_xyz[1]
            approach_pose.position.z = approach_xyz[2]
            approach_pose.orientation.x = qx
            approach_pose.orientation.y = qy
            approach_pose.orientation.z = qz
            approach_pose.orientation.w = qw
            approach_joints = self.compute_ik(approach_pose)
            if approach_joints is None:
                self.get_logger().error("Fallback approach IK failed")
                return False
            seq = []
            if start_joint:
                seq.append(start_joint)
            seq.append(hover_joints)
            seq.append(approach_joints)
            if final_joints:
                seq.append(final_joints)
            traj = self.build_joint_trajectory(seq)
            return self.execute_joint_trajectory(traj)
        else:
            # we have a sequence of joints from hover->approach
            seq = []
            if start_joint:
                seq.append(start_joint)
            seq.append(hover_joints)
            seq.extend(cart_joints_seq)
            if final_joints:
                seq.append(final_joints)
            traj = self.build_joint_trajectory(seq)
            return self.execute_joint_trajectory(traj)

    # ------------------------
    # Convenience demo API
    # ------------------------
    def create_pose(self, x: float, y: float, z: float, roll=0.0, pitch=0.0, yaw=0.0) -> Pose:
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


def main(args=None):
    rclpy.init(args=args)
    node = AkabotControllerCartesian()

    try:
        # small example: move to a hover above (0.15, 0.10, 0.065) then approach the final z
        # You can change these values to your detected ball position
        hover = (0.15, 0.10, 0.30)   # start by moving to hover (safe high Z)
        final = (0.15, 0.10, 0.065)  # final target (ball top)
        node.get_logger().info("Demo: moving to hover then approaching final point")
        ok = node.move_cartesian_to(hover, final, use_current_as_start=True)
        node.get_logger().info(f"Demo finished: success={ok}")
        # keep spinning if you want manual calls
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (KeyboardInterrupt)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
