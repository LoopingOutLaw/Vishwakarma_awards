#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import time
from rclpy.callback_groups import ReentrantCallbackGroup

# Safe quaternion helper
from tf_transformations import quaternion_from_euler


class AkabotController(Node):
    """
    Robust MoveIt / controller wrapper for akabot arm.
    """

    def __init__(self):
        super().__init__('akabot_controller')
        self.cb_group = ReentrantCallbackGroup()
        # Controller joint order (must match moveit controller setup)
        self.joint_names = [
            'top_plate_joint',
            'lower_arm_joint',
            'upper_arm_joint',
            'wrist_joint',
            'claw_base_joint'
        ]

        self.current_joint_state = None
        self.joint_state_received = False

        # Subscribers / clients
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10,
            callback_group=self.cb_group
        )

        # IK service (MoveIt)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        # wait up to 10s for IK service
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("IK service '/compute_ik' not available after 10s. IK calls will fail until service appears.")
        else:
            self.get_logger().info("IK service '/compute_ik' is available.")

        # Trajectory action client for the arm controller
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/akabot_arm_controller/follow_joint_trajectory'
        )
        if not self.trajectory_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().warn("Trajectory action server not available after 20s — continuing and will retry when sending goals")
        else:
            self.get_logger().info("Trajectory action server is ready.")

        self.get_logger().info('Akabot Controller initialized (Passive Mode)')

    # -------------------------
    # Joint state handling
    # -------------------------
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
        self.joint_state_received = True

    def _wait_for_future(self, future, timeout_sec):
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout_sec:
                return False
            time.sleep(0.02)
        return True

    def refresh_joint_state(self, timeout_sec=2.0):
        """Best-effort wait for a fresh joint_state message."""
        self.joint_state_received = False
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.joint_state_received:
                return True
            time.sleep(0.03)
        self.get_logger().warn("No fresh joint state received within %.2fs" % timeout_sec)
        return False

    # -------------------------
    # Pose helper
    # -------------------------
    def create_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
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

    # -------------------------
    # IK computation
    # -------------------------
    def compute_ik(self, target_pose: Pose):
        """Return joint positions in controller order or None."""
        # Attempt to refresh joint state before IK (best-effort)
        self.refresh_joint_state(timeout_sec=1.0)

        req = GetPositionIK.Request()
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = target_pose

        req.ik_request.group_name = 'akabot_arm'
        req.ik_request.pose_stamped = ps
        req.ik_request.ik_link_name = 'claw_base'   # the link that should reach target
        req.ik_request.timeout = Duration(sec=1, nanosec=0)

        # Optionally supply current robot state so IK solver has a good seed
        if self.current_joint_state is not None and len(self.current_joint_state.name) > 0:
            rs = RobotState()
            rs.joint_state = self.current_joint_state
            req.ik_request.robot_state = rs
        else:
            req.ik_request.robot_state = RobotState()  # empty

        # Call IK service
        fut = self.ik_client.call_async(req)
        if not self._wait_for_future(fut, timeout_sec=5.0):
            self.get_logger().error('IK Service Call Timed Out')
            return None

        res = fut.result()
        if res is None:
            self.get_logger().error('IK service returned None')
            return None

        self.get_logger().info(f"IK result error_code: {res.error_code.val}")

        # 1 == SUCCESS
        if res.error_code.val != 1:
            # optionally log any returned solution for debugging
            try:
                sol = res.solution.joint_state
                self.get_logger().debug(f"IK candidate names: {sol.name}")
                self.get_logger().debug(f"IK candidate positions: {sol.position}")
            except Exception:
                pass
            return None

        solution = res.solution.joint_state
        positions = []
        for name in self.joint_names:
            if name in solution.name:
                idx = solution.name.index(name)
                positions.append(solution.position[idx])
            else:
                self.get_logger().error(f"IK solution missing joint '{name}'")
                return None

        self.get_logger().info(f"IK succeeded — joint target: {positions}")
        return positions

    # -------------------------
    # Trajectory execution
    # -------------------------
    def execute_joint_trajectory(self, joint_positions, duration=5.0):
        """Send a single-point trajectory to the arm controller."""
        self.get_logger().info(f"Sending trajectory for {self.joint_names}")
        self.get_logger().info(f"Target joints: {joint_positions}")

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        # set time_from_start using builtin_interfaces Duration
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))

        traj.points.append(point)
        goal.trajectory = traj

        send_future = self.trajectory_client.send_goal_async(goal)
        self.get_logger().info("Waiting for action server to accept goal...")

        if not self._wait_for_future(send_future, timeout_sec=10.0):
            self.get_logger().error("Timed out waiting for action server")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected by controller")
            return False

        self.get_logger().info("Goal accepted, waiting for execution to finish...")
        result_future = goal_handle.get_result_async()

        # allow a generous timeout for slow sim
        wait_time = max(10.0, duration * 4.0)
        if not self._wait_for_future(result_future, timeout_sec=wait_time):
            self.get_logger().warn("Action execution timed out")
            return False

        result = result_future.result().result
        success = (getattr(result, 'error_code', 1) == 0)
        if success:
            self.get_logger().info("Trajectory execution SUCCESS")
        else:
            self.get_logger().error("Trajectory execution FAILED")
        return success

    # -------------------------
    # High-level helpers
    # -------------------------
    def move_to_pose(self, target_pose, duration=6.0):
        joints = self.compute_ik(target_pose)
        if joints:
            return self.execute_joint_trajectory(joints, duration)
        self.get_logger().error("move_to_pose(): IK returned no joints")
        return False

    def move_to_named_target(self, name):
        targets = {
            'home':  [3.12, 1.5686, 0.0, 0.0, 0.0],
            'ready': [3.12, 2.182, -0.925, -1.1177, 0.0]
        }
        if name not in targets:
            self.get_logger().error(f"Unknown named target: {name}")
            return False
        self.get_logger().info(f"Moving to named target: {name}")
        return self.execute_joint_trajectory(targets[name], duration=7.0)
