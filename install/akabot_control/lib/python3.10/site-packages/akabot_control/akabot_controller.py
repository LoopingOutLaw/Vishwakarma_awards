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
from tf_transformations import quaternion_from_euler


class AkabotController(Node):
    def __init__(self):
        super().__init__('akabot_controller')
        self.cb_group = ReentrantCallbackGroup()

        # IMPORTANT: controller joint order must match moveit controllers config
        self.joint_names = [
            'top_plate_joint',
            'lower_arm_joint',
            'upper_arm_joint',
            'wrist_joint',
            'claw_base_joint'
        ]

        self.current_joint_state = None
        self.joint_state_received = False

        # Subscribers & clients
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10,
            callback_group=self.cb_group
        )
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/akabot_arm_controller/follow_joint_trajectory'
        )

        # Wait for action server (non-blocking but with timeout)
        if not self.trajectory_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().warn('Trajectory action server not available after 20s (will retry on send)')
        else:
            self.get_logger().info('Trajectory action server is ready.')

        self.get_logger().info('Akabot Controller initialized')

    # joint state callback
    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.joint_state_received = True

    def _wait_for_future(self, future, timeout_sec):
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout_sec:
                return False
            time.sleep(0.02)
        return True

    def refresh_joint_state(self, timeout=2.0):
        self.joint_state_received = False
        start = time.time()
        while time.time() - start < timeout:
            if self.joint_state_received:
                return True
            time.sleep(0.02)
        return False

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

    def compute_ik(self, target_pose, timeout=5.0):
        # Try to refresh joint state, but continue even if we can't get fresh state
        self.refresh_joint_state(timeout=1.0)

        req = GetPositionIK.Request()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose

        req.ik_request.group_name = 'akabot_arm'
        req.ik_request.pose_stamped = pose_stamped
        # IMPORTANT: tell MoveIt which link should reach the pose
        req.ik_request.ik_link_name = 'claw_base'
        req.ik_request.timeout = Duration(sec=1, nanosec=0)

        # Only include current joint state if available and non-empty
        if self.current_joint_state is not None and len(self.current_joint_state.name) > 0:
            rs = RobotState()
            rs.joint_state = self.current_joint_state
            req.ik_request.robot_state = rs

        fut = self.ik_client.call_async(req)
        if not self._wait_for_future(fut, timeout):
            self.get_logger().error('IK service call timed out')
            return None

        res = fut.result()
        if res is None:
            self.get_logger().error('IK returned None')
            return None

        self.get_logger().info(f'IK result error_code: {res.error_code.val}')
        if res.error_code.val != 1:
            # log any debug info available
            try:
                sol = res.solution.joint_state
                self.get_logger().debug(f'IK returned joints: {sol.name}')
                self.get_logger().debug(f'IK returned positions: {sol.position}')
            except Exception:
                pass
            return None

        # map solution into controller joint order
        sol = res.solution.joint_state
        positions = []
        for j in self.joint_names:
            if j in sol.name:
                idx = sol.name.index(j)
                positions.append(sol.position[idx])
            else:
                self.get_logger().error(f"IK solution missing joint '{j}'")
                return None

        self.get_logger().info(f'IK success — joints: {positions}')
        return positions

    def execute_joint_trajectory(self, joint_positions, duration=5.0):
        self.get_logger().info(f"Sending trajectory for {self.joint_names}")
        self.get_logger().info(f"Target joints: {joint_positions}")

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = joint_positions
        pt.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1)*1e9))
        traj.points.append(pt)
        goal.trajectory = traj

        send_fut = self.trajectory_client.send_goal_async(goal)
        if not self._wait_for_future(send_fut, timeout_sec=10.0):
            self.get_logger().error('Timed out waiting for action server to accept goal')
            return False

        gh = send_fut.result()
        if not gh.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False

        res_fut = gh.get_result_async()
        # allow longer time for slow sim
        if not self._wait_for_future(res_fut, timeout_sec=duration * 4.0):
            self.get_logger().warn('Trajectory execution timed out')
            return False

        res = res_fut.result().result
        success = (res.error_code == 0)
        if success:
            self.get_logger().info('Trajectory execution SUCCESS')
        else:
            self.get_logger().error('Trajectory execution FAILED (error_code %s)' % str(res.error_code))
        return success

    def move_to_pose(self, pose, duration=5.0):
        joints = self.compute_ik(pose)
        if joints:
            return self.execute_joint_trajectory(joints, duration)
        self.get_logger().error('move_to_pose(): IK returned no joints')
        return False

    def move_to_named_target(self, name):
        targets = {
            'home':  [3.12, 1.5686, 0.0, 0.0, 0.0],
            'ready': [3.12, 2.182, -0.925, -1.1177, 0.0]
        }
        if name not in targets:
            self.get_logger().error('Unknown named target: %s' % name)
            return False
        return self.execute_joint_trajectory(targets[name], duration=7.0)


def main(args=None):
    rclpy.init(args=args)
    node = AkabotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
