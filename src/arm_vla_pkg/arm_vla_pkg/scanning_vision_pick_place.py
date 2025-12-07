#!/usr/bin/env python3
"""
Scanning Vision Pick and Place System for AkaBot
Detects balls using depth camera, calculates transforms, moves robot to pick/place positions
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import numpy as np
from sensor_msgs.msg import PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_srvs.srv import Trigger
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
import cv2
from cv_bridge import CvBridge
import threading
import time

class ScanningVisionPickPlace(Node):
    def __init__(self):
        super().__init__('scanning_vision_pick_place')
        
        # Parameters
        self.declare_parameter('depth_camera_topic', '/camera/depth/points')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('move_group_name', 'arm')
        self.declare_parameter('ee_frame', 'ee_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ball_radius_m', 0.015)  # 15mm ball
        self.declare_parameter('detection_min_height', 0.95)  # meters
        self.declare_parameter('num_balls', 3)
        self.declare_parameter('gripper_open_value', 0.5)
        self.declare_parameter('gripper_close_value', -0.5)
        
        self.depth_camera_topic = self.get_parameter('depth_camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.move_group_name = self.get_parameter('move_group_name').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ball_radius = self.get_parameter('ball_radius_m').value
        self.detection_min_height = self.get_parameter('detection_min_height').value
        self.num_balls = self.get_parameter('num_balls').value
        self.gripper_open = self.get_parameter('gripper_open_value').value
        self.gripper_close = self.get_parameter('gripper_close_value').value
        
        # State variables
        self.point_cloud = None
        self.camera_info = None
        self.latest_ball_position = None
        self.balls_picked = 0
        self.stop_flag = False
        
        # Callback groups for thread safety
        callback_group_1 = MutuallyExclusiveCallbackGroup()
        callback_group_2 = MutuallyExclusiveCallbackGroup()
        callback_group_reentrant = ReentrantCallbackGroup()
        
        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            self.depth_camera_topic,
            self.pointcloud_callback,
            10,
            callback_group=callback_group_1
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
            callback_group=callback_group_2
        )
        
        # Action client for movement
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # TF2 for transforms
        self.tf_buffer = None
        self.tf_listener = None
        self._init_tf2()
        
        self.bridge = CvBridge()
        self.get_logger().info('Scanning Vision Pick Place node initialized')
        
    def _init_tf2(self):
        try:
            from tf2_ros import TransformListener, Buffer
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except ImportError:
            self.get_logger().warn('tf2_ros not available, using approximate transforms')
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Callback for point cloud data"""
        self.point_cloud = msg
    
    def camera_info_callback(self, msg: CameraInfo):
        """Callback for camera info"""
        self.camera_info = msg
    
    def detect_ball(self) -> tuple[bool, np.ndarray]:
        """
        Detect ball in point cloud using depth-based clustering
        Returns: (detected, position_xyz)
        """
        if self.point_cloud is None:
            self.get_logger().warn('No point cloud data available')
            return False, None
        
        # Convert point cloud to numpy array
        points = np.array(list(pc2.read_points(self.point_cloud)))
        
        if len(points) == 0:
            return False, None
        
        # Filter points by height (detection_min_height to avoid table)
        points = points[points[:, 2] > self.detection_min_height]
        
        if len(points) < 10:
            return False, None
        
        # Cluster nearby points (simple radius-based clustering)
        clusters = self._cluster_points(points, radius=self.ball_radius * 3)
        
        if len(clusters) == 0:
            return False, None
        
        # Find largest cluster (likely the ball)
        largest_cluster = max(clusters, key=len)
        
        if len(largest_cluster) < 5:  # Minimum points for a valid cluster
            return False, None
        
        # Calculate centroid
        centroid = np.mean(largest_cluster, axis=0)
        
        # Validate cluster as ball (check radius)
        distances = np.linalg.norm(largest_cluster - centroid, axis=1)
        mean_radius = np.mean(distances)
        
        if mean_radius < self.ball_radius * 0.5 or mean_radius > self.ball_radius * 2:
            return False, None
        
        self.latest_ball_position = centroid
        self.get_logger().info(f'Ball detected at {centroid}')
        return True, centroid
    
    def _cluster_points(self, points, radius=0.05):
        """
        Simple radius-based clustering
        """
        clusters = []
        used = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if used[i]:
                continue
            
            cluster = [points[i]]
            used[i] = True
            
            for j in range(i+1, len(points)):
                if used[j]:
                    continue
                
                if np.linalg.norm(points[j] - points[i]) < radius:
                    cluster.append(points[j])
                    used[j] = True
            
            if len(cluster) > 2:
                clusters.append(np.array(cluster))
        
        return clusters
    
    def scan_for_ball(self) -> bool:
        """
        Scan by yawing (rotating top_plate_joint) to find a ball
        """
        self.get_logger().info('Starting scan for ball...')
        
        # Scan positions (yaw angles)
        scan_angles = [1.56, 2.5, 3.5, 4.68, 3.5, 2.5]  # yaw angles
        
        for angle in scan_angles:
            if self.stop_flag:
                return False
            
            # Move to scan position
            if not self.move_to_scan_position(angle):
                continue
            
            time.sleep(0.5)  # Wait for point cloud to update
            
            # Try to detect ball
            detected, position = self.detect_ball()
            if detected:
                return True
        
        self.get_logger().warn('No ball detected during scan')
        return False
    
    def move_to_scan_position(self, yaw_angle: float) -> bool:
        """
        Move robot to a scanning position
        """
        # Safe position for scanning
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Fixed position, only yaw changes
        target_pose.pose.position = Point(x=0.15, y=0.0, z=0.8)
        
        # Quaternion from yaw angle (simplified)
        q = R.from_euler('z', yaw_angle).as_quat()
        target_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return self.move_to_pose(target_pose)
    
    def move_to_ball(self) -> bool:
        """
        Move end effector to detected ball position for picking
        """
        if self.latest_ball_position is None:
            self.get_logger().error('No ball position available')
            return False
        
        # Create pose above the ball
        approach_height = 0.05  # Approach from above
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.point_cloud.header.frame_id  # Camera frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position directly above ball
        target_pose.pose.position = Point(
            x=self.latest_ball_position[0],
            y=self.latest_ball_position[1],
            z=self.latest_ball_position[2] + approach_height
        )
        
        # Gripper pointing down
        q = R.from_euler('xyz', [np.pi, 0, 0]).as_quat()
        target_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return self.move_to_pose(target_pose)
    
    def move_to_pose(self, target_pose: PoseStamped) -> bool:
        """
        Move end effector to target pose using MoveIt
        """
        if not self.move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('MoveGroup action server not available')
            return False
        
        goal = MoveGroup.Goal()
        goal.request.workspace_parameters.header = target_pose.header
        goal.request.workspace_parameters.min_corner = Point(x=-1.0, y=-1.0, z=0.0)
        goal.request.workspace_parameters.max_corner = Point(x=1.0, y=1.0, z=2.0)
        
        goal.request.goal_constraints.append(self._create_pose_constraint(target_pose))
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5
        goal.planning_options.plan_only = False
        goal.planning_options.look_with_approx_ik = True
        
        future = self.move_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().warn('MoveGroup action timed out')
            return False
        
        result = future.result()
        success = result.response.error_code.val == 0 if result else False
        
        if success:
            self.get_logger().info('Move succeeded')
        else:
            self.get_logger().warn('Move failed')
        
        return success
    
    def _create_pose_constraint(self, pose: PoseStamped):
        """
        Create a pose constraint for MoveIt
        """
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = self.ee_frame
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        primitive = SolidPrimitive()
        primitive.type = 2  # SPHERE
        primitive.dimensions = [0.05]  # 5cm tolerance
        
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        
        return constraints
    
    def pick_ball(self) -> bool:
        """
        Pick up detected ball
        """
        self.get_logger().info('Picking ball...')
        
        # Move to ball
        if not self.move_to_ball():
            return False
        
        # Close gripper
        if not self.control_gripper(self.gripper_close):
            return False
        
        # Lift ball
        if not self.lift_ball():
            return False
        
        self.get_logger().info('Ball picked successfully')
        self.balls_picked += 1
        return True
    
    def lift_ball(self) -> bool:
        """
        Lift the ball after closing gripper
        """
        # Move up slightly
        lift_height = 0.2  # 20cm
        current_pose = PoseStamped()
        current_pose.header.frame_id = self.base_frame
        current_pose.pose.position = Point(x=0.15, y=0.0, z=1.0)
        
        q = R.from_euler('xyz', [np.pi, 0, 0]).as_quat()
        current_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return self.move_to_pose(current_pose)
    
    def move_to_empty_bowl(self) -> bool:
        """
        Move to empty bowl position (right side, y=-0.10)
        """
        self.get_logger().info('Moving to empty bowl...')
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position above empty bowl
        target_pose.pose.position = Point(x=0.15, y=-0.10, z=1.05)
        
        q = R.from_euler('xyz', [np.pi, 0, 0]).as_quat()
        target_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return self.move_to_pose(target_pose)
    
    def place_ball(self) -> bool:
        """
        Place ball in empty bowl
        """
        self.get_logger().info('Placing ball...')
        
        # Move to empty bowl
        if not self.move_to_empty_bowl():
            return False
        
        # Open gripper
        if not self.control_gripper(self.gripper_open):
            return False
        
        self.get_logger().info('Ball placed successfully')
        return True
    
    def control_gripper(self, position: float) -> bool:
        """
        Control gripper position (mimic joint)
        Uses /right_claw_joint controller
        """
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from rclpy.duration import Duration
        
        try:
            # Create publisher for gripper command
            pub = self.create_publisher(JointTrajectory, '/gripper_controller/commands', 10)
            
            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = ['right_claw_joint']
            
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.velocities = [0.0]
            point.time_from_start = Duration(seconds=2).to_msg()
            
            traj.points.append(point)
            pub.publish(traj)
            
            time.sleep(2.5)  # Wait for gripper to move
            return True
        except Exception as e:
            self.get_logger().error(f'Gripper control error: {e}')
            return False
    
    def return_to_home(self) -> bool:
        """
        Return robot to home position
        """
        self.get_logger().info('Returning to home position...')
        
        home_pose = PoseStamped()
        home_pose.header.frame_id = self.base_frame
        home_pose.header.stamp = self.get_clock().now().to_msg()
        
        home_pose.pose.position = Point(x=0.0, y=0.0, z=0.5)
        q = R.from_euler('xyz', [0, 0, 0]).as_quat()
        home_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return self.move_to_pose(home_pose)
    
    def execute_task(self):
        """
        Main task execution: Pick and place 3 balls
        """
        try:
            self.get_logger().info('Starting pick and place task')
            self.get_logger().info(f'Will pick {self.num_balls} balls')
            
            for i in range(self.num_balls):
                if self.stop_flag:
                    break
                
                self.get_logger().info(f'\n=== Ball {i+1}/{self.num_balls} ===')
                
                # Scan for ball
                if not self.scan_for_ball():
                    self.get_logger().error(f'Failed to detect ball {i+1}')
                    continue
                
                # Pick ball
                if not self.pick_ball():
                    self.get_logger().error(f'Failed to pick ball {i+1}')
                    continue
                
                # Place ball
                if not self.place_ball():
                    self.get_logger().error(f'Failed to place ball {i+1}')
                    continue
                
                time.sleep(1.0)  # Brief pause between balls
            
            # Return to home
            if not self.stop_flag:
                self.return_to_home()
            
            self.get_logger().info(f'Task completed! Picked and placed {self.balls_picked} balls')
            
        except KeyboardInterrupt:
            self.get_logger().info('Task interrupted')
        except Exception as e:
            self.get_logger().error(f'Task failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanningVisionPickPlace()
    
    # Run task in separate thread
    task_thread = threading.Thread(target=node.execute_task)
    task_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_flag = True
    finally:
        task_thread.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
