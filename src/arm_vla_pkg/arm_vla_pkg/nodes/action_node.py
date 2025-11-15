#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ikpy.chain import Chain
from ikpy.link import URDFLink
# import serial  <-- REMOVED
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

# --- (1) CONFIGURE YOUR ARM ---
# This part is correct and stays the same.
ARM_CHAIN = Chain.from_links(
    [
        URDFLink(
            name="base_link",
            origin_translation=[0, 0, 0.0],  # From base to joint 1
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1], # Z-axis rotation
        ),
        URDFLink(
            name="link1",
            origin_translation=[0, 0, 0.1],  # From joint 1 to joint 2
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0], # Y-axis rotation
        ),
        URDFLink(
            name="link2",
            origin_translation=[0.15, 0, 0], # From joint 2 to joint 3
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0], # Y-axis rotation
        ),
        URDFLink(
            name="link3",
            origin_translation=[0.15, 0, 0], # From joint 3 to joint 4
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1], # Z-axis rotation
        ),
        URDFLink(
            name="link4",
            origin_translation=[0, 0, 0.1], # From joint 4 to joint 5
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0], # Y-axis rotation
        ),
        URDFLink(
            name="gripper",
            origin_translation=[0.05, 0, 0], # From joint 5 to gripper
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0], # No rotation (end effector)
        )
    ],
    active_links_mask=[False, True, True, True, True, True, False] # Base and gripper are not active
)

# --- REMOVED SERIAL PORT ---
# -------------------------------------------------

class ActionNode(Node):
    def __init__(self):
        super().__init__('action_node')
        self.get_logger().info('Action Node (Muscles) for GAZEBO is running...')
        
        # --- REMOVED SERIAL CONNECTION ---
        # try:
        #     self.arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        #     ...
        # except Exception as e:
        #     self.arduino = None
        # ---------------------------------

        # --- ADDED GAZEBO PUBLISHER ---
        # This topic name MUST match your Gazebo setup from cobot_description
        # [cite: src/cobot_description/urdf/cobot.ros2_control.xacro]
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory',  # <-- VERIFY THIS TOPIC NAME
            10)
        
        # These names MUST match your URDF file
        # [cite: src/cobot_description/urdf/cobot_core.xacro]
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
        ]
        # ---------------------------------

        # Subscribe to the "/arm_command" topic (This is correct)
        self.subscription = self.create_subscription(
            String,
            '/arm_command',
            self.command_callback,
            10)
        
        self.home_position_angles = [0] * 7  # 7 links including base

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')
        
        # This parser logic is correct
        if command.startswith("GOTO_ABOVE"):
            coords = self.parse_coords(command)
            if coords:
                self.move_arm(coords)
                
        elif command.startswith("GOTO_TARGET"):
            coords = self.parse_coords(command)
            if coords:
                self.move_arm(coords)

        elif command == "CLOSE_GRIPPER":
            self.control_gripper("CLOSE")

        elif command == "OPEN_GRIPPER":
            self.control_gripper("OPEN")

        elif command == "GOTO_HOME":
            # We send radians (0 is 0 in both)
            self.send_angles_to_arm(self.home_position_angles)
            
        elif command == "WAIT":
            pass # Do nothing
        
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def parse_coords(self, command_str):
        # This function is correct
        try:
            coord_str = command_str[command_str.find('(') + 1 : command_str.find(')')]
            parts = coord_str.split(',')
            x = float(parts[0]) / 100.0 # Convert cm (from AI) to meters (for ikpy)
            y = float(parts[1]) / 100.0
            z = float(parts[2]) / 100.0
            self.get_logger().info(f'Parsed coords (meters): {[x, y, z]}')
            return [x, y, z]
        except Exception as e:
            self.get_logger().error(f'Failed to parse coords: {e}')
            return None

    def move_arm(self, target_position):
        try:
            # Calculate Inverse Kinematics (this gives radians)
            target_angles_rad = ARM_CHAIN.inverse_kinematics(target_position, initial_position=self.home_position_angles)
            
            # --- REMOVED DEGREE CONVERSION ---
            
            self.get_logger().info(f'Calculated angles (rad): {target_angles_rad[1:7]}')
            
            # --- SEND RADIANS ---
            self.send_angles_to_arm(target_angles_rad)
            
        except Exception as e:
            self.get_logger().error(f'IK calculation failed: {e}')

    def send_angles_to_arm(self, angles_rad):
        # --- REMOVED ALL ARDUINO/SERIAL LOGIC ---
        
        # --- ADDED NEW GAZEBO LOGIC ---
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        # Assign the 6 motor angles from ikpy (must be in radians)
        point.positions = [
            float(angles_rad[1]), # Angle 1
            float(angles_rad[2]), # Angle 2
            float(angles_rad[3]), # Angle 3
            float(angles_rad[4]), # Angle 4
            float(angles_rad[5]), # Angle 5
            float(angles_rad[6])  # Angle 6
        ]
        # Tell Gazebo to reach this position in 1.0 seconds
        point.time_from_start = Duration(sec=1, nanosec=0) 
        
        msg.points.append(point)
        self.trajectory_pub.publish(msg)
        self.get_logger().info('Published new Joint Trajectory to Gazebo.')
        # ---------------------------------
        
    def control_gripper(self, action):
        # --- REMOVED ARDUINO LOGIC ---
        
        # --- YOU MUST ADD GAZEBO GRIPPER LOGIC HERE ---
        # This is a TO-DO. You need to create a *new* publisher
        # for your gripper controller (e.g., /gripper_controller/joint_trajectory)
        self.get_logger().warn(f'Gripper control ({action}) is not implemented for Gazebo yet.')
        # --------------------------------

def main(args=None):
    rclpy.init(args=args)
    action_node = ActionNode()
    rclpy.spin(action_node)
    action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()