#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ikpy.chain import Chain
from ikpy.link import URDFLink
import serial
import time

# --- (1) CONFIGURE YOUR ARM & SERIAL PORT HERE ---

# !!! IMPORTANT !!!
# You MUST measure your 6 arm links (in meters) and update this.
# This example is for a simple 6-DOF arm.
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

# Set your Pi's serial port (matches mind.py and MEGAlisten.ino)
# Find this by running 'ls /dev/tty*'
SERIAL_PORT = '/dev/ttyACM0' #
BAUD_RATE = 9600

# -------------------------------------------------

class ActionNode(Node):
    def __init__(self):
        super().__init__('action_node')
        self.get_logger().info('Action Node (Muscles) is running...')
        
        # Initialize Serial connection to the arm's controller
        try:
            self.arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {SERIAL_PORT}')
        except Exception as e:
            self.get_logger().error(f'FAILED TO CONNECT TO ARDUINO: {e}')
            self.arduino = None

        # Subscribe to the "/arm_command" topic
        self.subscription = self.create_subscription(
            String,
            '/arm_command',
            self.command_callback,
            10)
        
        # Define target for IK
        self.home_position_angles = [0] * 7  # 7 links including base

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')
        
        # This is the "Parser"
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
            self.send_angles_to_arm(self.home_position_angles)
            
        elif command == "WAIT":
            pass # Do nothing
        
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def parse_coords(self, command_str):
        # Parses "GOTO_...(X, Y, Z)" to [X, Y, Z]
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
            # Calculate Inverse Kinematics
            target_angles_rad = ARM_CHAIN.inverse_kinematics(target_position)
            
            # Convert radians to degrees (or whatever your arm needs)
            target_angles_deg = list(map(lambda r: round(r * 180 / 3.14159), target_angles_rad))
            
            self.get_logger().info(f'Calculated angles (deg): {target_angles_deg[1:7]}') # Show the 6 motor angles
            self.send_angles_to_arm(target_angles_deg)
            
        except Exception as e:
            self.get_logger().error(f'IK calculation failed: {e}')

    def send_angles_to_arm(self, angles_deg):
        if not self.arduino:
            self.get_logger().warn('Arm is not connected. Skipping movement.')
            return
            
        # Format: "<angle1,angle2,angle3,angle4,angle5,angle6>\n"
        # You MUST update this to match what your 'MEGAlisten.ino' expects
        # This format is just an example
        angle_str = "<{},{},{},{},{},{}>\n".format(
            int(angles_deg[1]), # Angle for motor 1
            int(angles_deg[2]), # Angle for motor 2
            int(angles_deg[3]), # Angle for motor 3
            int(angles_deg[4]), # Angle for motor 4
            int(angles_deg[5]), # Angle for motor 5
            int(angles_deg[6])  # Angle for motor 6
        )
        
        self.get_logger().info(f'Sending to arm: {angle_str}')
        self.arduino.write(angle_str.encode('utf-8'))
        time.sleep(0.1) # Wait for Arduino to process
        
    def control_gripper(self, action):
        if not self.arduino:
            self.get_logger().warn('Arm is not connected. Skipping gripper.')
            return
            
        # Example: send 'G' for close, 'O' for open
        # Update this to match your Arduino code
        if action == "CLOSE":
            self.get_logger().info('Sending to arm: CLOSE_GRIPPER')
            self.arduino.write(b'G\n')
        elif action == "OPEN":
            self.get_logger().info('Sending to arm: OPEN_GRIPPER')
            self.arduino.write(b'O\n')
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    action_node = ActionNode()
    rclpy.spin(action_node)
    action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()