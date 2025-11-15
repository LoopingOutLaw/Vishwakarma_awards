#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama  # The Ollama Python library
import time

# The custom model you created in Step 1
BRAIN_MODEL = 'my_arm_brain'

# The System Prompt (copied from your modelfile for clarity)
SYSTEM_PROMPT = """
You are a 6-DOF robotic arm controller.
Your goal is to pick up the 'ball' and place it in the 'basket'.
You must respond with ONLY ONE command.

Your available commands are:
- GOTO_ABOVE(X, Y, Z)  # Moves the gripper to a safe spot ABOVE the target
- GOTO_TARGET(X, Y, Z) # Moves the gripper DOWN to the target
- CLOSE_GRIPPER()
- OPEN_GRIPPER()
- GOTO_HOME()
- WAIT()
"""

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self.get_logger().info('Brain Node (Logic) is running...')

        # --- State Variables ---
        self.current_world_state = "ball_at:unknown, basket_at:unknown, gripper:unknown"
        self.current_goal = "WAIT()" # Default goal
        self.current_robot_state = "IDLE" # What the robot is currently doing
        
        # --- Subscribers ---
        # Listens to the "Eyes"
        self.world_sub = self.create_subscription(
            String,
            '/world_state',
            self.world_state_callback,
            10)
        
        # Listens to the "Ears"
        self.command_sub = self.create_subscription(
            String,
            '/user_command',
            self.user_command_callback,
            10)

        # --- Publisher ---
        # Publishes to the "Muscles"
        self.action_pub = self.create_publisher(String, '/arm_command', 10)

        # --- Think Timer ---
        # Create a timer to run the "think" loop once per second
        # This prevents spamming Ollama 30x/sec
        self.think_timer = self.create_timer(1.0, self.think_loop)
        self.get_logger().info('Brain is ready and thinking...')

    def world_state_callback(self, msg):
        # Update our knowledge of the world
        self.current_world_state = msg.data

    def user_command_callback(self, msg):
        # Update our goal
        self.current_goal = msg.data
        self.get_logger().info(f'New Goal Received: "{self.current_goal}"')

    def think_loop(self):
        self.get_logger().info('--- Thinking ---')
        
        # 1. Build the prompt for the AI
        prompt = f"""
        GOAL: "{self.current_goal}"
        
        CURRENT_STATE:
        - {self.current_world_state}
        - Robot is: {self.current_robot_state}
        
        What is your next single command?
        """
        
        messages = [
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': prompt}
        ]
        
        try:
            # 2. Call Ollama (the "Brain")
            response = ollama.chat(model=BRAIN_MODEL, messages=messages, stream=False)
            command_text = response['message']['content'].strip()

            # Clean up the command (remove extra text)
            if '[' in command_text and ']' in command_text:
                command_text = command_text[command_text.find('['):command_text.find(']')+1]
            
            # Simple parser to find the first valid command
            final_command = "WAIT()"
            for line in command_text.split('\n'):
                if line.startswith("GOTO_") or line.startswith("CLOSE_") or line.startswith("OPEN_") or line.startswith("WAIT"):
                    final_command = line
                    break # Only take the first command

            self.get_logger().info(f'AI decision: "{final_command}"')

            # 3. Publish the command to the "Muscles"
            action_msg = String()
            action_msg.data = final_command
            self.action_pub.publish(action_msg)
            
            # Update our internal state
            if final_command != "WAIT()":
                self.current_robot_state = f"EXECUTING: {final_command}"
            else:
                self.current_robot_state = "IDLE"

        except Exception as e:
            self.get_logger().error(f'Failed to query Ollama: {e}')

def main(args=None):
    rclpy.init(args=args)
    brain_node = BrainNode()
    rclpy.spin(brain_node)
    brain_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()