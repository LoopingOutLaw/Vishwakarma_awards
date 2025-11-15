#!/usr/bin/env python3
"""
Test script to demonstrate various poses
"""

import rclpy
from akabot_control.akabot_controller import AkabotController
import numpy as np
import time


def main():
    rclpy.init()
    
    controller = AkabotController()
    
    # Wait for joint states
    print("Waiting for robot...")
    while not controller.joint_state_received:
        rclpy.spin_once(controller, timeout_sec=0.1)
    
    print("Starting pose sequence test...\n")
    
    # Test 1: Move to home
    print("="*50)
    print("Test 1: Moving to HOME position")
    print("="*50)
    controller.move_to_named_target('home')
    time.sleep(2)
    
    # Test 2: Move to ready
    print("\n" + "="*50)
    print("Test 2: Moving to READY position")
    print("="*50)
    controller.move_to_named_target('ready')
    time.sleep(2)
    
    # Test 3: Custom poses
    test_poses = [
        {"name": "Forward Center", "x": 0.15, "y": 0.0, "z": 0.20},
        {"name": "Forward Left", "x": 0.12, "y": 0.08, "z": 0.18},
        {"name": "Forward Right", "x": 0.12, "y": -0.08, "z": 0.18},
    ]
    
    for i, pose_data in enumerate(test_poses, 3):
        print(f"\n{'='*50}")
        print(f"Test {i}: Moving to {pose_data['name']}")
        print(f"Position: ({pose_data['x']}, {pose_data['y']}, {pose_data['z']})")
        print("="*50)
        
        target_pose = controller.create_pose(
            x=pose_data['x'],
            y=pose_data['y'],
            z=pose_data['z'],
            roll=0.0,
            pitch=np.pi/2,  # Pointing down
            yaw=0.0
        )
        
        controller.move_to_pose(target_pose, duration=4.0)
        time.sleep(2)
    
    # Return to home
    print("\n" + "="*50)
    print("Returning to HOME position")
    print("="*50)
    controller.move_to_named_target('home')
    
    print("\n" + "="*50)
    print("Test sequence complete!")
    print("="*50)
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()