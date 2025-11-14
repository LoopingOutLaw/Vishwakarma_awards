#!/usr/bin/env python3
"""
Interactive control script for akabot
Provides a simple CLI to control the arm
"""

import rclpy
from rclpy.node import Node
from akabot_control.akabot_controller import AkabotController
import numpy as np


def print_menu():
    print("\n" + "="*50)
    print("Akabot Interactive Control")
    print("="*50)
    print("1. Move to HOME")
    print("2. Move to READY")
    print("3. Move to RIGHT")
    print("4. Move to LEFT")
    print("5. Move to custom XYZ position")
    print("6. Get current pose")
    print("0. Exit")
    print("="*50)


def main():
    rclpy.init()
    
    controller = AkabotController()
    
    # Wait for joint states
    print("Waiting for robot connection...")
    while not controller.joint_state_received:
        rclpy.spin_once(controller, timeout_sec=0.1)
    
    print("\nRobot connected! Ready for commands.")
    
    try:
        while True:
            print_menu()
            choice = input("Enter choice: ").strip()
            
            if choice == '0':
                print("Exiting...")
                break
                
            elif choice == '1':
                print("\nMoving to HOME position...")
                controller.move_to_named_target('home')
                
            elif choice == '2':
                print("\nMoving to READY position...")
                controller.move_to_named_target('ready')
                
            elif choice == '3':
                print("\nMoving to RIGHT position...")
                controller.move_to_named_target('right')
                
            elif choice == '4':
                print("\nMoving to LEFT position...")
                controller.move_to_named_target('left')
                
            elif choice == '5':
                try:
                    x = float(input("Enter X position (m): "))
                    y = float(input("Enter Y position (m): "))
                    z = float(input("Enter Z position (m): "))
                    
                    # Optional orientation
                    use_orient = input("Set orientation? (y/n): ").lower() == 'y'
                    if use_orient:
                        roll = float(input("Enter roll (degrees): ")) * np.pi / 180
                        pitch = float(input("Enter pitch (degrees): ")) * np.pi / 180
                        yaw = float(input("Enter yaw (degrees): ")) * np.pi / 180
                    else:
                        roll, pitch, yaw = 0.0, np.pi/2, 0.0  # Default: pointing down
                    
                    target_pose = controller.create_pose(x, y, z, roll, pitch, yaw)
                    print(f"\nMoving to pose: ({x:.3f}, {y:.3f}, {z:.3f})...")
                    controller.move_to_pose(target_pose, duration=5.0)
                    
                except ValueError:
                    print("Invalid input! Please enter numbers.")
                    
            elif choice == '6':
                if controller.current_joint_state:
                    print("\nCurrent joint positions:")
                    for name, pos in zip(controller.joint_names, 
                                        controller.current_joint_state.position[:5]):
                        print(f"  {name}: {pos:.3f} rad ({pos*180/np.pi:.1f}°)")
                else:
                    print("No joint state available!")
                    
            else:
                print("Invalid choice!")
            
            input("\nPress Enter to continue...")
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()