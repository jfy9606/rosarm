#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
SCServo Robot Arm Control Demo
A demonstration of controlling a robot arm with SCServo servos and DC motors
"""

import os
import sys
import time
import argparse
import platform
from src.robot_arm import RobotArm


def detect_serial_ports():
    """
    Detect available serial ports on the system
    
    Returns:
        List of available serial ports
    """
    import serial.tools.list_ports
    
    ports = list(serial.tools.list_ports.comports())
    
    available_ports = []
    for p in ports:
        available_ports.append(p.device)
    
    return available_ports


def demo_basic_movement(robot, args):
    """
    Demonstrate basic movement of the robot arm
    """
    print("Calibrating robot arm...")
    robot.calibrate()
    time.sleep(1)
    
    print("Moving joints sequentially...")
    # Move each joint individually
    robot.set_speeds(servo_speed=500)  # Slower speed for demo
    
    print("Moving joint 1...")
    robot.set_joint_position('joint1', 1500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint1', 2500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint1', 2048, blocking=True)  # Center position
    
    print("Moving joint 2...")
    robot.set_joint_position('joint2', 1500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint2', 2500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint2', 2048, blocking=True)
    
    print("Moving joint 3...")
    robot.set_joint_position('joint3', 1500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint3', 2500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint3', 2048, blocking=True)
    
    print("Moving joint 4...")
    robot.set_joint_position('joint4', 1500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint4', 2500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint4', 2048, blocking=True)
    
    print("Moving all joints simultaneously...")
    positions = {
        'joint1': 1500,
        'joint2': 2500,
        'joint3': 1500,
        'joint4': 2500
    }
    robot.set_joint_positions(positions, blocking=True)
    time.sleep(1)
    
    positions = {
        'joint1': 2500,
        'joint2': 1500,
        'joint3': 2500,
        'joint4': 1500
    }
    robot.set_joint_positions(positions, blocking=True)
    time.sleep(1)
    
    # Return to home position
    print("Returning to home position...")
    robot.home(blocking=True)


def demo_dc_motors(robot, args):
    """
    Demonstrate DC motor movement (pitch and linear feed)
    """
    print("Testing pitch motor...")
    robot.set_speeds(pitch_speed=100)
    
    print("Moving pitch up...")
    robot.set_pitch_position(1000, blocking=True)
    time.sleep(1)
    
    print("Moving pitch down...")
    robot.set_pitch_position(-1000, blocking=True)
    time.sleep(1)
    
    print("Returning pitch to center...")
    robot.set_pitch_position(0, blocking=True)
    time.sleep(1)
    
    print("Testing linear feed...")
    robot.set_speeds(linear_speed=100)
    
    print("Extending linear feed...")
    robot.set_linear_position(5000, blocking=True)
    time.sleep(1)
    
    print("Retracting linear feed...")
    robot.set_linear_position(0, blocking=True)
    time.sleep(1)


def demo_cartesian_movement(robot, args):
    """
    Demonstrate cartesian movement
    """
    print("Demonstrating cartesian movement...")
    robot.set_speeds(servo_speed=500, pitch_speed=100, linear_speed=100)
    
    # Move in a square pattern
    print("Moving in a square pattern...")
    robot.move_cartesian(10, 10, 10)
    time.sleep(1)
    
    robot.move_cartesian(10, 20, 10)
    time.sleep(1)
    
    robot.move_cartesian(-10, 20, 10)
    time.sleep(1)
    
    robot.move_cartesian(-10, 10, 10)
    time.sleep(1)
    
    robot.move_cartesian(10, 10, 10)
    time.sleep(1)
    
    # Move in a vertical pattern
    print("Moving in a vertical pattern...")
    robot.move_cartesian(0, 15, -10)
    time.sleep(1)
    
    robot.move_cartesian(0, 15, 10)
    time.sleep(1)
    
    robot.move_cartesian(0, 15, 0)
    time.sleep(1)
    
    # Return to home
    print("Returning to home position...")
    robot.home(blocking=True)


def demo_pick_and_place(robot, args):
    """
    Demonstrate a simple pick and place operation
    """
    print("Performing pick and place demo...")
    robot.set_speeds(servo_speed=500, pitch_speed=100, linear_speed=100)
    
    # Move to pick position
    print("Moving to pick position...")
    robot.set_joint_position('joint1', 1500, blocking=True)  # Base rotation
    robot.set_joint_position('joint2', 1700, blocking=True)  # Shoulder
    robot.set_joint_position('joint3', 2300, blocking=True)  # Elbow
    robot.set_joint_position('joint4', 2048, blocking=True)  # Wrist
    
    # Lower to pick
    print("Lowering to pick...")
    robot.set_pitch_position(-1000, blocking=True)
    
    # Simulate grabbing (in a real robot, this would activate a gripper)
    print("Grabbing object...")
    time.sleep(1)
    
    # Raise after picking
    print("Raising after pick...")
    robot.set_pitch_position(0, blocking=True)
    
    # Move to place position
    print("Moving to place position...")
    robot.set_joint_position('joint1', 2500, blocking=True)  # Rotate base
    
    # Lower to place
    print("Lowering to place...")
    robot.set_pitch_position(-800, blocking=True)
    
    # Simulate releasing (in a real robot, this would release the gripper)
    print("Releasing object...")
    time.sleep(1)
    
    # Raise after placing
    print("Raising after place...")
    robot.set_pitch_position(0, blocking=True)
    
    # Return to home position
    print("Returning to home position...")
    robot.home(blocking=True)


def main():
    """
    Main function for the demo
    """
    parser = argparse.ArgumentParser(description='SCServo Robot Arm Control Demo')
    parser.add_argument('--servo-port', dest='servo_port', help='Serial port for servo control')
    parser.add_argument('--motor-port', dest='motor_port', help='Serial port for motor control')
    parser.add_argument('--servo-baudrate', dest='servo_baudrate', type=int, default=1000000,
                        help='Baudrate for servo communication')
    parser.add_argument('--motor-baudrate', dest='motor_baudrate', type=int, default=115200,
                        help='Baudrate for motor communication')
    parser.add_argument('--demo', dest='demo', default='all',
                        choices=['all', 'basic', 'dc', 'cartesian', 'pick'],
                        help='Type of demo to run')
    
    args = parser.parse_args()
    
    # Detect available ports if not specified
    if not args.servo_port or not args.motor_port:
        print("Detecting available serial ports...")
        available_ports = detect_serial_ports()
        
        if not available_ports:
            print("No serial ports detected. Please specify ports manually.")
            sys.exit(1)
        
        print("Available ports:")
        for i, port in enumerate(available_ports):
            print(f"  {i}: {port}")
        
        if not args.servo_port:
            servo_idx = int(input("Select servo port index: "))
            args.servo_port = available_ports[servo_idx]
        
        if not args.motor_port:
            motor_idx = int(input("Select motor port index: "))
            args.motor_port = available_ports[motor_idx]
    
    print(f"Using servo port: {args.servo_port} at {args.servo_baudrate} baud")
    print(f"Using motor port: {args.motor_port} at {args.motor_baudrate} baud")
    
    # Initialize robot arm
    robot = RobotArm()
    
    # Connect to controllers
    print("Connecting to controllers...")
    if not robot.connect(args.servo_port, args.motor_port, 
                       servo_baudrate=args.servo_baudrate, 
                       motor_baudrate=args.motor_baudrate):
        print("Failed to connect to controllers. Please check connections and try again.")
        sys.exit(1)
    
    print("Connection successful!")
    
    try:
        # Enable torque on all servos
        print("Enabling torque on all servos...")
        robot.enable_torque(True)
        
        # Run selected demo
        if args.demo == 'all' or args.demo == 'basic':
            demo_basic_movement(robot, args)
        
        if args.demo == 'all' or args.demo == 'dc':
            demo_dc_motors(robot, args)
        
        if args.demo == 'all' or args.demo == 'cartesian':
            demo_cartesian_movement(robot, args)
        
        if args.demo == 'all' or args.demo == 'pick':
            demo_pick_and_place(robot, args)
        
        print("Demo completed successfully!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user.")
    except Exception as e:
        print(f"Error during demo: {e}")
    finally:
        # Clean up
        print("Stopping all motors...")
        robot.stop_all()
        
        print("Disabling torque...")
        robot.enable_torque(False)
        
        print("Disconnecting...")
        robot.disconnect()
        
        print("Done!")


if __name__ == "__main__":
    main() 