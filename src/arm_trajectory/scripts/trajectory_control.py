#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import time
import math
from geometry_msgs.msg import Pose, Point, Quaternion

class TrajectoryControl:
    """
    Core trajectory functionality for robot arm motion planning and kinematics
    """
    
    def __init__(self):
        """Initialize the trajectory control"""
        # Initialize joint state
        self.current_joint_positions = None
        self.current_joint_velocities = None
        self.current_joint_efforts = None
        self.joint_names = None
        
        # Initialize kinematics parameters
        self.dh_params = self._get_default_dh_params()
        
        # Initialize planning parameters
        self.max_velocity = 1.0  # rad/s
        self.max_acceleration = 2.0  # rad/s^2
        self.planning_time = 5.0  # seconds
        self.num_planning_attempts = 10
        
        # Initialize workspace limits
        self.workspace_limits = {
            'x': (-0.5, 0.5),  # meters
            'y': (-0.5, 0.5),  # meters
            'z': (0.0, 0.8),   # meters
            'roll': (-math.pi, math.pi),  # radians
            'pitch': (-math.pi/2, math.pi/2),  # radians
            'yaw': (-math.pi, math.pi)  # radians
        }
        
        # Initialize joint limits
        self.joint_limits = self._get_default_joint_limits()
    
    def _get_default_dh_params(self):
        """
        Get default DH parameters for a 6-DOF robot arm
        
        Returns:
            dh_params: Dictionary of DH parameters
        """
        # Default DH parameters for a 6-DOF robot arm
        # [a, alpha, d, theta_offset]
        return [
            [0, math.pi/2, 0.1, 0],  # Joint 1
            [0.2, 0, 0, 0],         # Joint 2
            [0.2, 0, 0, 0],         # Joint 3
            [0, math.pi/2, 0.1, 0],  # Joint 4
            [0, -math.pi/2, 0.1, 0], # Joint 5
            [0, 0, 0.1, 0]           # Joint 6
        ]
    
    def _get_default_joint_limits(self):
        """
        Get default joint limits
        
        Returns:
            joint_limits: Dictionary of joint limits
        """
        # Default joint limits [min, max] in radians
        return [
            [-math.pi, math.pi],      # Joint 1
            [-math.pi/2, math.pi/2],  # Joint 2
            [-math.pi/2, math.pi/2],  # Joint 3
            [-math.pi, math.pi],      # Joint 4
            [-math.pi/2, math.pi/2],  # Joint 5
            [-math.pi, math.pi]       # Joint 6
        ]
    
    def set_current_joint_state(self, positions, velocities, efforts, joint_names):
        """
        Set current joint state
        
        Args:
            positions: Joint positions
            velocities: Joint velocities
            efforts: Joint efforts
            joint_names: Joint names
        """
        self.current_joint_positions = positions
        self.current_joint_velocities = velocities
        self.current_joint_efforts = efforts
        self.joint_names = joint_names
    
    def compute_forward_kinematics(self, joint_positions):
        """
        Compute forward kinematics
        
        Args:
            joint_positions: Joint positions
            
        Returns:
            pose: End effector pose
        """
        # Check if joint positions are valid
        if not joint_positions or len(joint_positions) < 6:
            raise ValueError("Invalid joint positions")
        
        # Initialize transformation matrix
        T = np.eye(4)
        
        # Compute forward kinematics using DH parameters
        for i in range(min(len(joint_positions), len(self.dh_params))):
            # Get DH parameters
            a, alpha, d, theta_offset = self.dh_params[i]
            
            # Compute joint angle
            theta = joint_positions[i] + theta_offset
            
            # Compute transformation matrix for this joint
            T_i = np.array([
                [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
                [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
                [0, math.sin(alpha), math.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            
            # Update transformation matrix
            T = np.dot(T, T_i)
        
        # Extract position and orientation from transformation matrix
        position = Point(T[0, 3], T[1, 3], T[2, 3])
        
        # Extract rotation matrix
        R = T[:3, :3]
        
        # Convert rotation matrix to quaternion
        # Using the algorithm from https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = math.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        
        # Create quaternion
        orientation = Quaternion(qx, qy, qz, qw)
        
        # Create pose
        pose = Pose(position, orientation)
        
        return pose
    
    def compute_inverse_kinematics(self, pose, seed_joint_positions=None):
        """
        Compute inverse kinematics
        
        Args:
            pose: Target pose
            seed_joint_positions: Seed joint positions for IK solver
            
        Returns:
            joint_positions: Joint positions
        """
        # Check if pose is valid
        if not pose:
            raise ValueError("Invalid pose")
        
        # Use numerical IK solver
        return self._numerical_ik_solver(pose, seed_joint_positions)
    
    def _numerical_ik_solver(self, pose, seed_joint_positions=None):
        """
        Numerical inverse kinematics solver
        
        Args:
            pose: Target pose
            seed_joint_positions: Seed joint positions for IK solver
            
        Returns:
            joint_positions: Joint positions
        """
        # Initialize seed joint positions
        if seed_joint_positions is None:
            if self.current_joint_positions is not None:
                seed_joint_positions = self.current_joint_positions
            else:
                # Default seed positions (all zeros)
                seed_joint_positions = [0.0] * len(self.dh_params)
        
        # Ensure seed joint positions have correct length
        if len(seed_joint_positions) < len(self.dh_params):
            seed_joint_positions = list(seed_joint_positions) + [0.0] * (len(self.dh_params) - len(seed_joint_positions))
        
        # Extract position and orientation from pose
        target_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        target_orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        
        # Normalize quaternion
        quat_norm = np.linalg.norm(target_orientation)
        if quat_norm > 0:
            target_orientation = target_orientation / quat_norm
        
        # Convert quaternion to rotation matrix
        qx, qy, qz, qw = target_orientation
        R_target = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        
        # Initialize joint positions with seed
        joint_positions = np.array(seed_joint_positions[:len(self.dh_params)])
        
        # Initialize parameters for optimization
        max_iterations = 100
        tolerance = 1e-3
        damping = 0.1
        
        # Iterative optimization
        for iteration in range(max_iterations):
            # Compute forward kinematics
            T = np.eye(4)
            for i in range(len(self.dh_params)):
                # Get DH parameters
                a, alpha, d, theta_offset = self.dh_params[i]
                
                # Compute joint angle
                theta = joint_positions[i] + theta_offset
                
                # Compute transformation matrix for this joint
                T_i = np.array([
                    [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
                    [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
                    [0, math.sin(alpha), math.cos(alpha), d],
                    [0, 0, 0, 1]
                ])
                
                # Update transformation matrix
                T = np.dot(T, T_i)
            
            # Extract current position and orientation
            current_position = T[:3, 3]
            current_rotation = T[:3, :3]
            
            # Compute position error
            position_error = target_position - current_position
            
            # Compute orientation error (using rotation matrix difference)
            orientation_error = 0.5 * (np.cross(current_rotation[:, 0], R_target[:, 0]) +
                                       np.cross(current_rotation[:, 1], R_target[:, 1]) +
                                       np.cross(current_rotation[:, 2], R_target[:, 2]))
            
            # Combine errors
            error = np.concatenate([position_error, orientation_error])
            
            # Check if error is small enough
            if np.linalg.norm(error) < tolerance:
                break
            
            # Compute Jacobian
            J = self._compute_jacobian(joint_positions)
            
            # Compute joint updates using damped least squares
            J_inv = np.linalg.pinv(J.T @ J + damping * np.eye(len(self.dh_params))) @ J.T
            delta_theta = J_inv @ error
            
            # Update joint positions
            joint_positions = joint_positions + delta_theta
            
            # Apply joint limits
            for i in range(len(joint_positions)):
                if i < len(self.joint_limits):
                    joint_positions[i] = max(self.joint_limits[i][0], min(self.joint_limits[i][1], joint_positions[i]))
        
        # Check if solution is valid
        T = np.eye(4)
        for i in range(len(self.dh_params)):
            # Get DH parameters
            a, alpha, d, theta_offset = self.dh_params[i]
            
            # Compute joint angle
            theta = joint_positions[i] + theta_offset
            
            # Compute transformation matrix for this joint
            T_i = np.array([
                [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
                [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
                [0, math.sin(alpha), math.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            
            # Update transformation matrix
            T = np.dot(T, T_i)
        
        # Extract final position and orientation
        final_position = T[:3, 3]
        
        # Compute final position error
        position_error = np.linalg.norm(target_position - final_position)
        
        # Check if position error is acceptable
        if position_error > 0.01:  # 1 cm tolerance
            rospy.logwarn(f"IK solution has high position error: {position_error:.3f} m")
        
        return joint_positions.tolist()
    
    def _compute_jacobian(self, joint_positions):
        """
        Compute Jacobian matrix
        
        Args:
            joint_positions: Joint positions
            
        Returns:
            J: Jacobian matrix
        """
        # Initialize Jacobian matrix
        J = np.zeros((6, len(self.dh_params)))
        
        # Compute transformation matrices
        T = np.eye(4)
        T_list = [np.eye(4)]
        
        for i in range(len(self.dh_params)):
            # Get DH parameters
            a, alpha, d, theta_offset = self.dh_params[i]
            
            # Compute joint angle
            theta = joint_positions[i] + theta_offset
            
            # Compute transformation matrix for this joint
            T_i = np.array([
                [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
                [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
                [0, math.sin(alpha), math.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            
            # Update transformation matrix
            T = np.dot(T, T_i)
            T_list.append(T.copy())
        
        # End effector position
        p_end = T_list[-1][:3, 3]
        
        # Compute Jacobian columns
        for i in range(len(self.dh_params)):
            # Z axis of the joint
            z_i = T_list[i][:3, 2]
            
            # Position of the joint
            p_i = T_list[i][:3, 3]
            
            # Linear velocity component
            J_v = np.cross(z_i, p_end - p_i)
            
            # Angular velocity component
            J_w = z_i
            
            # Add to Jacobian
            J[:3, i] = J_v
            J[3:, i] = J_w
        
        return J
    
    def plan_trajectory_to_pose(self, target_pose, start_joint_state=None):
        """
        Plan trajectory to target pose
        
        Args:
            target_pose: Target pose
            start_joint_state: Start joint state
            
        Returns:
            trajectory: List of trajectory points
        """
        # Use current joint state if not provided
        if start_joint_state is None:
            if self.current_joint_positions is None:
                rospy.logwarn("No current joint state available")
                return None
            start_joint_positions = self.current_joint_positions
        else:
            start_joint_positions = start_joint_state.position
        
        # Compute inverse kinematics
        try:
            target_joint_positions = self.compute_inverse_kinematics(target_pose, start_joint_positions)
        except Exception as e:
            rospy.logerr(f"Error computing inverse kinematics: {str(e)}")
            return None
        
        # Plan trajectory between start and target joint positions
        trajectory = self.plan_trajectory(start_joint_positions, target_joint_positions)
        
        return trajectory
    
    def plan_trajectory(self, start_joint_state=None, target_pose=None, planning_time=None, num_planning_attempts=None):
        """
        Plan trajectory
        
        Args:
            start_joint_state: Start joint state
            target_pose: Target pose
            planning_time: Planning time limit
            num_planning_attempts: Number of planning attempts
            
        Returns:
            trajectory: List of trajectory points
        """
        # Set planning parameters
        if planning_time is not None:
            self.planning_time = planning_time
        if num_planning_attempts is not None:
            self.num_planning_attempts = num_planning_attempts
        
        # Use current joint state if not provided
        if start_joint_state is None:
            if self.current_joint_positions is None:
                rospy.logwarn("No current joint state available")
                return None
            start_joint_positions = self.current_joint_positions
        elif hasattr(start_joint_state, 'position'):
            start_joint_positions = start_joint_state.position
        else:
            start_joint_positions = start_joint_state
        
        # Compute inverse kinematics if target pose is provided
        if target_pose is not None:
            try:
                target_joint_positions = self.compute_inverse_kinematics(target_pose, start_joint_positions)
            except Exception as e:
                rospy.logerr(f"Error computing inverse kinematics: {str(e)}")
                return None
        else:
            rospy.logwarn("No target pose provided")
            return None
        
        # Plan trajectory between start and target joint positions
        trajectory = self._plan_joint_trajectory(start_joint_positions, target_joint_positions)
        
        return trajectory
    
    def _plan_joint_trajectory(self, start_joint_positions, target_joint_positions):
        """
        Plan joint trajectory
        
        Args:
            start_joint_positions: Start joint positions
            target_joint_positions: Target joint positions
            
        Returns:
            trajectory: List of trajectory points
        """
        # Check if joint positions are valid
        if not start_joint_positions or not target_joint_positions:
            rospy.logwarn("Invalid joint positions")
            return None
        
        # Ensure joint positions have the same length
        min_length = min(len(start_joint_positions), len(target_joint_positions))
        start_joint_positions = start_joint_positions[:min_length]
        target_joint_positions = target_joint_positions[:min_length]
        
        # Compute trajectory duration based on maximum joint movement
        max_movement = 0
        for i in range(min_length):
            movement = abs(target_joint_positions[i] - start_joint_positions[i])
            max_movement = max(max_movement, movement)
        
        # Compute duration based on maximum velocity
        duration = max_movement / self.max_velocity
        
        # Ensure minimum duration
        duration = max(duration, 0.5)
        
        # Create trajectory points
        trajectory = []
        
        # Number of points in trajectory
        num_points = max(int(duration * 10), 10)  # 10 points per second, minimum 10 points
        
        for i in range(num_points + 1):
            # Compute time from start
            time_from_start = i * duration / num_points
            
            # Compute joint positions using cubic interpolation
            positions = []
            velocities = []
            accelerations = []
            
            for j in range(min_length):
                # Compute normalized time (0 to 1)
                t = i / num_points
                
                # Cubic interpolation
                # p(t) = a*t^3 + b*t^2 + c*t + d
                # p(0) = start, p(1) = target, p'(0) = 0, p'(1) = 0
                # a = 2*start - 2*target
                # b = 3*target - 3*start
                # c = 0
                # d = start
                start = start_joint_positions[j]
                target = target_joint_positions[j]
                
                a = 2 * start - 2 * target
                b = 3 * target - 3 * start
                c = 0
                d = start
                
                # Compute position
                position = a * t**3 + b * t**2 + c * t + d
                
                # Compute velocity
                velocity = 3 * a * t**2 + 2 * b * t + c
                
                # Compute acceleration
                acceleration = 6 * a * t + 2 * b
                
                # Scale velocity and acceleration by duration
                velocity = velocity / duration
                acceleration = acceleration / (duration**2)
                
                # Add to lists
                positions.append(position)
                velocities.append(velocity)
                accelerations.append(acceleration)
            
            # Create trajectory point
            point = {
                'positions': positions,
                'velocities': velocities,
                'accelerations': accelerations,
                'time_from_start': time_from_start
            }
            
            # Add to trajectory
            trajectory.append(point)
        
        return trajectory 