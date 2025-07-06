#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion

class KinematicsControl:
    """
    Core kinematics functionality for robot arm forward and inverse kinematics
    """
    
    def __init__(self):
        """Initialize the kinematics control"""
        # Initialize kinematics parameters
        self.dh_params = self._get_default_dh_params()
        
        # Initialize joint limits
        self.joint_limits = self._get_default_joint_limits()
    
    def _get_default_dh_params(self):
        """
        Get default DH parameters for a 6-DOF robot arm
        
        Returns:
            dh_params: List of DH parameters [a, alpha, d, theta_offset]
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
            joint_limits: List of joint limits [min, max]
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
    
    def set_dh_params(self, dh_params):
        """
        Set DH parameters
        
        Args:
            dh_params: List of DH parameters [a, alpha, d, theta_offset]
        """
        self.dh_params = dh_params
    
    def set_joint_limits(self, joint_limits):
        """
        Set joint limits
        
        Args:
            joint_limits: List of joint limits [min, max]
        """
        self.joint_limits = joint_limits
    
    def compute_transform(self, a, alpha, d, theta):
        """
        Compute transformation matrix from DH parameters
        
        Args:
            a: Link length
            alpha: Link twist
            d: Link offset
            theta: Joint angle
            
        Returns:
            T: Transformation matrix
        """
        # Compute transformation matrix
        T = np.array([
            [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
            [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        
        return T
    
    def forward_kinematics(self, joint_positions):
        """
        Compute forward kinematics
        
        Args:
            joint_positions: Joint positions
            
        Returns:
            pose: End effector pose
        """
        # Check if joint positions are valid
        if not joint_positions or len(joint_positions) < len(self.dh_params):
            raise ValueError(f"Invalid joint positions: expected {len(self.dh_params)} values, got {len(joint_positions) if joint_positions else 0}")
        
        # Initialize transformation matrix
        T = np.eye(4)
        
        # Compute forward kinematics using DH parameters
        for i in range(len(self.dh_params)):
            # Get DH parameters
            a, alpha, d, theta_offset = self.dh_params[i]
            
            # Compute joint angle
            theta = joint_positions[i] + theta_offset
            
            # Compute transformation matrix for this joint
            T_i = self.compute_transform(a, alpha, d, theta)
            
            # Update transformation matrix
            T = np.dot(T, T_i)
        
        # Extract position from transformation matrix
        position = Point(T[0, 3], T[1, 3], T[2, 3])
        
        # Extract rotation matrix
        R = T[:3, :3]
        
        # Convert rotation matrix to quaternion
        orientation = self.rotation_matrix_to_quaternion(R)
        
        # Create pose
        pose = Pose(position, orientation)
        
        return pose
    
    def rotation_matrix_to_quaternion(self, R):
        """
        Convert rotation matrix to quaternion
        
        Args:
            R: Rotation matrix
            
        Returns:
            quaternion: Quaternion
        """
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
        quaternion = Quaternion(qx, qy, qz, qw)
        
        return quaternion
    
    def quaternion_to_rotation_matrix(self, quaternion):
        """
        Convert quaternion to rotation matrix
        
        Args:
            quaternion: Quaternion
            
        Returns:
            R: Rotation matrix
        """
        # Extract quaternion components
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        
        # Normalize quaternion
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if norm > 0:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm
        
        # Compute rotation matrix
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        
        return R
    
    def inverse_kinematics(self, pose, seed_joint_positions=None):
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
            # Default seed positions (all zeros)
            seed_joint_positions = [0.0] * len(self.dh_params)
        
        # Ensure seed joint positions have correct length
        if len(seed_joint_positions) < len(self.dh_params):
            seed_joint_positions = list(seed_joint_positions) + [0.0] * (len(self.dh_params) - len(seed_joint_positions))
        
        # Extract position and orientation from pose
        target_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        # Convert quaternion to rotation matrix
        R_target = self.quaternion_to_rotation_matrix(pose.orientation)
        
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
                T_i = self.compute_transform(a, alpha, d, theta)
                
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
            T_i = self.compute_transform(a, alpha, d, theta)
            
            # Update transformation matrix
            T = np.dot(T, T_i)
        
        # Extract final position
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
            T_i = self.compute_transform(a, alpha, d, theta)
            
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
    
    def check_joint_limits(self, joint_positions):
        """
        Check if joint positions are within limits
        
        Args:
            joint_positions: Joint positions
            
        Returns:
            valid: Whether joint positions are valid
            violations: List of joint limit violations
        """
        # Check if joint positions are valid
        if not joint_positions:
            return False, ["No joint positions provided"]
        
        # Check each joint
        valid = True
        violations = []
        
        for i, position in enumerate(joint_positions):
            if i < len(self.joint_limits):
                min_limit, max_limit = self.joint_limits[i]
                
                if position < min_limit:
                    valid = False
                    violations.append(f"Joint {i} below limit: {position:.3f} < {min_limit:.3f}")
                elif position > max_limit:
                    valid = False
                    violations.append(f"Joint {i} above limit: {position:.3f} > {max_limit:.3f}")
        
        return valid, violations 