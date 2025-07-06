#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion

class VisualServoControl:
    """
    Core visual servoing functionality for robot arm control
    """
    
    def __init__(self):
        """Initialize the visual servo control"""
        # Initialize control parameters
        self.gain = 0.5  # Control gain
        self.max_velocity = 0.2  # Maximum velocity (m/s or rad/s)
        self.min_velocity = 0.01  # Minimum velocity (m/s or rad/s)
        self.target_tolerance = 0.01  # Target tolerance (m)
        self.orientation_tolerance = 0.05  # Orientation tolerance (rad)
        
        # Initialize feature parameters
        self.feature_depth = 0.5  # Default feature depth (m)
        self.camera_focal_length = 500.0  # Camera focal length (pixels)
        
        # Initialize camera parameters
        self.camera_matrix = np.array([
            [self.camera_focal_length, 0, 320],
            [0, self.camera_focal_length, 240],
            [0, 0, 1]
        ])
        
        # Initialize interaction matrix
        self.interaction_matrix = None
        
        # Initialize current feature points
        self.current_features = None
        
        # Initialize target feature points
        self.target_features = None
        
        # Initialize control status
        self.control_active = False
        self.target_reached = False
        
        # Initialize error
        self.error = None
        self.error_norm = float('inf')
        
        # Initialize velocity
        self.velocity = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
    
    def set_camera_parameters(self, focal_length, principal_point):
        """
        Set camera parameters
        
        Args:
            focal_length: Camera focal length (pixels)
            principal_point: Principal point coordinates (cx, cy)
        """
        cx, cy = principal_point
        
        # Update camera matrix
        self.camera_matrix = np.array([
            [focal_length, 0, cx],
            [0, focal_length, cy],
            [0, 0, 1]
        ])
        
        # Update camera focal length
        self.camera_focal_length = focal_length
    
    def set_control_parameters(self, gain=None, max_velocity=None, min_velocity=None, 
                              target_tolerance=None, orientation_tolerance=None):
        """
        Set control parameters
        
        Args:
            gain: Control gain
            max_velocity: Maximum velocity (m/s or rad/s)
            min_velocity: Minimum velocity (m/s or rad/s)
            target_tolerance: Target tolerance (m)
            orientation_tolerance: Orientation tolerance (rad)
        """
        if gain is not None:
            self.gain = gain
        if max_velocity is not None:
            self.max_velocity = max_velocity
        if min_velocity is not None:
            self.min_velocity = min_velocity
        if target_tolerance is not None:
            self.target_tolerance = target_tolerance
        if orientation_tolerance is not None:
            self.orientation_tolerance = orientation_tolerance
    
    def set_target_features(self, features):
        """
        Set target feature points
        
        Args:
            features: Target feature points [(x1, y1), (x2, y2), ...]
        """
        self.target_features = np.array(features)
    
    def set_current_features(self, features):
        """
        Set current feature points
        
        Args:
            features: Current feature points [(x1, y1), (x2, y2), ...]
        """
        self.current_features = np.array(features)
        
        # Update error
        self._compute_error()
    
    def _compute_error(self):
        """Compute error between current and target features"""
        if self.current_features is None or self.target_features is None:
            self.error = None
            self.error_norm = float('inf')
            return
        
        # Check if feature arrays have the same shape
        if self.current_features.shape != self.target_features.shape:
            rospy.logerr(f"Feature arrays have different shapes: {self.current_features.shape} vs {self.target_features.shape}")
            self.error = None
            self.error_norm = float('inf')
            return
        
        # Compute error
        self.error = self.current_features.flatten() - self.target_features.flatten()
        
        # Compute error norm
        self.error_norm = np.linalg.norm(self.error)
    
    def _compute_interaction_matrix(self):
        """Compute interaction matrix for current features"""
        if self.current_features is None:
            self.interaction_matrix = None
            return
        
        # Number of feature points
        n_points = len(self.current_features)
        
        # Initialize interaction matrix
        self.interaction_matrix = np.zeros((2 * n_points, 6))
        
        # Compute interaction matrix for each feature point
        for i, (x, y) in enumerate(self.current_features):
            # Normalized image coordinates
            x_n = (x - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
            y_n = (y - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]
            
            # Depth
            Z = self.feature_depth
            
            # Compute interaction matrix for this feature point
            L_x = np.array([
                [-1/Z, 0, x_n/Z, x_n*y_n, -(1 + x_n**2), y_n],
                [0, -1/Z, y_n/Z, 1 + y_n**2, -x_n*y_n, -x_n]
            ])
            
            # Add to interaction matrix
            self.interaction_matrix[2*i:2*i+2, :] = L_x
    
    def compute_velocity(self):
        """
        Compute velocity command based on visual servoing
        
        Returns:
            velocity: Velocity command [vx, vy, vz, wx, wy, wz]
            error_norm: Error norm
        """
        # Check if control is active
        if not self.control_active:
            return np.zeros(6), self.error_norm
        
        # Check if target is reached
        if self.target_reached:
            return np.zeros(6), self.error_norm
        
        # Check if features are available
        if self.current_features is None or self.target_features is None:
            return np.zeros(6), float('inf')
        
        # Compute error
        self._compute_error()
        
        # Check if error is small enough
        if self.error_norm < self.target_tolerance:
            self.target_reached = True
            return np.zeros(6), self.error_norm
        
        # Compute interaction matrix
        self._compute_interaction_matrix()
        
        # Check if interaction matrix is available
        if self.interaction_matrix is None:
            return np.zeros(6), self.error_norm
        
        # Compute velocity using visual servoing law
        # v = -lambda * L+ * e
        try:
            # Compute pseudo-inverse of interaction matrix
            L_pinv = np.linalg.pinv(self.interaction_matrix)
            
            # Compute velocity
            self.velocity = -self.gain * np.dot(L_pinv, self.error)
            
            # Apply velocity limits
            velocity_norm = np.linalg.norm(self.velocity)
            if velocity_norm > self.max_velocity:
                self.velocity = self.velocity * self.max_velocity / velocity_norm
            elif velocity_norm < self.min_velocity and velocity_norm > 0:
                self.velocity = self.velocity * self.min_velocity / velocity_norm
            
            return self.velocity, self.error_norm
            
        except np.linalg.LinAlgError as e:
            rospy.logerr(f"Error computing velocity: {str(e)}")
            return np.zeros(6), self.error_norm
    
    def start_control(self):
        """Start visual servoing control"""
        self.control_active = True
        self.target_reached = False
        rospy.loginfo("Visual servoing control started")
    
    def stop_control(self):
        """Stop visual servoing control"""
        self.control_active = False
        rospy.loginfo("Visual servoing control stopped")
    
    def is_target_reached(self):
        """
        Check if target is reached
        
        Returns:
            target_reached: Whether target is reached
        """
        return self.target_reached
    
    def get_error(self):
        """
        Get current error
        
        Returns:
            error: Error between current and target features
            error_norm: Error norm
        """
        return self.error, self.error_norm
    
    def project_point_to_image(self, point_3d, camera_pose):
        """
        Project 3D point to image plane
        
        Args:
            point_3d: 3D point in world coordinates
            camera_pose: Camera pose in world coordinates
            
        Returns:
            point_2d: 2D point in image coordinates
        """
        # Convert point to numpy array
        point_3d = np.array(point_3d)
        
        # Extract camera position and orientation
        camera_position = np.array([
            camera_pose.position.x,
            camera_pose.position.y,
            camera_pose.position.z
        ])
        
        camera_orientation = np.array([
            camera_pose.orientation.x,
            camera_pose.orientation.y,
            camera_pose.orientation.z,
            camera_pose.orientation.w
        ])
        
        # Convert quaternion to rotation matrix
        R = self._quaternion_to_rotation_matrix(camera_orientation)
        
        # Compute camera transformation matrix
        T_camera = np.eye(4)
        T_camera[:3, :3] = R
        T_camera[:3, 3] = camera_position
        
        # Invert camera transformation matrix
        T_camera_inv = np.linalg.inv(T_camera)
        
        # Transform point to camera coordinates
        point_homogeneous = np.append(point_3d, 1)
        point_camera = np.dot(T_camera_inv, point_homogeneous)
        
        # Check if point is in front of camera
        if point_camera[2] <= 0:
            return None
        
        # Project point to image plane
        point_normalized = point_camera[:3] / point_camera[2]
        
        # Apply camera matrix
        point_image = np.dot(self.camera_matrix, point_normalized)
        
        # Return 2D point
        return (point_image[0], point_image[1])
    
    def _quaternion_to_rotation_matrix(self, quaternion):
        """
        Convert quaternion to rotation matrix
        
        Args:
            quaternion: Quaternion [x, y, z, w]
            
        Returns:
            R: Rotation matrix
        """
        # Extract quaternion components
        x, y, z, w = quaternion
        
        # Normalize quaternion
        norm = math.sqrt(x**2 + y**2 + z**2 + w**2)
        if norm > 0:
            x /= norm
            y /= norm
            z /= norm
            w /= norm
        
        # Compute rotation matrix
        R = np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])
        
        return R 

# Add this at the end of the file to properly export the class
__all__ = ['VisualServoControl'] 