#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion

# Import optimization algorithm
from whales_optimizer import WhaleOptimizer

class PathPlannerControl:
    """
    Core path planning functionality for robot arm motion planning
    """
    
    def __init__(self):
        """Initialize the path planner control"""
        # Initialize planning parameters
        self.planning_time = 5.0  # Planning time limit (seconds)
        self.num_waypoints = 10  # Number of waypoints in path
        self.collision_threshold = 0.05  # Collision threshold (meters)
        self.smoothness_weight = 0.3  # Weight for path smoothness
        self.obstacle_weight = 0.7  # Weight for obstacle avoidance
        
        # Initialize workspace limits
        self.workspace_limits = {
            'x': (-0.5, 0.5),  # meters
            'y': (-0.5, 0.5),  # meters
            'z': (0.0, 0.8),   # meters
            'roll': (-math.pi, math.pi),  # radians
            'pitch': (-math.pi/2, math.pi/2),  # radians
            'yaw': (-math.pi, math.pi)  # radians
        }
        
        # Initialize obstacles
        self.obstacles = []
        
        # Initialize optimizer
        self.optimizer = WhaleOptimizer()
        
        # Initialize start and target poses
        self.start_pose = None
        self.target_pose = None
    
    def set_planning_parameters(self, planning_time=None, num_waypoints=None, 
                              collision_threshold=None, smoothness_weight=None, 
                              obstacle_weight=None):
        """
        Set planning parameters
        
        Args:
            planning_time: Planning time limit (seconds)
            num_waypoints: Number of waypoints in path
            collision_threshold: Collision threshold (meters)
            smoothness_weight: Weight for path smoothness
            obstacle_weight: Weight for obstacle avoidance
        """
        if planning_time is not None:
            self.planning_time = planning_time
        if num_waypoints is not None:
            self.num_waypoints = num_waypoints
        if collision_threshold is not None:
            self.collision_threshold = collision_threshold
        if smoothness_weight is not None:
            self.smoothness_weight = smoothness_weight
        if obstacle_weight is not None:
            self.obstacle_weight = obstacle_weight
    
    def set_workspace_limits(self, workspace_limits):
        """
        Set workspace limits
        
        Args:
            workspace_limits: Dictionary of workspace limits
        """
        self.workspace_limits = workspace_limits
    
    def set_obstacles(self, obstacles):
        """
        Set obstacles
        
        Args:
            obstacles: List of obstacles, each as [x, y, z, radius]
        """
        self.obstacles = obstacles
    
    def set_start_pose(self, pose):
        """
        Set start pose
        
        Args:
            pose: Start pose
        """
        self.start_pose = pose
    
    def set_target_pose(self, pose):
        """
        Set target pose
        
        Args:
            pose: Target pose
        """
        self.target_pose = pose
    
    def plan_path(self):
        """
        Plan path from start pose to target pose
        
        Returns:
            path: List of poses along the path
            success: Whether planning was successful
        """
        # Check if start and target poses are set
        if self.start_pose is None or self.target_pose is None:
            rospy.logerr("Start or target pose not set")
            return None, False
        
        # Convert poses to position and orientation arrays
        start_pos = np.array([self.start_pose.position.x, self.start_pose.position.y, self.start_pose.position.z])
        start_quat = np.array([self.start_pose.orientation.x, self.start_pose.orientation.y, 
                               self.start_pose.orientation.z, self.start_pose.orientation.w])
        
        target_pos = np.array([self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z])
        target_quat = np.array([self.target_pose.orientation.x, self.target_pose.orientation.y, 
                                self.target_pose.orientation.z, self.target_pose.orientation.w])
        
        # Define optimization problem
        def objective_function(x):
            # Reshape x to get waypoint positions and orientations
            waypoints = x.reshape(self.num_waypoints, 7)  # [x, y, z, qx, qy, qz, qw]
            
            # Initialize cost
            cost = 0.0
            
            # Add smoothness cost
            smoothness_cost = 0.0
            for i in range(1, self.num_waypoints):
                # Position smoothness
                pos_diff = waypoints[i, :3] - waypoints[i-1, :3]
                smoothness_cost += np.linalg.norm(pos_diff)**2
                
                # Orientation smoothness (quaternion distance)
                q1 = waypoints[i-1, 3:7]
                q2 = waypoints[i, 3:7]
                dot_product = np.abs(np.dot(q1, q2))
                smoothness_cost += 1.0 - dot_product
            
            # Add obstacle avoidance cost
            obstacle_cost = 0.0
            for waypoint in waypoints:
                for obstacle in self.obstacles:
                    # Compute distance to obstacle
                    obstacle_pos = obstacle[:3]
                    obstacle_radius = obstacle[3]
                    distance = np.linalg.norm(waypoint[:3] - obstacle_pos) - obstacle_radius
                    
                    # Add cost if too close to obstacle
                    if distance < self.collision_threshold:
                        obstacle_cost += (self.collision_threshold - distance)**2
            
            # Add start and target pose costs
            start_cost = np.linalg.norm(waypoints[0, :3] - start_pos)**2
            start_cost += 10.0 * (1.0 - np.abs(np.dot(waypoints[0, 3:7], start_quat)))
            
            target_cost = np.linalg.norm(waypoints[-1, :3] - target_pos)**2
            target_cost += 10.0 * (1.0 - np.abs(np.dot(waypoints[-1, 3:7], target_quat)))
            
            # Combine costs
            cost = (
                self.smoothness_weight * smoothness_cost +
                self.obstacle_weight * obstacle_cost +
                10.0 * (start_cost + target_cost)  # High weight for start and target
            )
            
            return cost
        
        # Define bounds for optimization
        lower_bounds = np.zeros(self.num_waypoints * 7)
        upper_bounds = np.zeros(self.num_waypoints * 7)
        
        for i in range(self.num_waypoints):
            # Position bounds
            lower_bounds[i*7 + 0] = self.workspace_limits['x'][0]
            lower_bounds[i*7 + 1] = self.workspace_limits['y'][0]
            lower_bounds[i*7 + 2] = self.workspace_limits['z'][0]
            upper_bounds[i*7 + 0] = self.workspace_limits['x'][1]
            upper_bounds[i*7 + 1] = self.workspace_limits['y'][1]
            upper_bounds[i*7 + 2] = self.workspace_limits['z'][1]
            
            # Quaternion bounds (-1 to 1)
            lower_bounds[i*7 + 3:i*7 + 7] = -1.0
            upper_bounds[i*7 + 3:i*7 + 7] = 1.0
        
        # Initialize solution with linear interpolation
        initial_solution = np.zeros(self.num_waypoints * 7)
        for i in range(self.num_waypoints):
            # Linear interpolation for position
            t = i / (self.num_waypoints - 1) if self.num_waypoints > 1 else 0
            pos = (1 - t) * start_pos + t * target_pos
            
            # Spherical linear interpolation for orientation
            quat = self._slerp(start_quat, target_quat, t)
            
            # Set initial solution
            initial_solution[i*7 + 0:i*7 + 3] = pos
            initial_solution[i*7 + 3:i*7 + 7] = quat
        
        # Set optimizer parameters
        self.optimizer.set_parameters(
            num_agents=30,
            max_iterations=100,
            lower_bounds=lower_bounds,
            upper_bounds=upper_bounds,
            objective_function=objective_function
        )
        
        # Run optimization
        try:
            best_solution, best_cost = self.optimizer.optimize(initial_solution)
            
            # Check if solution is valid
            if best_solution is None:
                rospy.logerr("Optimization failed")
                return None, False
            
            # Convert solution to path
            path = []
            waypoints = best_solution.reshape(self.num_waypoints, 7)
            
            for waypoint in waypoints:
                # Extract position and orientation
                position = Point(waypoint[0], waypoint[1], waypoint[2])
                
                # Normalize quaternion
                quat = waypoint[3:7]
                quat_norm = np.linalg.norm(quat)
                if quat_norm > 0:
                    quat = quat / quat_norm
                
                orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
                
                # Create pose
                pose = Pose(position, orientation)
                
                # Add to path
                path.append(pose)
            
            return path, True
            
        except Exception as e:
            rospy.logerr(f"Error planning path: {str(e)}")
            return None, False
    
    def _slerp(self, q1, q2, t):
        """
        Spherical linear interpolation between quaternions
        
        Args:
            q1: First quaternion [x, y, z, w]
            q2: Second quaternion [x, y, z, w]
            t: Interpolation parameter (0 to 1)
            
        Returns:
            q: Interpolated quaternion
        """
        # Compute dot product
        dot = np.dot(q1, q2)
        
        # If dot product is negative, negate one quaternion
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # Clamp dot product to valid range
        dot = min(1.0, max(-1.0, dot))
        
        # Compute angle
        theta = math.acos(dot)
        
        # Check if quaternions are very close
        if abs(theta) < 1e-6:
            return q1
        
        # Compute sin of angle
        sin_theta = math.sin(theta)
        
        # Compute interpolation weights
        w1 = math.sin((1.0 - t) * theta) / sin_theta
        w2 = math.sin(t * theta) / sin_theta
        
        # Interpolate
        q = w1 * q1 + w2 * q2
        
        # Normalize
        q_norm = np.linalg.norm(q)
        if q_norm > 0:
            q = q / q_norm
        
        return q
    
    def check_collision(self, pose):
        """
        Check if pose is in collision with obstacles
        
        Args:
            pose: Pose to check
            
        Returns:
            collision: Whether pose is in collision
        """
        # Extract position
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        # Check collision with each obstacle
        for obstacle in self.obstacles:
            obstacle_pos = obstacle[:3]
            obstacle_radius = obstacle[3]
            
            # Compute distance to obstacle
            distance = np.linalg.norm(position - obstacle_pos) - obstacle_radius
            
            # Check if distance is less than threshold
            if distance < self.collision_threshold:
                return True
        
        return False
    
    def check_path_collision(self, path):
        """
        Check if path is in collision with obstacles
        
        Args:
            path: List of poses
            
        Returns:
            collision: Whether path is in collision
            collision_index: Index of first collision
        """
        for i, pose in enumerate(path):
            if self.check_collision(pose):
                return True, i
        
        return False, -1
    
    def smooth_path(self, path, smoothing_factor=0.5):
        """
        Smooth path using moving average
        
        Args:
            path: List of poses
            smoothing_factor: Smoothing factor (0 to 1)
            
        Returns:
            smoothed_path: Smoothed path
        """
        if path is None or len(path) < 3:
            return path
        
        # Create copy of path
        smoothed_path = [pose for pose in path]
        
        # Smooth positions
        for i in range(1, len(path) - 1):
            # Get positions
            prev_pos = np.array([path[i-1].position.x, path[i-1].position.y, path[i-1].position.z])
            curr_pos = np.array([path[i].position.x, path[i].position.y, path[i].position.z])
            next_pos = np.array([path[i+1].position.x, path[i+1].position.y, path[i+1].position.z])
            
            # Compute smoothed position
            smoothed_pos = (1 - smoothing_factor) * curr_pos + smoothing_factor * 0.5 * (prev_pos + next_pos)
            
            # Update position
            smoothed_path[i].position.x = smoothed_pos[0]
            smoothed_path[i].position.y = smoothed_pos[1]
            smoothed_path[i].position.z = smoothed_pos[2]
            
            # Get orientations
            prev_quat = np.array([path[i-1].orientation.x, path[i-1].orientation.y, 
                                path[i-1].orientation.z, path[i-1].orientation.w])
            curr_quat = np.array([path[i].orientation.x, path[i].orientation.y, 
                                path[i].orientation.z, path[i].orientation.w])
            next_quat = np.array([path[i+1].orientation.x, path[i+1].orientation.y, 
                                path[i+1].orientation.z, path[i+1].orientation.w])
            
            # Compute smoothed orientation using SLERP
            smoothed_quat1 = self._slerp(curr_quat, prev_quat, smoothing_factor * 0.5)
            smoothed_quat2 = self._slerp(curr_quat, next_quat, smoothing_factor * 0.5)
            smoothed_quat = self._slerp(smoothed_quat1, smoothed_quat2, 0.5)
            
            # Update orientation
            smoothed_path[i].orientation.x = smoothed_quat[0]
            smoothed_path[i].orientation.y = smoothed_quat[1]
            smoothed_path[i].orientation.z = smoothed_quat[2]
            smoothed_path[i].orientation.w = smoothed_quat[3]
        
        return smoothed_path 