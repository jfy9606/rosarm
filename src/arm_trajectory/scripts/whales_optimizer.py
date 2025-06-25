#!/usr/bin/env python3
import numpy as np
import rospy
from math import pi

class WhaleOptimizer:
    """
    Whale Optimization Algorithm (WOA) implementation for robot arm trajectory planning.
    Based on the whale optimization algorithm for finding optimal joint configurations
    that minimize the error between the target pose and the achieved pose.
    """
    def __init__(self, 
                 num_whales=30,
                 max_iter=100, 
                 forward_kinematics_func=None,
                 dh_params=None,
                 joint_limits=None,
                 b=1.0):
        """
        Initialize the WOA optimizer
        
        Args:
            num_whales: Number of whales (population size)
            max_iter: Maximum iterations 
            forward_kinematics_func: Function to compute forward kinematics (T matrix)
            dh_params: DH parameters of the robot arm
            joint_limits: List of [min, max] for each joint
            b: Spiral constant (default: 1.0)
        """
        self.num_whales = num_whales
        self.max_iter = max_iter
        self.forward_kinematics = forward_kinematics_func
        self.dh_params = dh_params
        self.b = b
        
        # Default joint limits if not provided
        if joint_limits is None:
            # Default limits for a 6-DOF arm: [theta1, d2, theta3, theta4, theta5, d6]
            self.joint_limits = [
                [-pi, pi],      # theta1 (rad)
                [0, 43],       # d2 (cm)
                [-pi/2, pi/2], # theta3 (rad)
                [0, pi],       # theta4 (rad)
                [pi/2, pi/2],  # theta5 (rad, fixed)
                [5, 15]        # d6 (cm)
            ]
        else:
            self.joint_limits = joint_limits
        
        self.dim = len(self.joint_limits)  # Number of joints
        
        # Initialize logging
        rospy.loginfo(f"Initialized WhaleOptimizer with {self.num_whales} whales, {self.max_iter} iterations, b={self.b}")
    
    def random_joint_config(self):
        """Generate a random joint configuration within limits"""
        config = []
        for i in range(self.dim):
            min_val, max_val = self.joint_limits[i]
            config.append(np.random.uniform(min_val, max_val))
        return np.array(config)
    
    def clamp_joints(self, joints):
        """Clamp joint values to stay within limits"""
        for i in range(self.dim):
            min_val, max_val = self.joint_limits[i]
            joints[i] = max(min_val, min(max_val, joints[i]))
        return joints
    
    def transformation_error(self, T1, T2):
        """
        Compute error between two transformation matrices
        Args:
            T1, T2: 4x4 homogeneous transformation matrices
        Returns:
            Combined position and orientation error
        """
        # Position error - Euclidean distance
        pos_error = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
        
        # Orientation error - using rotation matrix difference
        R1 = T1[:3, :3]
        R2 = T2[:3, :3]
        orientation_error = np.linalg.norm(R1 - R2, 'fro') / np.sqrt(6)
        
        # Weight position vs orientation (can be adjusted)
        position_weight = 0.7
        orientation_weight = 0.3
        
        return position_weight * pos_error + orientation_weight * orientation_error
    
    def fitness_function(self, joints, target_T):
        """
        Calculate fitness based on transformation error between current and target pose
        
        Args:
            joints: Joint configuration to evaluate
            target_T: Target transformation matrix (4x4)
        
        Returns:
            fitness: Lower is better
        """
        # Get transformation matrix for these joints
        current_T = self.forward_kinematics(joints)
        
        # Calculate error between transformations
        error = self.transformation_error(current_T, target_T)
        
        return error
    
    def optimize(self, target_T, initial_joints=None):
        """
        Run WOA optimization to find joint configuration that reaches target pose
        
        Args:
            target_T: Target transformation matrix (4x4)
            initial_joints: Optional initial joint configuration (can be None)
        
        Returns:
            best_joints: Best joint configuration found
            best_fitness: Best fitness achieved
            fitness_history: List of best fitness values through iterations
        """
        # Initialize whale population
        whales = np.zeros((self.num_whales, self.dim))
        
        # If initial joints are provided, make one whale start from there
        if initial_joints is not None:
            whales[0] = initial_joints
            for i in range(1, self.num_whales):
                whales[i] = self.random_joint_config()
        else:
            for i in range(self.num_whales):
                whales[i] = self.random_joint_config()
        
        # Initialize best whale position
        fitness = np.zeros(self.num_whales)
        for i in range(self.num_whales):
            fitness[i] = self.fitness_function(whales[i], target_T)
        
        best_idx = np.argmin(fitness)
        best_whale = whales[best_idx].copy()
        best_fitness = fitness[best_idx]
        
        # Initialize tracking variables
        fitness_history = []
        
        # Main WOA loop
        for iteration in range(self.max_iter):
            # WOA parameters
            a = 2 - iteration * (2 / self.max_iter)  # linearly decreased from 2 to 0
            a2 = -1 + iteration * (-1 / self.max_iter)  # linearly decreased from -1 to -2
            
            # Update each whale position
            for i in range(self.num_whales):
                # Random values for the algorithm
                r1 = np.random.rand()
                r2 = np.random.rand()
                A = 2 * a * r1 - a
                C = 2 * r2
                l = np.random.uniform(-1, 1)
                p = np.random.rand()
                
                # Update position based on WOA algorithm choice
                if p < 0.5:
                    # Encircling prey or searching based on |A|
                    if abs(A) < 1:
                        # Encircling prey
                        D = abs(C * best_whale - whales[i])
                        whales[i] = best_whale - A * D
                    else:
                        # Searching for prey (global search)
                        random_idx = np.random.randint(0, self.num_whales)
                        X_rand = whales[random_idx]
                        D = abs(C * X_rand - whales[i])
                        whales[i] = X_rand - A * D
                else:
                    # Bubble-net attacking (exploitation phase)
                    D = abs(best_whale - whales[i])
                    # Spiral updating position
                    whales[i] = D * np.exp(a2 * l) * np.cos(2 * np.pi * l) + best_whale
                
                # Clamp joints to limits
                whales[i] = self.clamp_joints(whales[i])
                
                # Update fitness
                fitness[i] = self.fitness_function(whales[i], target_T)
            
            # Update best whale
            new_best_idx = np.argmin(fitness)
            if fitness[new_best_idx] < best_fitness:
                best_whale = whales[new_best_idx].copy()
                best_fitness = fitness[new_best_idx]
            
            # Store best fitness for this iteration
            fitness_history.append(best_fitness)
            
            # Log progress at intervals
            if iteration % 10 == 0 or iteration == self.max_iter - 1:
                rospy.loginfo(f"Iteration {iteration}: Best fitness = {best_fitness}")
        
        return best_whale, best_fitness, fitness_history

# Example forward kinematics function based on DH parameters
def compute_dh_transform(theta, d, a, alpha):
    """Compute the DH transformation matrix for a single joint"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_dh(joint_values, dh_params):
    """
    Compute forward kinematics using DH parameters
    
    Args:
        joint_values: List of joint values [θ1, d2, θ3, θ4, θ5, d6]
        dh_params: List of DH parameters [(d, θ, a, α), ...] where joint values replace the appropriate parameter
    
    Returns:
        T: End-effector transformation matrix (4x4)
    """
    T = np.eye(4)
    
    for i, (joint_type, d, theta, a, alpha) in enumerate(dh_params):
        if joint_type == "revolute":
            # θ is variable
            theta = joint_values[i]
        elif joint_type == "prismatic":
            # d is variable
            d = joint_values[i]
            
        # Compute transformation matrix for this joint
        T_i = compute_dh_transform(theta, d, a, alpha)
        
        # Update the total transformation
        T = T @ T_i
        
    return T 