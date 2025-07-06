#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from arm_trajectory.srv import ForwardKinematics, ForwardKinematicsResponse
from arm_trajectory.srv import InverseKinematics, InverseKinematicsResponse

# 使用绝对导入路径
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from kinematics_control import KinematicsControl

class KinematicsNode:
    """
    ROS node for robot arm kinematics
    """
    
    def __init__(self):
        """Initialize the kinematics node"""
        rospy.init_node('kinematics_node', anonymous=True)
        
        # Initialize parameters
        self.dh_params_file = rospy.get_param('~dh_params_file', '')
        
        # Initialize kinematics control
        self.kinematics_control = KinematicsControl()
        
        # Load DH parameters from file if provided
        if self.dh_params_file:
            self._load_dh_params(self.dh_params_file)
        
        # Create services
        self.fk_service = rospy.Service('/kinematics/forward', ForwardKinematics, self.forward_kinematics_callback)
        self.ik_service = rospy.Service('/kinematics/inverse', InverseKinematics, self.inverse_kinematics_callback)
        
        # Subscribe to joint state topic
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Create publishers
        self.pose_pub = rospy.Publisher('/kinematics/pose', Pose, queue_size=10)
        
        # Initialize joint state
        self.current_joint_state = None
        
        rospy.loginfo(f"Kinematics node initialized")
    
    def _load_dh_params(self, file_path):
        """
        Load DH parameters from file
        
        Args:
            file_path: Path to DH parameters file
        """
        try:
            # Load DH parameters from file
            dh_params = []
            
            with open(file_path, 'r') as f:
                for line in f:
                    # Skip comments and empty lines
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    # Parse DH parameters
                    params = [float(x) for x in line.split()]
                    
                    # Check if we have 4 parameters
                    if len(params) == 4:
                        dh_params.append(params)
            
            # Set DH parameters
            if dh_params:
                self.kinematics_control.set_dh_params(dh_params)
                rospy.loginfo(f"Loaded {len(dh_params)} DH parameters from {file_path}")
            else:
                rospy.logwarn(f"No DH parameters found in {file_path}")
                
        except Exception as e:
            rospy.logerr(f"Error loading DH parameters from {file_path}: {str(e)}")
    
    def joint_state_callback(self, msg):
        """Handle joint state message"""
        # Store current joint state
        self.current_joint_state = msg
        
        # Compute forward kinematics
        try:
            pose = self.kinematics_control.forward_kinematics(msg.position)
            
            # Publish pose
            self.pose_pub.publish(pose)
            
        except Exception as e:
            rospy.logerr(f"Error computing forward kinematics: {str(e)}")
    
    def forward_kinematics_callback(self, req):
        """Handle forward kinematics service request"""
        try:
            # Call kinematics control to compute forward kinematics
            pose = self.kinematics_control.forward_kinematics(req.joint_positions)
            
            # Create response
            response = ForwardKinematicsResponse()
            response.pose = pose
            response.success = True
            response.message = "Forward kinematics computed successfully"
            
            return response
            
        except Exception as e:
            rospy.logerr(f"Error computing forward kinematics: {str(e)}")
            
            # Create error response
            response = ForwardKinematicsResponse()
            response.pose = Pose()
            response.success = False
            response.message = f"Error: {str(e)}"
            
            return response
    
    def inverse_kinematics_callback(self, req):
        """Handle inverse kinematics service request"""
        try:
            # Get seed joint positions
            seed_joint_positions = None
            
            if req.use_seed:
                seed_joint_positions = req.seed_joint_positions
            elif self.current_joint_state is not None:
                seed_joint_positions = self.current_joint_state.position
            
            # Call kinematics control to compute inverse kinematics
            joint_positions = self.kinematics_control.inverse_kinematics(
                req.pose,
                seed_joint_positions=seed_joint_positions
            )
            
            # Check joint limits
            valid, violations = self.kinematics_control.check_joint_limits(joint_positions)
            
            # Create response
            response = InverseKinematicsResponse()
            response.joint_positions = joint_positions
            response.success = valid
            
            if valid:
                response.message = "Inverse kinematics computed successfully"
            else:
                response.message = "Inverse kinematics solution violates joint limits: " + ", ".join(violations)
            
            return response
            
        except Exception as e:
            rospy.logerr(f"Error computing inverse kinematics: {str(e)}")
            
            # Create error response
            response = InverseKinematicsResponse()
            response.joint_positions = []
            response.success = False
            response.message = f"Error: {str(e)}"
            
            return response

def main():
    """Main function"""
    try:
        # Create kinematics node
        kinematics_node = KinematicsNode()
        
        # Process callbacks
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 