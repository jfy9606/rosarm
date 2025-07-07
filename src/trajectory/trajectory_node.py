#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Pose, PoseArray
from trajectory.msg import TrajectoryPoint, TrajectoryPath
from trajectory.srv import ForwardKinematics, InverseKinematics, PlanTrajectory

# 使用直接导入
import sys, os
# 添加scripts目录到Python路径
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scripts'))
from trajectory_control import TrajectoryControl

class TrajectoryNode:
    """
    ROS node for robot arm trajectory planning and execution
    """
    
    def __init__(self):
        """Initialize the trajectory node"""
        rospy.init_node('trajectory_node', anonymous=True)
        
        # Initialize parameters
        self.planning_enabled = rospy.get_param('~planning_enabled', True)
        self.execution_enabled = rospy.get_param('~execution_enabled', True)
        self.planning_frame = rospy.get_param('~planning_frame', 'base_link')
        
        # Initialize trajectory control
        self.trajectory_control = TrajectoryControl()
        
        # Create services
        self.fk_service = rospy.Service('/trajectory/forward_kinematics', ForwardKinematics, self.forward_kinematics_callback)
        self.ik_service = rospy.Service('/trajectory/inverse_kinematics', InverseKinematics, self.inverse_kinematics_callback)
        self.plan_service = rospy.Service('/trajectory/plan_trajectory', PlanTrajectory, self.plan_trajectory_callback)
        
        # Subscribe to joint state topic
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Subscribe to target pose topic
        self.target_pose_sub = rospy.Subscriber('/trajectory/target_pose', Pose, self.target_pose_callback)
        
        # Subscribe to control topics
        self.planning_sub = rospy.Subscriber('/trajectory/planning_enabled', Bool, self.planning_enabled_callback)
        self.execution_sub = rospy.Subscriber('/trajectory/execution_enabled', Bool, self.execution_enabled_callback)
        
        # Create publishers
        self.trajectory_pub = rospy.Publisher('/trajectory/path', TrajectoryPath, queue_size=10)
        self.joint_command_pub = rospy.Publisher('/joint_command', Float64MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher('/trajectory/status', Bool, queue_size=10, latch=True)
        
        # Initialize joint state
        self.current_joint_state = None
        
        # Initialize trajectory
        self.current_trajectory = None
        self.trajectory_index = 0
        self.trajectory_active = False
        
        # Create timer for trajectory execution
        self.execution_timer = rospy.Timer(rospy.Duration(0.1), self.execution_timer_callback)
        
        # Publish initial status
        self.publish_status()
        
        rospy.loginfo(f"Trajectory node initialized")
    
    def publish_status(self):
        """Publish current trajectory status"""
        status_msg = Bool()
        status_msg.data = self.trajectory_active
        self.status_pub.publish(status_msg)
    
    def joint_state_callback(self, msg):
        """Handle joint state message"""
        # Store current joint state
        self.current_joint_state = msg
        
        # Update trajectory control
        self.trajectory_control.set_current_joint_state(
            positions=msg.position,
            velocities=msg.velocity,
            efforts=msg.effort,
            joint_names=msg.name
        )
    
    def target_pose_callback(self, msg):
        """Handle target pose message"""
        if not self.planning_enabled:
            rospy.logwarn("Trajectory planning is disabled")
            return
        
        try:
            # Plan trajectory to target pose
            trajectory = self.trajectory_control.plan_trajectory_to_pose(
                target_pose=msg,
                start_joint_state=self.current_joint_state
            )
            
            if trajectory:
                # Publish trajectory
                self.publish_trajectory(trajectory)
                
                # Start trajectory execution
                if self.execution_enabled:
                    self.start_trajectory_execution(trajectory)
            else:
                rospy.logwarn("Failed to plan trajectory to target pose")
                
        except Exception as e:
            rospy.logerr(f"Error planning trajectory: {str(e)}")
    
    def planning_enabled_callback(self, msg):
        """Handle planning enabled message"""
        self.planning_enabled = msg.data
        rospy.loginfo(f"Trajectory planning {'enabled' if self.planning_enabled else 'disabled'}")
    
    def execution_enabled_callback(self, msg):
        """Handle execution enabled message"""
        self.execution_enabled = msg.data
        rospy.loginfo(f"Trajectory execution {'enabled' if self.execution_enabled else 'disabled'}")
        
        # Stop trajectory execution if disabled
        if not self.execution_enabled and self.trajectory_active:
            self.stop_trajectory_execution()
    
    def forward_kinematics_callback(self, req):
        """Handle forward kinematics service request"""
        try:
            # Call trajectory control to compute forward kinematics
            pose = self.trajectory_control.compute_forward_kinematics(req.joint_positions)
            
            # Return pose
            return {'pose': pose, 'success': True, 'message': "Forward kinematics computed successfully"}
            
        except Exception as e:
            rospy.logerr(f"Error computing forward kinematics: {str(e)}")
            return {'pose': Pose(), 'success': False, 'message': f"Error: {str(e)}"}
    
    def inverse_kinematics_callback(self, req):
        """Handle inverse kinematics service request"""
        try:
            # Call trajectory control to compute inverse kinematics
            joint_positions = self.trajectory_control.compute_inverse_kinematics(
                req.pose,
                seed_joint_positions=req.seed_joint_positions if req.use_seed else None
            )
            
            # Return joint positions
            return {'joint_positions': joint_positions, 'success': True, 'message': "Inverse kinematics computed successfully"}
            
        except Exception as e:
            rospy.logerr(f"Error computing inverse kinematics: {str(e)}")
            return {'joint_positions': [], 'success': False, 'message': f"Error: {str(e)}"}
    
    def plan_trajectory_callback(self, req):
        """Handle plan trajectory service request"""
        try:
            # Call trajectory control to plan trajectory
            trajectory = self.trajectory_control.plan_trajectory(
                start_joint_state=req.start_joint_state if req.use_start_state else self.current_joint_state,
                target_pose=req.target_pose,
                planning_time=req.planning_time,
                num_planning_attempts=req.num_planning_attempts
            )
            
            if trajectory:
                # Publish trajectory
                self.publish_trajectory(trajectory)
                
                # Start trajectory execution if requested
                if req.execute and self.execution_enabled:
                    self.start_trajectory_execution(trajectory)
                
                return {'success': True, 'message': "Trajectory planned successfully"}
            else:
                return {'success': False, 'message': "Failed to plan trajectory"}
                
        except Exception as e:
            rospy.logerr(f"Error planning trajectory: {str(e)}")
            return {'success': False, 'message': f"Error: {str(e)}"}
    
    def publish_trajectory(self, trajectory):
        """
        Publish trajectory
        
        Args:
            trajectory: List of trajectory points
        """
        # Create trajectory path message
        path_msg = TrajectoryPath()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.planning_frame
        
        # Add trajectory points
        for point in trajectory:
            # Create trajectory point message
            point_msg = TrajectoryPoint()
            point_msg.joint_positions = point['positions']
            point_msg.joint_velocities = point['velocities']
            point_msg.joint_accelerations = point['accelerations']
            point_msg.time_from_start = point['time_from_start']
            
            # Add to path
            path_msg.points.append(point_msg)
        
        # Publish trajectory path
        self.trajectory_pub.publish(path_msg)
    
    def start_trajectory_execution(self, trajectory):
        """
        Start trajectory execution
        
        Args:
            trajectory: List of trajectory points
        """
        # Store trajectory
        self.current_trajectory = trajectory
        self.trajectory_index = 0
        self.trajectory_active = True
        
        # Publish status
        self.publish_status()
        
        rospy.loginfo(f"Starting trajectory execution with {len(trajectory)} points")
    
    def stop_trajectory_execution(self):
        """Stop trajectory execution"""
        self.trajectory_active = False
        
        # Publish status
        self.publish_status()
        
        rospy.loginfo("Trajectory execution stopped")
    
    def execution_timer_callback(self, event):
        """Handle execution timer event"""
        if not self.trajectory_active or not self.execution_enabled:
            return
        
        try:
            # Check if trajectory is complete
            if self.trajectory_index >= len(self.current_trajectory):
                rospy.loginfo("Trajectory execution complete")
                self.stop_trajectory_execution()
                return
            
            # Get current trajectory point
            point = self.current_trajectory[self.trajectory_index]
            
            # Create joint command message
            command_msg = Float64MultiArray()
            command_msg.data = point['positions']
            
            # Publish joint command
            self.joint_command_pub.publish(command_msg)
            
            # Increment trajectory index
            self.trajectory_index += 1
            
        except Exception as e:
            rospy.logerr(f"Error executing trajectory: {str(e)}")
            self.stop_trajectory_execution()

def main():
    """Main function"""
    try:
        # Create trajectory node
        trajectory_node = TrajectoryNode()
        
        # Process callbacks
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 