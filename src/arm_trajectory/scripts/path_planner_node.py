#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from arm_trajectory.srv import PlanTrajectory, PlanTrajectoryResponse

from path_planner_control import PathPlannerControl

class PathPlannerNode:
    """
    ROS node for path planning
    """
    
    def __init__(self):
        """Initialize the path planner node"""
        rospy.init_node('path_planner_node', anonymous=True)
        
        # Initialize parameters
        self.planning_time = rospy.get_param('~planning_time', 5.0)
        self.num_waypoints = rospy.get_param('~num_waypoints', 10)
        self.collision_threshold = rospy.get_param('~collision_threshold', 0.05)
        self.smoothness_weight = rospy.get_param('~smoothness_weight', 0.3)
        self.obstacle_weight = rospy.get_param('~obstacle_weight', 0.7)
        self.planning_frame = rospy.get_param('~planning_frame', 'base_link')
        
        # Initialize path planner control
        self.path_planner = PathPlannerControl()
        self.path_planner.set_planning_parameters(
            planning_time=self.planning_time,
            num_waypoints=self.num_waypoints,
            collision_threshold=self.collision_threshold,
            smoothness_weight=self.smoothness_weight,
            obstacle_weight=self.obstacle_weight
        )
        
        # Create service
        self.plan_service = rospy.Service('/path_planner/plan', PlanTrajectory, self.plan_trajectory_callback)
        
        # Subscribe to obstacle topic
        self.obstacles_sub = rospy.Subscriber('/path_planner/obstacles', MarkerArray, self.obstacles_callback)
        
        # Subscribe to start pose topic
        self.start_pose_sub = rospy.Subscriber('/path_planner/start_pose', Pose, self.start_pose_callback)
        
        # Subscribe to target pose topic
        self.target_pose_sub = rospy.Subscriber('/path_planner/target_pose', Pose, self.target_pose_callback)
        
        # Create publishers
        self.path_pub = rospy.Publisher('/path_planner/path', PoseArray, queue_size=10)
        self.path_markers_pub = rospy.Publisher('/path_planner/path_markers', MarkerArray, queue_size=10)
        self.status_pub = rospy.Publisher('/path_planner/status', Bool, queue_size=10, latch=True)
        
        # Initialize poses
        self.start_pose = None
        self.target_pose = None
        
        # Initialize obstacles
        self.obstacles = []
        
        # Initialize path
        self.current_path = None
        
        # Publish initial status
        self.publish_status(False)
        
        rospy.loginfo(f"Path planner node initialized")
    
    def publish_status(self, planning_active):
        """Publish current planning status"""
        status_msg = Bool()
        status_msg.data = planning_active
        self.status_pub.publish(status_msg)
    
    def obstacles_callback(self, msg):
        """Handle obstacles message"""
        # Extract obstacles from marker array
        obstacles = []
        
        for marker in msg.markers:
            if marker.type == Marker.SPHERE:
                # Extract position and scale
                position = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
                radius = marker.scale.x / 2.0  # Assuming scale is diameter
                
                # Add to obstacles list
                obstacles.append(position + [radius])
        
        # Set obstacles in path planner
        self.obstacles = obstacles
        self.path_planner.set_obstacles(obstacles)
        
        rospy.loginfo(f"Received {len(obstacles)} obstacles")
    
    def start_pose_callback(self, msg):
        """Handle start pose message"""
        self.start_pose = msg
        self.path_planner.set_start_pose(msg)
        rospy.loginfo("Received start pose")
    
    def target_pose_callback(self, msg):
        """Handle target pose message"""
        self.target_pose = msg
        self.path_planner.set_target_pose(msg)
        rospy.loginfo("Received target pose")
        
        # Plan path if start pose is also available
        if self.start_pose is not None:
            self.plan_path()
    
    def plan_path(self):
        """Plan path from start pose to target pose"""
        # Publish planning status
        self.publish_status(True)
        
        try:
            # Plan path
            path, success = self.path_planner.plan_path()
            
            if success and path:
                # Store path
                self.current_path = path
                
                # Publish path
                self.publish_path(path)
                
                rospy.loginfo(f"Path planning successful with {len(path)} waypoints")
            else:
                rospy.logwarn("Path planning failed")
                
            # Publish planning status
            self.publish_status(False)
            
            return path, success
            
        except Exception as e:
            rospy.logerr(f"Error planning path: {str(e)}")
            
            # Publish planning status
            self.publish_status(False)
            
            return None, False
    
    def plan_trajectory_callback(self, req):
        """Handle plan trajectory service request"""
        # Set planning parameters
        self.path_planner.set_planning_parameters(
            planning_time=req.planning_time if req.planning_time > 0 else self.planning_time,
            num_waypoints=req.num_planning_attempts if req.num_planning_attempts > 0 else self.num_waypoints
        )
        
        # Set start and target poses
        if req.use_start_state:
            self.path_planner.set_start_pose(req.start_joint_state.pose)
            self.start_pose = req.start_joint_state.pose
        elif self.start_pose is None:
            return PlanTrajectoryResponse(False, "No start pose available")
        
        self.path_planner.set_target_pose(req.target_pose)
        self.target_pose = req.target_pose
        
        # Plan path
        path, success = self.plan_path()
        
        if success and path:
            return PlanTrajectoryResponse(True, f"Path planning successful with {len(path)} waypoints")
        else:
            return PlanTrajectoryResponse(False, "Path planning failed")
    
    def publish_path(self, path):
        """
        Publish path
        
        Args:
            path: List of poses
        """
        # Create pose array message
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = self.planning_frame
        pose_array.poses = path
        
        # Publish pose array
        self.path_pub.publish(pose_array)
        
        # Create marker array message
        marker_array = MarkerArray()
        
        # Add path markers
        for i, pose in enumerate(path):
            # Create marker
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.planning_frame
            marker.ns = "path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)
            
            # Add to marker array
            marker_array.markers.append(marker)
            
            # Add line marker if not first pose
            if i > 0:
                line_marker = Marker()
                line_marker.header.stamp = rospy.Time.now()
                line_marker.header.frame_id = self.planning_frame
                line_marker.ns = "path_lines"
                line_marker.id = i - 1
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                line_marker.points = [path[i-1].position, pose.position]
                line_marker.scale.x = 0.01
                line_marker.color.r = 0.0
                line_marker.color.g = 0.8
                line_marker.color.b = 0.0
                line_marker.color.a = 1.0
                line_marker.lifetime = rospy.Duration(0)
                
                # Add to marker array
                marker_array.markers.append(line_marker)
        
        # Add start and target markers
        if self.start_pose is not None:
            start_marker = Marker()
            start_marker.header.stamp = rospy.Time.now()
            start_marker.header.frame_id = self.planning_frame
            start_marker.ns = "path_endpoints"
            start_marker.id = 0
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose = self.start_pose
            start_marker.scale.x = 0.05
            start_marker.scale.y = 0.05
            start_marker.scale.z = 0.05
            start_marker.color.r = 1.0
            start_marker.color.g = 0.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            start_marker.lifetime = rospy.Duration(0)
            
            # Add to marker array
            marker_array.markers.append(start_marker)
        
        if self.target_pose is not None:
            target_marker = Marker()
            target_marker.header.stamp = rospy.Time.now()
            target_marker.header.frame_id = self.planning_frame
            target_marker.ns = "path_endpoints"
            target_marker.id = 1
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose = self.target_pose
            target_marker.scale.x = 0.05
            target_marker.scale.y = 0.05
            target_marker.scale.z = 0.05
            target_marker.color.r = 0.0
            target_marker.color.g = 0.0
            target_marker.color.b = 1.0
            target_marker.color.a = 1.0
            target_marker.lifetime = rospy.Duration(0)
            
            # Add to marker array
            marker_array.markers.append(target_marker)
        
        # Publish marker array
        self.path_markers_pub.publish(marker_array)

def main():
    """Main function"""
    try:
        # Create path planner node
        path_planner_node = PathPlannerNode()
        
        # Process callbacks
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 