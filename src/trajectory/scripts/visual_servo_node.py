#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from visualization_msgs.msg import Marker, MarkerArray

# 导入visual_servo_bridge
from visual_servo_bridge import VisualServoBridge

class VisualServoNode:
    """
    ROS node for visual servoing
    """
    
    def __init__(self):
        """Initialize the visual servo node"""
        # 检查节点是否已初始化
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('visual_servo_node', anonymous=True)
        except:
            # 节点可能已经在其他地方被初始化，继续执行
            rospy.loginfo("Node may be already initialized, continuing")
        
        # Initialize parameters
        self.update_rate = rospy.get_param('~update_rate', 10.0)
        self.servo_enabled = rospy.get_param('~enabled', False)
        
        # Initialize visual servo bridge
        self.visual_servo = VisualServoBridge()
        
        # Create services
        self.enable_service = rospy.Service('/visual_servo/enable', SetBool, self.enable_servo_callback)
        
        # Subscribe to target pose topic
        self.target_pose_sub = rospy.Subscriber('/visual_servo/target_pose', Pose, self.target_pose_callback)
        
        # Create publishers
        self.status_pub = rospy.Publisher('/visual_servo/status', Bool, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher('/visual_servo/markers', MarkerArray, queue_size=10)
        
        # Initialize target pose
        self.target_pose = None
        
        # Publish initial status
        self.publish_status()
        
        # Create timer for visual servo control
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.control_timer_callback)
        
        rospy.loginfo("Visual servo node initialized")
    
    def enable_servo_callback(self, req):
        """Handle enable/disable servo service request"""
        self.servo_enabled = req.data
        
        # Enable/disable visual servo bridge
        self.visual_servo.set_enabled(self.servo_enabled)
        
        # Publish status update
        self.publish_status()
        
        return SetBoolResponse(True, f"Visual servoing {'enabled' if self.servo_enabled else 'disabled'}")
    
    def target_pose_callback(self, msg):
        """Handle target pose message"""
        self.target_pose = msg
        self.visual_servo.set_target_pose(msg)
        rospy.loginfo("Received target pose for visual servoing")
    
    def publish_status(self):
        """Publish current visual servo status"""
        status_msg = Bool()
        status_msg.data = self.servo_enabled
        self.status_pub.publish(status_msg)
    
    def control_timer_callback(self, event):
        """Execute visual servo control on timer event"""
        if not self.servo_enabled or self.target_pose is None:
            return
        
        try:
            # Execute visual servo control
            success = self.visual_servo.update()
            
            # Publish visual servo markers
            self.publish_markers()
            
        except Exception as e:
            rospy.logerr(f"Error in visual servo control: {str(e)}")
    
    def publish_markers(self):
        """Publish visual servo markers"""
        marker_array = MarkerArray()
        
        # Get current target pose
        target_pose = self.visual_servo.get_target_pose()
        if target_pose is None:
            return
        
        # Create target marker
        target_marker = Marker()
        target_marker.header.stamp = rospy.Time.now()
        target_marker.header.frame_id = "base_link"
        target_marker.ns = "visual_servo"
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose = target_pose
        target_marker.scale.x = 0.05
        target_marker.scale.y = 0.05
        target_marker.scale.z = 0.05
        target_marker.color.r = 0.0
        target_marker.color.g = 1.0
        target_marker.color.b = 0.0
        target_marker.color.a = 0.8
        target_marker.lifetime = rospy.Duration(0)
        
        # Add to marker array
        marker_array.markers.append(target_marker)
        
        # Publish marker array
        self.marker_pub.publish(marker_array)

def main():
    """Main function"""
    try:
        # Create visual servo node
        visual_servo_node = VisualServoNode()
        
        # Spin
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    
    except Exception as e:
        rospy.logerr(f"Error in visual servo node: {str(e)}")

if __name__ == '__main__':
    main() 