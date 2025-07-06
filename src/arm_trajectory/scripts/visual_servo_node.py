#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse

# 使用包导入
from arm_trajectory import VisualServoControl

class VisualServoNode:
    """
    ROS node for visual servoing control
    """
    
    def __init__(self):
        """Initialize the visual servo node"""
        rospy.init_node('visual_servo_node', anonymous=True)
        
        # Initialize parameters
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
        self.feature_detection_method = rospy.get_param('~feature_detection_method', 'aruco')
        self.control_rate = rospy.get_param('~control_rate', 10)  # Hz
        self.servo_enabled = rospy.get_param('~servo_enabled', False)
        
        # Initialize control parameters
        gain = rospy.get_param('~gain', 0.5)
        max_velocity = rospy.get_param('~max_velocity', 0.2)
        min_velocity = rospy.get_param('~min_velocity', 0.01)
        target_tolerance = rospy.get_param('~target_tolerance', 0.01)
        orientation_tolerance = rospy.get_param('~orientation_tolerance', 0.05)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize visual servo control
        self.visual_servo = VisualServoControl()
        self.visual_servo.set_control_parameters(
            gain=gain,
            max_velocity=max_velocity,
            min_velocity=min_velocity,
            target_tolerance=target_tolerance,
            orientation_tolerance=orientation_tolerance
        )
        
        # Create service
        self.enable_service = rospy.Service('/visual_servo/enable', SetBool, self.enable_servo_callback)
        
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        # Subscribe to joint state topic
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # Create publishers
        self.velocity_pub = rospy.Publisher('/visual_servo/velocity', Twist, queue_size=10)
        self.image_pub = rospy.Publisher('/visual_servo/image', Image, queue_size=10)
        self.status_pub = rospy.Publisher('/visual_servo/status', Bool, queue_size=10, latch=True)
        
        # Initialize feature detector
        self._init_feature_detector()
        
        # Initialize target features
        self.target_features = None
        
        # Initialize current image
        self.current_image = None
        
        # Initialize joint state
        self.current_joint_state = None
        
        # Create timer for control loop
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_timer_callback)
        
        # Publish initial status
        self.publish_status()
        
        rospy.loginfo(f"Visual servo node initialized")
    
    def _init_feature_detector(self):
        """Initialize feature detector based on detection method"""
        if self.feature_detection_method == 'aruco':
            # Initialize ArUco detector
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        elif self.feature_detection_method == 'sift':
            # Initialize SIFT detector
            self.sift = cv2.SIFT_create()
            self.flann = cv2.FlannBasedMatcher({'algorithm': 0, 'trees': 5}, {'checks': 50})
        else:
            rospy.logwarn(f"Unknown feature detection method: {self.feature_detection_method}, using ArUco")
            self.feature_detection_method = 'aruco'
            # Initialize ArUco detector
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    
    def publish_status(self):
        """Publish current servo status"""
        status_msg = Bool()
        status_msg.data = self.servo_enabled and not self.visual_servo.is_target_reached()
        self.status_pub.publish(status_msg)
    
    def enable_servo_callback(self, req):
        """Handle enable/disable servo service request"""
        enable = req.data
        
        if enable and not self.servo_enabled:
            # Enable visual servoing
            self.servo_enabled = True
            self.visual_servo.start_control()
            rospy.loginfo("Visual servoing enabled")
        elif not enable and self.servo_enabled:
            # Disable visual servoing
            self.servo_enabled = False
            self.visual_servo.stop_control()
            rospy.loginfo("Visual servoing disabled")
            
            # Publish zero velocity
            self.publish_velocity(np.zeros(6))
        
        # Publish status
        self.publish_status()
        
        return SetBoolResponse(True, f"Visual servoing {'enabled' if enable else 'disabled'}")
    
    def joint_state_callback(self, msg):
        """Handle joint state message"""
        self.current_joint_state = msg
    
    def image_callback(self, msg):
        """Handle image message"""
        try:
            # Convert ROS image message to OpenCV format
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect features
            features = self.detect_features(self.current_image)
            
            if features is not None and len(features) > 0:
                # Set current features
                self.visual_servo.set_current_features(features)
                
                # If target features not set, use current features as target
                if self.target_features is None:
                    self.target_features = features
                    self.visual_servo.set_target_features(self.target_features)
                    rospy.loginfo(f"Target features set: {self.target_features}")
            
            # Draw features on image
            annotated_image = self.draw_features(self.current_image, features)
            
            # Convert OpenCV image back to ROS image message
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            
            # Publish annotated image
            self.image_pub.publish(annotated_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {str(e)}")
    
    def detect_features(self, image):
        """
        Detect features in image
        
        Args:
            image: Input image
            
        Returns:
            features: Detected feature points
        """
        if image is None:
            return None
        
        if self.feature_detection_method == 'aruco':
            # Detect ArUco markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(image)
            
            if ids is not None and len(ids) > 0:
                # Extract corner points
                features = []
                for corner in corners:
                    # Use center of marker
                    center = np.mean(corner[0], axis=0)
                    features.append(center)
                
                return np.array(features)
            else:
                return None
        elif self.feature_detection_method == 'sift':
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect SIFT features
            keypoints, descriptors = self.sift.detectAndCompute(gray, None)
            
            if keypoints and len(keypoints) > 0:
                # Extract keypoint coordinates
                features = np.array([kp.pt for kp in keypoints])
                
                # Limit number of features
                max_features = 10
                if len(features) > max_features:
                    # Select features with highest response
                    responses = np.array([kp.response for kp in keypoints])
                    indices = np.argsort(responses)[-max_features:]
                    features = features[indices]
                
                return features
            else:
                return None
        else:
            rospy.logwarn(f"Unknown feature detection method: {self.feature_detection_method}")
            return None
    
    def draw_features(self, image, features):
        """
        Draw features on image
        
        Args:
            image: Input image
            features: Feature points
            
        Returns:
            annotated_image: Image with features drawn
        """
        if image is None:
            return None
        
        # Create copy of image
        annotated_image = image.copy()
        
        # Draw current features
        if features is not None and len(features) > 0:
            for i, (x, y) in enumerate(features):
                # Draw circle
                cv2.circle(annotated_image, (int(x), int(y)), 5, (0, 255, 0), 2)
                
                # Draw index
                cv2.putText(
                    annotated_image,
                    str(i),
                    (int(x) + 10, int(y) + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
        
        # Draw target features
        if self.target_features is not None and len(self.target_features) > 0:
            for i, (x, y) in enumerate(self.target_features):
                # Draw circle
                cv2.circle(annotated_image, (int(x), int(y)), 5, (0, 0, 255), 2)
                
                # Draw index
                cv2.putText(
                    annotated_image,
                    f"T{i}",
                    (int(x) + 10, int(y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2
                )
        
        # Draw status
        status_text = "SERVO: " + ("ON" if self.servo_enabled else "OFF")
        cv2.putText(
            annotated_image,
            status_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0) if self.servo_enabled else (0, 0, 255),
            2
        )
        
        # Draw error
        error, error_norm = self.visual_servo.get_error()
        if error is not None:
            error_text = f"Error: {error_norm:.3f}"
            cv2.putText(
                annotated_image,
                error_text,
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0) if error_norm < self.visual_servo.target_tolerance else (0, 0, 255),
                2
            )
        
        return annotated_image
    
    def control_timer_callback(self, event):
        """Handle control timer event"""
        if not self.servo_enabled:
            return
        
        # Check if target is reached
        if self.visual_servo.is_target_reached():
            rospy.loginfo("Target reached")
            self.servo_enabled = False
            self.visual_servo.stop_control()
            
            # Publish zero velocity
            self.publish_velocity(np.zeros(6))
            
            # Publish status
            self.publish_status()
            
            return
        
        # Compute velocity
        velocity, error_norm = self.visual_servo.compute_velocity()
        
        # Publish velocity
        self.publish_velocity(velocity)
    
    def publish_velocity(self, velocity):
        """
        Publish velocity command
        
        Args:
            velocity: Velocity command [vx, vy, vz, wx, wy, wz]
        """
        # Create Twist message
        twist = Twist()
        
        # Set linear velocity
        twist.linear.x = velocity[0]
        twist.linear.y = velocity[1]
        twist.linear.z = velocity[2]
        
        # Set angular velocity
        twist.angular.x = velocity[3]
        twist.angular.y = velocity[4]
        twist.angular.z = velocity[5]
        
        # Publish velocity
        self.velocity_pub.publish(twist)

def main():
    """Main function"""
    try:
        # Create visual servo node
        visual_servo_node = VisualServoNode()
        
        # Process callbacks
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 