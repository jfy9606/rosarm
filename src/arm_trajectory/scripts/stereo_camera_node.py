#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import threading

class StereoCameraNode:
    """Stereo camera node for capturing and publishing stereo images"""
    
    def __init__(self):
        rospy.init_node('stereo_camera_node', anonymous=True)
        
        # Get parameters
        self.camera_index = int(rospy.get_param('~camera_index', 0))
        self.frame_rate = int(rospy.get_param('~frame_rate', 30))
        
        # Initialize bridge
        self.bridge = CvBridge()
        
        # Initialize publishers
        self.left_pub = rospy.Publisher('/stereo_camera/left/image_raw', Image, queue_size=1)
        self.right_pub = rospy.Publisher('/stereo_camera/right/image_raw', Image, queue_size=1)
        self.merged_pub = rospy.Publisher('/stereo_camera/image_merged', Image, queue_size=1)
        self.left_info_pub = rospy.Publisher('/stereo_camera/left/camera_info', CameraInfo, queue_size=1)
        self.right_info_pub = rospy.Publisher('/stereo_camera/right/camera_info', CameraInfo, queue_size=1)
        
        # Initialize camera
        self.camera = None
        self.is_running = False
        self.camera_lock = threading.Lock()
        
        # Try to open the camera
        self.open_camera()
        
        # Start publishing thread
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.daemon = True
        self.thread.start()
        
        rospy.loginfo("Stereo Camera Node initialized")
        
        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)
    
    def open_camera(self):
        """Open the camera and set up parameters"""
        try:
            with self.camera_lock:
                # Close existing camera if open
                if self.camera is not None:
                    self.camera.release()
                
                # Open camera
                self.camera = cv2.VideoCapture(self.camera_index)
                
                if not self.camera.isOpened():
                    rospy.logerr(f"Failed to open camera at index {self.camera_index}")
                    return False
                
                # Set camera properties
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Width for stereo setup
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Height for stereo setup
                self.camera.set(cv2.CAP_PROP_FPS, self.frame_rate)
                
                # Get actual camera properties
                actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
                actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
                
                rospy.loginfo(f"Camera opened with resolution: {int(actual_width)}x{int(actual_height)} @ {int(actual_fps)}fps")
                
                # Set running flag
                self.is_running = True
                return True
                
        except Exception as e:
            rospy.logerr(f"Error opening camera: {e}")
            self.is_running = False
            return False
    
    def publish_loop(self):
        """Main loop for capturing and publishing images"""
        rate = rospy.Rate(self.frame_rate)
        
        camera_info_left = CameraInfo()
        camera_info_right = CameraInfo()
        
        # Set up basic camera info
        camera_info_left.header.frame_id = "stereo_left"
        camera_info_right.header.frame_id = "stereo_right"
        camera_info_left.height = 480
        camera_info_left.width = 640
        camera_info_right.height = 480
        camera_info_right.width = 640
        
        # Set up camera matrix (approximate values)
        K = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        camera_info_left.K = K
        camera_info_right.K = K
        
        # Set distortion model
        camera_info_left.distortion_model = "plumb_bob"
        camera_info_right.distortion_model = "plumb_bob"
        
        while not rospy.is_shutdown():
            if not self.is_running:
                # Try to reopen camera if it's closed
                if not self.open_camera():
                    rate.sleep()
                    continue
            
            # Capture frame
            with self.camera_lock:
                if self.camera is None:
                    rate.sleep()
                    continue
                    
                ret, frame = self.camera.read()
            
            if not ret:
                rospy.logerr("Failed to capture frame")
                self.is_running = False
                rate.sleep()
                continue
            
            try:
                # Split frame into left and right images
                height, width = frame.shape[:2]
                half_width = width // 2
                
                if width > 1:
                    left_image = frame[:, :half_width]
                    right_image = frame[:, half_width:]
                else:
                    # If we can't split the image, create dummy images
                    left_image = np.zeros((480, 640, 3), dtype=np.uint8)
                    right_image = np.zeros((480, 640, 3), dtype=np.uint8)
                
                # Create merged image (side by side with colored borders)
                merged_image = np.copy(frame)
                cv2.line(merged_image, (half_width, 0), (half_width, height), (0, 255, 0), 2)
                
                # Create timestamps
                now = rospy.Time.now()
                
                # Publish left image
                left_msg = self.bridge.cv2_to_imgmsg(left_image, "bgr8")
                left_msg.header.stamp = now
                left_msg.header.frame_id = "stereo_left"
                self.left_pub.publish(left_msg)
                
                # Publish right image
                right_msg = self.bridge.cv2_to_imgmsg(right_image, "bgr8")
                right_msg.header.stamp = now
                right_msg.header.frame_id = "stereo_right"
                self.right_pub.publish(right_msg)
                
                # Publish merged image
                merged_msg = self.bridge.cv2_to_imgmsg(merged_image, "bgr8")
                merged_msg.header.stamp = now
                merged_msg.header.frame_id = "stereo_camera"
                self.merged_pub.publish(merged_msg)
                
                # Publish camera info
                camera_info_left.header.stamp = now
                camera_info_right.header.stamp = now
                self.left_info_pub.publish(camera_info_left)
                self.right_info_pub.publish(camera_info_right)
                
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge error: {e}")
            except Exception as e:
                rospy.logerr(f"Error in publish loop: {e}")
            
            rate.sleep()
    
    def shutdown(self):
        """Clean up on shutdown"""
        rospy.loginfo("Shutting down stereo camera node")
        self.is_running = False
        
        with self.camera_lock:
            if self.camera is not None:
                self.camera.release()
                self.camera = None

if __name__ == '__main__':
    try:
        node = StereoCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 