#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import time
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from stereo_vision.msg import ObjectDetection
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header, Bool
import tf2_ros
import geometry_msgs.msg
from camera_config import (left_camera_matrix, left_distortion,
                          right_camera_matrix, right_distortion,
                          R, T, size,
                          R1, R2, P1, P2, Q,
                          left_map1, left_map2, right_map1, right_map2)

class StereoDetectionNode:
    def __init__(self):
        rospy.init_node('stereo_detection_node', anonymous=True)
        
        # Load YOLOv8 model
        model_path = rospy.get_param('~yolo_model_path', 'yolov8n.pt')
        try:
            self.model = YOLO(model_path)
            rospy.loginfo(f"Successfully loaded YOLO model: {model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {str(e)}")
            rospy.loginfo("Trying default model yolov8n.pt")
            try:
                self.model = YOLO('yolov8n.pt')
                rospy.loginfo("Successfully loaded default YOLO model")
            except Exception as e:
                rospy.logerr(f"Failed to load default YOLO model: {str(e)}")
                self.model = None
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # SGBM parameters for stereo matching
        blockSize = 8
        img_channels = 3
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=1,
            numDisparities=64,
            blockSize=blockSize,
            P1=8 * img_channels * blockSize * blockSize,
            P2=32 * img_channels * blockSize * blockSize,
            disp12MaxDiff=-1,
            preFilterCap=140,
            uniquenessRatio=1,
            speckleWindowSize=100,
            speckleRange=100,
            mode=cv2.STEREO_SGBM_MODE_HH)
        
        # Initialize transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Publishers
        self.detection_pub = rospy.Publisher('stereo_detections', ObjectDetection, queue_size=10)
        self.depth_image_pub = rospy.Publisher('depth_image', Image, queue_size=10)
        self.result_image_pub = rospy.Publisher('detection_image', Image, queue_size=10)
        self.pose_array_pub = rospy.Publisher('detected_poses', PoseArray, queue_size=10)
        self.left_image_pub = rospy.Publisher('left_image', Image, queue_size=10)
        self.right_image_pub = rospy.Publisher('right_image', Image, queue_size=10)
        
        # GUI compatible publishers
        self.gui_detection_pub = rospy.Publisher('/detections/image', Image, queue_size=10)
        self.gui_poses_pub = rospy.Publisher('/detections/poses', PoseArray, queue_size=10)
        
        # Set camera parameters
        self.camera_info = CameraInfo()
        self.camera_info.K = left_camera_matrix.flatten().tolist()
        self.camera_info.D = left_distortion.tolist()
        self.camera_info.R = np.eye(3).flatten().tolist()
        self.camera_info.P = P1.flatten().tolist()
        self.camera_info.width = size[0]
        self.camera_info.height = size[1]
        
        # Current view mode (0=left, 1=right, 2=depth)
        self.view_mode = 0
        
        # Subscribe to stereo camera input
        if rospy.get_param('~use_split_feed', True):
            # Using two separate camera feeds
            self.left_sub = rospy.Subscriber('left_camera/image_raw', Image, self.left_callback, queue_size=1)
            self.right_sub = rospy.Subscriber('right_camera/image_raw', Image, self.right_callback, queue_size=1)
            self.left_image = None
            self.right_image = None
        else:
            # Using one combined feed (side-by-side images)
            self.camera_sub = rospy.Subscriber('stereo_camera/image_raw', Image, self.camera_callback, queue_size=1)
        
        # Subscribe to view mode changes
        self.view_mode_sub = rospy.Subscriber('/stereo_vision/view_mode', Header, self.view_mode_callback, queue_size=1)
        
        rospy.loginfo("Stereo detection node initialized")
    
    def view_mode_callback(self, msg):
        """Handle view mode changes"""
        if hasattr(msg, 'frame_id') and msg.frame_id:
            try:
                mode = int(msg.frame_id)
                if 0 <= mode <= 2:
                    self.view_mode = mode
                    rospy.loginfo(f"Changed view mode to {mode} (0=left, 1=right, 2=depth)")
            except ValueError:
                rospy.logwarn(f"Invalid view mode: {msg.frame_id}")
    
    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.right_image is not None:
            self.process_stereo_pair(self.left_image, self.right_image)
    
    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.left_image is not None:
            self.process_stereo_pair(self.left_image, self.right_image)
    
    def camera_callback(self, msg):
        # Convert combined image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Split the image into left and right
        height, width = frame.shape[:2]
        mid = width // 2
        left_image = frame[:, :mid]
        right_image = frame[:, mid:]
        
        # Process the stereo pair
        self.process_stereo_pair(left_image, right_image)
    
    def process_stereo_pair(self, left_img, right_img):
        # Publish left and right images
        left_msg = self.bridge.cv2_to_imgmsg(left_img, "bgr8")
        right_msg = self.bridge.cv2_to_imgmsg(right_img, "bgr8")
        self.left_image_pub.publish(left_msg)
        self.right_image_pub.publish(right_msg)
        
        # Convert to grayscale for stereo matching
        imgL_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        imgR_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        
        # Rectify images
        img1_rectified = cv2.remap(imgL_gray, left_map1, left_map2, cv2.INTER_LINEAR)
        img2_rectified = cv2.remap(imgR_gray, right_map1, right_map2, cv2.INTER_LINEAR)
        
        # Compute disparity
        disparity = self.stereo.compute(img1_rectified, img2_rectified)
        
        # Convert to depth map
        disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disparity, alpha=255 / 16), cv2.COLORMAP_JET)
        depth_msg = self.bridge.cv2_to_imgmsg(disp_color, "bgr8")
        self.depth_image_pub.publish(depth_msg)
        
        # Reproject to 3D
        points_3d = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True) * 16
        
        # Run YOLOv8 detection on left image if model is available
        if self.model:
            results = self.model(left_img, imgsz=640, conf=0.5)
            annotated_frame = results[0].plot()
        else:
            annotated_frame = left_img.copy()
            cv2.putText(annotated_frame, "YOLO model not available", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Create pose array for all detections
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "camera_link"
        
        # Process detections if model is available
        if self.model:
            for box in results[0].boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                
                # Get depth and 3D position
                depth = points_3d[center_y][center_x][2] / 1000.0  # Convert to meters
                
                if depth > 0:
                    # Calculate 3D position
                    x_3d = points_3d[center_y][center_x][0] / 10.0  # Convert to cm
                    y_3d = points_3d[center_y][center_x][1] / 10.0
                    z_3d = points_3d[center_y][center_x][2] / 10.0
                    distance = np.sqrt(x_3d**2 + y_3d**2 + z_3d**2)
                    
                    # Create detection message
                    detect_msg = ObjectDetection()
                    detect_msg.header = Header()
                    detect_msg.header.stamp = rospy.Time.now()
                    detect_msg.header.frame_id = "camera_link"
                    detect_msg.class_name = results[0].names[int(box.cls[0])]
                    detect_msg.confidence = float(box.conf[0])
                    detect_msg.x_min = float(x1)
                    detect_msg.y_min = float(y1)
                    detect_msg.x_max = float(x2)
                    detect_msg.y_max = float(y2)
                    detect_msg.x_3d = float(x_3d)
                    detect_msg.y_3d = float(y_3d)
                    detect_msg.z_3d = float(z_3d)
                    detect_msg.distance = float(distance)
                    self.detection_pub.publish(detect_msg)
                    
                    # Add to pose array
                    pose = Pose()
                    pose.position.x = x_3d / 100.0  # Convert to meters for ROS standard
                    pose.position.y = y_3d / 100.0
                    pose.position.z = z_3d / 100.0
                    pose.orientation.w = 1.0  # Default orientation (no rotation)
                    pose_array.poses.append(pose)
                    
                    # Publish TF frame for this detection
                    class_id = int(box.cls[0])
                    class_name = results[0].names[class_id]
                    tf_id = f"object_{class_name}_{class_id}"
                    self.publish_tf(tf_id, x_3d / 100.0, y_3d / 100.0, z_3d / 100.0)
                    
                    # Add text to image
                    cv2.rectangle(annotated_frame, (int(x1+(x2-x1)), int(y1)),
                              (int(x1+(x2-x1))+5+150, int(y1+80)), (0, 0, 0), -1)
                    cv2.putText(annotated_frame, f"x:{x_3d:.1f}cm", (int(x1+(x2-x1)+5), int(y1+20)), 
                                cv2.FONT_ITALIC, 0.6, (255, 255, 255), 2)
                    cv2.putText(annotated_frame, f"y:{y_3d:.1f}cm", (int(x1+(x2-x1)+5), int(y1+40)), 
                                cv2.FONT_ITALIC, 0.6, (255, 255, 255), 2)
                    cv2.putText(annotated_frame, f"z:{z_3d:.1f}cm", (int(x1+(x2-x1)+5), int(y1+60)), 
                                cv2.FONT_ITALIC, 0.6, (255, 255, 255), 2)
                    cv2.putText(annotated_frame, f"dis:{distance:.1f}cm", (int(x1+(x2-x1)+5), int(y1+80)), 
                                cv2.FONT_ITALIC, 0.6, (255, 255, 255), 2)
        
        # Publish pose array
        self.pose_array_pub.publish(pose_array)
        self.gui_poses_pub.publish(pose_array)
        
        # Select the appropriate view based on view_mode
        if self.view_mode == 0:  # Left view
            view_image = annotated_frame
        elif self.view_mode == 1:  # Right view
            view_image = right_img
        else:  # Depth view
            view_image = disp_color
        
        # Publish result image
        result_msg = self.bridge.cv2_to_imgmsg(view_image, "bgr8")
        self.result_image_pub.publish(result_msg)
        self.gui_detection_pub.publish(result_msg)
    
    def publish_tf(self, frame_id, x, y, z):
        """Publish TF frame for an object"""
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "camera_link"
        transform.child_frame_id = frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        node = StereoDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 