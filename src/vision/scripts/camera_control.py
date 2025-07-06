#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import time
from enum import Enum

class CameraMode(Enum):
    """Enum for different camera modes"""
    RGB = 0
    GRAYSCALE = 1
    DEPTH = 2

class CameraControl:
    """
    Core camera functionality for capturing and processing images
    """
    
    def __init__(self, camera_index=0, width=640, height=480, fps=30):
        """
        Initialize the camera control
        
        Args:
            camera_index: Camera device index
            width: Image width
            height: Image height
            fps: Frames per second
        """
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        
        # Initialize camera
        self.camera = None
        self.camera_open = False
        
        # Initialize camera mode
        self.camera_mode = CameraMode.RGB
        
        # Initialize camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Initialize frame counter
        self.frame_count = 0
        self.last_frame_time = time.time()
        self.fps_actual = 0
    
    def open_camera(self):
        """
        Open camera
        
        Returns:
            success: Whether camera was successfully opened
        """
        try:
            # Open camera
            self.camera = cv2.VideoCapture(self.camera_index)
            
            # Set camera parameters
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.camera.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Check if camera is open
            self.camera_open = self.camera.isOpened()
            
            if self.camera_open:
                rospy.loginfo(f"Camera {self.camera_index} opened successfully")
                
                # Get actual camera parameters
                self.width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.fps = self.camera.get(cv2.CAP_PROP_FPS)
                
                rospy.loginfo(f"Camera resolution: {self.width}x{self.height}, FPS: {self.fps}")
            else:
                rospy.logerr(f"Failed to open camera {self.camera_index}")
            
            return self.camera_open
            
        except Exception as e:
            rospy.logerr(f"Error opening camera: {str(e)}")
            self.camera_open = False
            return False
    
    def close_camera(self):
        """Close camera"""
        if self.camera is not None and self.camera_open:
            self.camera.release()
            self.camera_open = False
            rospy.loginfo(f"Camera {self.camera_index} closed")
    
    def set_camera_mode(self, mode):
        """
        Set camera mode
        
        Args:
            mode: Camera mode (RGB, GRAYSCALE, DEPTH)
        """
        if isinstance(mode, int):
            try:
                self.camera_mode = CameraMode(mode)
            except ValueError:
                rospy.logwarn(f"Invalid camera mode: {mode}, using RGB")
                self.camera_mode = CameraMode.RGB
        elif isinstance(mode, CameraMode):
            self.camera_mode = mode
        else:
            rospy.logwarn(f"Invalid camera mode type: {type(mode)}, using RGB")
            self.camera_mode = CameraMode.RGB
    
    def set_camera_parameters(self, camera_matrix, dist_coeffs):
        """
        Set camera parameters
        
        Args:
            camera_matrix: Camera matrix
            dist_coeffs: Distortion coefficients
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
    
    def capture_frame(self):
        """
        Capture frame from camera
        
        Returns:
            frame: Captured frame
            timestamp: Frame timestamp
        """
        if not self.camera_open:
            rospy.logwarn("Camera not open")
            return None, None
        
        try:
            # Capture frame
            ret, frame = self.camera.read()
            
            if not ret:
                rospy.logwarn("Failed to capture frame")
                return None, None
            
            # Get timestamp
            timestamp = rospy.Time.now()
            
            # Update frame counter
            self.frame_count += 1
            current_time = time.time()
            time_diff = current_time - self.last_frame_time
            
            # Update FPS every second
            if time_diff >= 1.0:
                self.fps_actual = self.frame_count / time_diff
                self.frame_count = 0
                self.last_frame_time = current_time
            
            # Process frame based on camera mode
            if self.camera_mode == CameraMode.RGB:
                # Return RGB frame
                processed_frame = frame
            elif self.camera_mode == CameraMode.GRAYSCALE:
                # Convert to grayscale
                processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            elif self.camera_mode == CameraMode.DEPTH:
                # Not implemented for regular camera
                rospy.logwarn("Depth mode not implemented for regular camera")
                processed_frame = frame
            else:
                # Default to RGB
                processed_frame = frame
            
            return processed_frame, timestamp
            
        except Exception as e:
            rospy.logerr(f"Error capturing frame: {str(e)}")
            return None, None
    
    def undistort_frame(self, frame):
        """
        Undistort frame using camera parameters
        
        Args:
            frame: Input frame
            
        Returns:
            undistorted_frame: Undistorted frame
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            # No camera parameters available, return original frame
            return frame
        
        try:
            # Undistort frame
            undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            return undistorted_frame
        except Exception as e:
            rospy.logerr(f"Error undistorting frame: {str(e)}")
            return frame
    
    def apply_effects(self, frame, effects=None):
        """
        Apply effects to frame
        
        Args:
            frame: Input frame
            effects: List of effects to apply
            
        Returns:
            processed_frame: Processed frame
        """
        if frame is None:
            return None
        
        if effects is None:
            return frame
        
        processed_frame = frame.copy()
        
        try:
            for effect in effects:
                if effect == 'blur':
                    # Apply blur
                    processed_frame = cv2.GaussianBlur(processed_frame, (5, 5), 0)
                elif effect == 'edge':
                    # Apply edge detection
                    processed_frame = cv2.Canny(processed_frame, 100, 200)
                elif effect == 'flip':
                    # Flip horizontally
                    processed_frame = cv2.flip(processed_frame, 1)
                # Add more effects as needed
            
            return processed_frame
            
        except Exception as e:
            rospy.logerr(f"Error applying effects: {str(e)}")
            return frame
    
    def draw_info(self, frame):
        """
        Draw information on frame
        
        Args:
            frame: Input frame
            
        Returns:
            annotated_frame: Frame with information drawn
        """
        if frame is None:
            return None
        
        try:
            # Create copy of frame
            annotated_frame = frame.copy()
            
            # Draw FPS
            cv2.putText(
                annotated_frame,
                f"FPS: {self.fps_actual:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )
            
            # Draw camera mode
            cv2.putText(
                annotated_frame,
                f"Mode: {self.camera_mode.name}",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )
            
            # Draw resolution
            cv2.putText(
                annotated_frame,
                f"Res: {self.width}x{self.height}",
                (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )
            
            return annotated_frame
            
        except Exception as e:
            rospy.logerr(f"Error drawing info: {str(e)}")
            return frame 