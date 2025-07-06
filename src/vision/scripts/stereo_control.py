#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import time
from enum import Enum

class ViewMode(Enum):
    """Enum for different stereo view modes"""
    LEFT = 0
    RIGHT = 1
    STEREO = 2
    DEPTH = 3

class StereoControl:
    """
    Core stereo vision functionality for depth estimation and 3D reconstruction
    """
    
    def __init__(self):
        """Initialize the stereo control"""
        # Initialize stereo parameters
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        self.dist_coeffs_left = None
        self.dist_coeffs_right = None
        self.R = None  # Rotation matrix
        self.T = None  # Translation vector
        self.E = None  # Essential matrix
        self.F = None  # Fundamental matrix
        
        # Initialize rectification parameters
        self.R1 = None
        self.R2 = None
        self.P1 = None
        self.P2 = None
        self.Q = None
        self.roi_left = None
        self.roi_right = None
        self.map_left_x = None
        self.map_left_y = None
        self.map_right_x = None
        self.map_right_y = None
        
        # Initialize stereo matcher
        self.stereo_matcher = None
        self.wls_filter = None
        
        # Initialize view mode
        self.view_mode = ViewMode.LEFT
        
        # Initialize stereo parameters
        self.min_disparity = 0
        self.num_disparities = 64
        self.block_size = 15
        self.uniqueness_ratio = 15
        self.speckle_window_size = 100
        self.speckle_range = 32
        self.disp12_max_diff = 1
        self.p1 = 8 * 3 * self.block_size ** 2
        self.p2 = 32 * 3 * self.block_size ** 2
        
        # Initialize stereo matcher
        self.init_stereo_matcher()
    
    def init_stereo_matcher(self):
        """Initialize stereo matcher"""
        # Create stereo matcher
        self.stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=self.min_disparity,
            numDisparities=self.num_disparities,
            blockSize=self.block_size,
            P1=self.p1,
            P2=self.p2,
            disp12MaxDiff=self.disp12_max_diff,
            uniquenessRatio=self.uniqueness_ratio,
            speckleWindowSize=self.speckle_window_size,
            speckleRange=self.speckle_range,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        # Create WLS filter for post-processing
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo_matcher)
        self.wls_filter.setLambda(8000.0)
        self.wls_filter.setSigmaColor(1.5)
        
        # Create right matcher for WLS filter
        right_matcher = cv2.ximgproc.createRightMatcher(self.stereo_matcher)
        self.right_matcher = right_matcher
    
    def set_calibration_params(self, camera_matrix_left, camera_matrix_right, 
                              dist_coeffs_left, dist_coeffs_right, 
                              R, T, E, F, image_size):
        """
        Set stereo calibration parameters
        
        Args:
            camera_matrix_left: Left camera matrix
            camera_matrix_right: Right camera matrix
            dist_coeffs_left: Left distortion coefficients
            dist_coeffs_right: Right distortion coefficients
            R: Rotation matrix
            T: Translation vector
            E: Essential matrix
            F: Fundamental matrix
            image_size: Image size (width, height)
        """
        self.camera_matrix_left = camera_matrix_left
        self.camera_matrix_right = camera_matrix_right
        self.dist_coeffs_left = dist_coeffs_left
        self.dist_coeffs_right = dist_coeffs_right
        self.R = R
        self.T = T
        self.E = E
        self.F = F
        
        # Compute rectification parameters
        self.R1, self.R2, self.P1, self.P2, self.Q, self.roi_left, self.roi_right = cv2.stereoRectify(
            self.camera_matrix_left, self.dist_coeffs_left,
            self.camera_matrix_right, self.dist_coeffs_right,
            image_size, self.R, self.T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.9
        )
        
        # Compute rectification maps
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            self.camera_matrix_left, self.dist_coeffs_left, self.R1, self.P1,
            image_size, cv2.CV_32FC1
        )
        
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            self.camera_matrix_right, self.dist_coeffs_right, self.R2, self.P2,
            image_size, cv2.CV_32FC1
        )
    
    def set_view_mode(self, mode):
        """
        Set view mode
        
        Args:
            mode: View mode (LEFT, RIGHT, STEREO, DEPTH)
        """
        if isinstance(mode, int):
            try:
                self.view_mode = ViewMode(mode)
            except ValueError:
                rospy.logwarn(f"Invalid view mode: {mode}, using LEFT")
                self.view_mode = ViewMode.LEFT
        elif isinstance(mode, ViewMode):
            self.view_mode = mode
        else:
            rospy.logwarn(f"Invalid view mode type: {type(mode)}, using LEFT")
            self.view_mode = ViewMode.LEFT
    
    def rectify_images(self, left_image, right_image):
        """
        Rectify stereo images
        
        Args:
            left_image: Left camera image
            right_image: Right camera image
            
        Returns:
            left_rect: Rectified left image
            right_rect: Rectified right image
        """
        if self.map_left_x is None or self.map_left_y is None or self.map_right_x is None or self.map_right_y is None:
            rospy.logwarn("Rectification maps not initialized, returning original images")
            return left_image, right_image
        
        # Rectify images
        left_rect = cv2.remap(left_image, self.map_left_x, self.map_left_y, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_image, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)
        
        return left_rect, right_rect
    
    def compute_disparity(self, left_rect, right_rect):
        """
        Compute disparity map
        
        Args:
            left_rect: Rectified left image
            right_rect: Rectified right image
            
        Returns:
            disparity: Disparity map
            filtered_disparity: Filtered disparity map
        """
        if self.stereo_matcher is None:
            rospy.logwarn("Stereo matcher not initialized")
            return None, None
        
        # Convert to grayscale if needed
        if len(left_rect.shape) == 3:
            left_gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left_rect
            right_gray = right_rect
        
        # Compute disparity maps
        disparity_left = self.stereo_matcher.compute(left_gray, right_gray)
        disparity_right = self.right_matcher.compute(right_gray, left_gray)
        
        # Filter disparity map
        filtered_disparity = self.wls_filter.filter(disparity_left, left_rect, disparity_map_right=disparity_right)
        
        # Normalize disparity maps for visualization
        disparity_normalized = cv2.normalize(disparity_left, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        filtered_disparity_normalized = cv2.normalize(filtered_disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        
        return disparity_normalized, filtered_disparity_normalized
    
    def compute_point_cloud(self, disparity, left_rect):
        """
        Compute 3D point cloud from disparity map
        
        Args:
            disparity: Disparity map
            left_rect: Rectified left image
            
        Returns:
            points: 3D points
            colors: Point colors
        """
        if self.Q is None:
            rospy.logwarn("Q matrix not initialized")
            return None, None
        
        # Convert disparity to float32
        disparity_float = disparity.astype(np.float32) / 16.0
        
        # Reproject to 3D
        points_3d = cv2.reprojectImageTo3D(disparity_float, self.Q)
        
        # Get colors from left image
        colors = cv2.cvtColor(left_rect, cv2.COLOR_BGR2RGB) if len(left_rect.shape) == 3 else np.dstack([left_rect, left_rect, left_rect])
        
        # Filter out invalid points (too far or too close)
        mask = (disparity_float > self.min_disparity) & (disparity_float < self.min_disparity + self.num_disparities)
        
        # Apply mask
        points = points_3d[mask]
        colors = colors[mask]
        
        return points, colors
    
    def get_3d_point(self, disparity, x, y):
        """
        Get 3D point from disparity map at given pixel coordinates
        
        Args:
            disparity: Disparity map
            x: X coordinate
            y: Y coordinate
            
        Returns:
            point: 3D point (x, y, z)
        """
        if self.Q is None:
            rospy.logwarn("Q matrix not initialized")
            return None
        
        # Check if coordinates are valid
        if x < 0 or y < 0 or x >= disparity.shape[1] or y >= disparity.shape[0]:
            return None
        
        # Get disparity value
        disp = disparity[y, x]
        
        # Check if disparity is valid
        if disp <= self.min_disparity or disp >= self.min_disparity + self.num_disparities:
            return None
        
        # Convert disparity to float32
        disp_float = disp / 16.0
        
        # Reproject to 3D
        point = cv2.perspectiveTransform(np.array([[[x, y, disp_float]]]), self.Q)[0, 0]
        
        return point
    
    def process_stereo_images(self, left_image, right_image):
        """
        Process stereo images based on current view mode
        
        Args:
            left_image: Left camera image
            right_image: Right camera image
            
        Returns:
            output_image: Processed image based on view mode
            disparity: Disparity map (if computed)
        """
        # Check if images are valid
        if left_image is None or right_image is None:
            rospy.logwarn("Invalid stereo images")
            return None, None
        
        # Rectify images
        left_rect, right_rect = self.rectify_images(left_image, right_image)
        
        # Initialize disparity
        disparity = None
        filtered_disparity = None
        
        # Process based on view mode
        if self.view_mode == ViewMode.LEFT:
            output_image = left_rect
        elif self.view_mode == ViewMode.RIGHT:
            output_image = right_rect
        elif self.view_mode == ViewMode.STEREO:
            # Create anaglyph stereo image (red-cyan)
            output_image = np.zeros_like(left_rect)
            if len(left_rect.shape) == 3:
                output_image[:, :, 0] = right_rect[:, :, 0]  # Red channel from right image
                output_image[:, :, 1] = left_rect[:, :, 1]   # Green channel from left image
                output_image[:, :, 2] = left_rect[:, :, 2]   # Blue channel from left image
            else:
                # For grayscale images
                output_image = np.dstack([right_rect, left_rect, left_rect])
        elif self.view_mode == ViewMode.DEPTH:
            # Compute disparity map
            disparity, filtered_disparity = self.compute_disparity(left_rect, right_rect)
            
            # Apply colormap to disparity
            if filtered_disparity is not None:
                output_image = cv2.applyColorMap(filtered_disparity, cv2.COLORMAP_JET)
            else:
                output_image = cv2.applyColorMap(disparity, cv2.COLORMAP_JET)
        else:
            rospy.logwarn(f"Unsupported view mode: {self.view_mode}")
            output_image = left_rect
        
        return output_image, filtered_disparity if filtered_disparity is not None else disparity 

# Add this at the end of the file to properly export the classes
__all__ = ['StereoControl', 'ViewMode'] 