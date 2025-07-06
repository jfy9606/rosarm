#ifndef VISION_CAMERA_CONFIG_H
#define VISION_CAMERA_CONFIG_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace vision {

namespace camera_config {

// Camera calibration parameters
extern cv::Mat left_camera_matrix;
extern cv::Mat left_distortion;
extern cv::Mat right_camera_matrix;
extern cv::Mat right_distortion;
extern cv::Mat R;
extern cv::Mat T;
extern cv::Size size;

// Stereo rectification parameters
extern cv::Mat R1, R2;
extern cv::Mat P1, P2;
extern cv::Mat Q;
extern cv::Rect validRoi1, validRoi2;

// Rectification maps
extern cv::Mat left_map1, left_map2;
extern cv::Mat right_map1, right_map2;

// Initialize stereo rectification parameters
void initRectificationMaps();

// Load camera parameters from ROS parameter server
void loadFromParamServer(ros::NodeHandle& nh);

} // namespace camera_config

} // namespace vision

#endif // VISION_CAMERA_CONFIG_H 