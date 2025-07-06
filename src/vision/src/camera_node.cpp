#include "vision/camera_node.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace vision {

CameraNode::CameraNode(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize parameters with default values
    device_id_ = 0;
    width_ = 1280;
    height_ = 480;
    fps_ = 30;
    publish_rate_ = 30.0;
}

CameraNode::~CameraNode() {
    // Nothing to do here
}

bool CameraNode::init() {
    // Get parameters from ROS parameter server
    nh_.param<int>("device_id", device_id_, 0);
    nh_.param<int>("width", width_, 1280);
    nh_.param<int>("height", height_, 480);
    nh_.param<int>("fps", fps_, 30);
    nh_.param<double>("publish_rate", publish_rate_, 30.0);
    
    // Create camera control
    camera_control_ = std::make_unique<CameraControl>(device_id_, width_, height_, fps_);
    
    // Initialize camera
    if (!camera_control_->init()) {
        ROS_ERROR("Failed to initialize camera");
        return false;
    }
    
    // Create publishers
    left_image_pub_ = nh_.advertise<sensor_msgs::Image>("left_camera/image_raw", 1);
    right_image_pub_ = nh_.advertise<sensor_msgs::Image>("right_camera/image_raw", 1);
    
    // Create timer for publishing images
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &CameraNode::timerCallback, this);
    
    ROS_INFO("Camera node initialized");
    return true;
}

void CameraNode::run() {
    // Nothing to do here, the timer will handle publishing
}

void CameraNode::publishImages() {
    // Capture frame
    cv::Mat left_image, right_image;
    if (!camera_control_->captureFrame(left_image, right_image)) {
        ROS_WARN_THROTTLE(5.0, "Failed to capture frame, using test images");
    }
    
    // Convert to ROS messages
    cv_bridge::CvImage left_bridge, right_bridge;
    ros::Time timestamp = ros::Time::now();
    
    // Left image
    left_bridge.encoding = "bgr8";
    left_bridge.header.stamp = timestamp;
    left_bridge.header.frame_id = "camera_left";
    left_bridge.image = left_image;
    
    // Right image
    right_bridge.encoding = "bgr8";
    right_bridge.header.stamp = timestamp;
    right_bridge.header.frame_id = "camera_right";
    right_bridge.image = right_image;
    
    // Publish images
    left_image_pub_.publish(left_bridge.toImageMsg());
    right_image_pub_.publish(right_bridge.toImageMsg());
}

void CameraNode::timerCallback(const ros::TimerEvent& event) {
    publishImages();
}

} // namespace vision

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh("~");
    
    vision::CameraNode node(nh);
    
    if (!node.init()) {
        ROS_ERROR("Failed to initialize camera node");
        return 1;
    }
    
    ROS_INFO("Camera node running");
    ros::spin();
    
    return 0;
} 