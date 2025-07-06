#include "vision/camera_control.h"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace vision {

CameraControl::CameraControl(int device_id, int width, int height, int fps)
    : device_id_(device_id), width_(width), height_(height), fps_(fps), initialized_(false) {
}

CameraControl::~CameraControl() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}

bool CameraControl::init() {
    // Try to open the camera
    cap_ = cv::VideoCapture(device_id_);
    
    if (!cap_.isOpened()) {
        std::cerr << "Failed to open camera device " << device_id_ << std::endl;
        return false;
    }
    
    // Set camera properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    
    // Set pixel format to MJPEG for better performance
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    
    // Check if the camera is actually working by reading a test frame
    cv::Mat test_frame;
    if (!cap_.read(test_frame) || test_frame.empty()) {
        std::cerr << "Failed to read test frame from camera" << std::endl;
        cap_.release();
        return false;
    }
    
    // Get actual camera properties
    int actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    int actual_fps = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));
    
    std::cout << "Camera initialized with resolution: " << actual_width << "x" << actual_height 
              << ", FPS: " << actual_fps << std::endl;
    
    initialized_ = true;
    return true;
}

bool CameraControl::captureFrame(cv::Mat& left_image, cv::Mat& right_image) {
    if (!initialized_ || !cap_.isOpened()) {
        if (!reopenCamera()) {
            // If reopening fails, create test images
            cv::Mat test_image = createTestImage(width_, height_);
            int mid = test_image.cols / 2;
            left_image = test_image(cv::Rect(0, 0, mid, test_image.rows)).clone();
            right_image = test_image(cv::Rect(mid, 0, test_image.cols - mid, test_image.rows)).clone();
            return false;
        }
    }
    
    // Capture a frame
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
        std::cerr << "Failed to capture frame" << std::endl;
        initialized_ = false;
        
        // Create test images
        cv::Mat test_image = createTestImage(width_, height_);
        int mid = test_image.cols / 2;
        left_image = test_image(cv::Rect(0, 0, mid, test_image.rows)).clone();
        right_image = test_image(cv::Rect(mid, 0, test_image.cols - mid, test_image.rows)).clone();
        return false;
    }
    
    // Split the frame into left and right images
    int mid = frame.cols / 2;
    left_image = frame(cv::Rect(0, 0, mid, frame.rows)).clone();
    right_image = frame(cv::Rect(mid, 0, frame.cols - mid, frame.rows)).clone();
    
    return true;
}

cv::Mat CameraControl::createTestImage(int width, int height) {
    // Create a black image
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    
    // Draw some shapes
    // Red circle
    cv::circle(image, cv::Point(width/4, height/2), 50, cv::Scalar(0, 0, 255), -1);
    
    // Green rectangle
    cv::rectangle(image, cv::Point(width/2, height/4), 
                 cv::Point(3*width/4, 3*height/4), cv::Scalar(0, 255, 0), -1);
    
    // Blue triangle
    std::vector<cv::Point> pts;
    pts.push_back(cv::Point(3*width/4, height/4));
    pts.push_back(cv::Point(7*width/8, height/2));
    pts.push_back(cv::Point(3*width/4, 3*height/4));
    cv::fillPoly(image, std::vector<std::vector<cv::Point>>{pts}, cv::Scalar(255, 0, 0));
    
    // Add text
    cv::putText(image, "Test Image", cv::Point(20, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(image, "No Camera Available", cv::Point(20, 70), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    // Add timestamp
    std::string timestamp = std::to_string(cv::getTickCount() / cv::getTickFrequency());
    cv::putText(image, "Time: " + timestamp, cv::Point(20, height-20), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    return image;
}

bool CameraControl::reopenCamera() {
    // Release the current camera
    if (cap_.isOpened()) {
        cap_.release();
    }
    
    // Try to reopen the camera
    cap_ = cv::VideoCapture(device_id_);
    
    if (!cap_.isOpened()) {
        std::cerr << "Failed to reopen camera device " << device_id_ << std::endl;
        return false;
    }
    
    // Set camera properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    
    // Check if the camera is actually working by reading a test frame
    cv::Mat test_frame;
    if (!cap_.read(test_frame) || test_frame.empty()) {
        std::cerr << "Failed to read test frame after reopening camera" << std::endl;
        cap_.release();
        return false;
    }
    
    initialized_ = true;
    return true;
}

} // namespace vision 