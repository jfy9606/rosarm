#ifndef VISION_CAMERA_CONTROL_H
#define VISION_CAMERA_CONTROL_H

#include <string>
#include <opencv2/opencv.hpp>

namespace vision {

/**
 * @brief Camera control class for handling stereo camera operations
 * 
 * This class provides functionality to control and access stereo camera hardware
 * following the control part of the node+control architecture.
 */
class CameraControl {
public:
    /**
     * @brief Constructor
     * 
     * @param device_id Camera device ID
     * @param width Image width
     * @param height Image height
     * @param fps Frame rate
     */
    CameraControl(int device_id = 0, int width = 1280, int height = 480, int fps = 30);
    
    /**
     * @brief Destructor
     */
    ~CameraControl();
    
    /**
     * @brief Initialize the camera
     * 
     * @return True if initialization was successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Capture a frame from the camera
     * 
     * @param left_image Output left camera image
     * @param right_image Output right camera image
     * @return True if capture was successful, false otherwise
     */
    bool captureFrame(cv::Mat& left_image, cv::Mat& right_image);
    
    /**
     * @brief Create a test image when camera is not available
     * 
     * @param width Image width
     * @param height Image height
     * @return Generated test image
     */
    cv::Mat createTestImage(int width, int height);
    
    /**
     * @brief Check if the camera is initialized
     * 
     * @return True if camera is initialized, false otherwise
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Get the camera width
     * 
     * @return Camera width in pixels
     */
    int getWidth() const { return width_; }
    
    /**
     * @brief Get the camera height
     * 
     * @return Camera height in pixels
     */
    int getHeight() const { return height_; }
    
private:
    int device_id_;
    int width_;
    int height_;
    int fps_;
    bool initialized_;
    cv::VideoCapture cap_;
    
    // Private methods
    bool reopenCamera();
};

} // namespace vision

#endif // VISION_CAMERA_CONTROL_H 