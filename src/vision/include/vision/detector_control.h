#ifndef VISION_DETECTOR_CONTROL_H
#define VISION_DETECTOR_CONTROL_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace vision {

/**
 * @brief Detection result structure
 */
struct DetectionResult {
    int class_id;
    std::string class_name;
    float confidence;
    cv::Rect bbox;
    float distance;
    cv::Point3f position_3d;
};

/**
 * @brief Detector control class for object detection
 * 
 * This class provides functionality to detect objects in images
 * following the control part of the node+control architecture.
 */
class DetectorControl {
public:
    /**
     * @brief Constructor
     * 
     * @param model_path Path to the model file
     * @param conf_threshold Confidence threshold for detections
     */
    DetectorControl(const std::string& model_path = "yolo11n.pt", float conf_threshold = 0.25f);
    
    /**
     * @brief Destructor
     */
    ~DetectorControl();
    
    /**
     * @brief Initialize the detector
     * 
     * @return True if initialization was successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Detect objects in an image
     * 
     * @param image Input image
     * @param results Output detection results
     * @return True if detection was successful, false otherwise
     */
    bool detect(const cv::Mat& image, std::vector<DetectionResult>& results);
    
    /**
     * @brief Draw detection results on an image
     * 
     * @param image Input/output image
     * @param results Detection results
     */
    void drawDetections(cv::Mat& image, const std::vector<DetectionResult>& results);
    
    /**
     * @brief Set the confidence threshold
     * 
     * @param threshold New confidence threshold
     */
    void setConfidenceThreshold(float threshold) { conf_threshold_ = threshold; }
    
    /**
     * @brief Get the confidence threshold
     * 
     * @return Current confidence threshold
     */
    float getConfidenceThreshold() const { return conf_threshold_; }
    
    /**
     * @brief Check if the detector is initialized
     * 
     * @return True if detector is initialized, false otherwise
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief Enable or disable the detector
     * 
     * @param enabled True to enable, false to disable
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }
    
    /**
     * @brief Check if the detector is enabled
     * 
     * @return True if detector is enabled, false otherwise
     */
    bool isEnabled() const { return enabled_; }
    
private:
    std::string model_path_;
    float conf_threshold_;
    bool initialized_;
    bool enabled_;
    
    // Private implementation details would go here
    // In a real implementation, this would include the actual detector model
};

} // namespace vision

#endif // VISION_DETECTOR_CONTROL_H 