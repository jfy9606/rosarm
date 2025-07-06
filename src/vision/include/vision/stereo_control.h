#ifndef VISION_STEREO_CONTROL_H
#define VISION_STEREO_CONTROL_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "vision/detector_control.h"

namespace vision {

/**
 * @brief Stereo control class for stereo vision processing
 * 
 * This class provides functionality for stereo vision processing
 * following the control part of the node+control architecture.
 */
class StereoControl {
public:
    /**
     * @brief Constructor
     */
    StereoControl();
    
    /**
     * @brief Destructor
     */
    ~StereoControl();
    
    /**
     * @brief Initialize the stereo processor
     * 
     * @return True if initialization was successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Process a stereo image pair
     * 
     * @param left_image Left camera image
     * @param right_image Right camera image
     * @param disparity_map Output disparity map
     * @param points_3d Output 3D points
     * @return True if processing was successful, false otherwise
     */
    bool processStereo(const cv::Mat& left_image, const cv::Mat& right_image, 
                       cv::Mat& disparity_map, cv::Mat& points_3d);
    
    /**
     * @brief Compute 3D positions of detected objects
     * 
     * @param detections Input/output detection results
     * @param points_3d 3D points from stereo processing
     * @return True if computation was successful, false otherwise
     */
    bool compute3DPositions(std::vector<DetectionResult>& detections, const cv::Mat& points_3d);
    
    /**
     * @brief Set camera parameters
     * 
     * @param left_camera_matrix Left camera intrinsic matrix
     * @param left_distortion Left camera distortion coefficients
     * @param right_camera_matrix Right camera intrinsic matrix
     * @param right_distortion Right camera distortion coefficients
     * @param R Rotation matrix between cameras
     * @param T Translation vector between cameras
     * @param image_size Image size (width, height)
     */
    void setCameraParameters(const cv::Mat& left_camera_matrix, const cv::Mat& left_distortion,
                            const cv::Mat& right_camera_matrix, const cv::Mat& right_distortion,
                            const cv::Mat& R, const cv::Mat& T, const cv::Size& image_size);
    
    /**
     * @brief Check if the stereo processor is initialized
     * 
     * @return True if stereo processor is initialized, false otherwise
     */
    bool isInitialized() const { return initialized_; }
    
private:
    bool initialized_;
    
    // Camera parameters
    cv::Mat left_camera_matrix_;
    cv::Mat left_distortion_;
    cv::Mat right_camera_matrix_;
    cv::Mat right_distortion_;
    cv::Mat R_; // Rotation matrix
    cv::Mat T_; // Translation vector
    cv::Size image_size_;
    
    // Stereo rectification parameters
    cv::Mat R1_, R2_; // Rectification transforms
    cv::Mat P1_, P2_; // Projection matrices
    cv::Mat Q_; // Disparity-to-depth mapping matrix
    
    // Rectification maps
    cv::Mat left_map1_, left_map2_;
    cv::Mat right_map1_, right_map2_;
    
    // Stereo matcher
    cv::Ptr<cv::StereoSGBM> stereo_matcher_;
    
    // Helper functions
    void initStereoMatcher();
    void computeRectificationMaps();
};

} // namespace vision

#endif // VISION_STEREO_CONTROL_H 