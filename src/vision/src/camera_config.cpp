#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace vision {

// Camera calibration parameters
namespace camera_config {

// Left camera intrinsic matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3) << 
    708.021578084636, 0.0, 317.045973456068,
    0.0, 708.407895199159, 246.255369290019,
    0.0, 0.0, 1.0);

// Left camera distortion coefficients [k1, k2, p1, p2, k3]
cv::Mat left_distortion = (cv::Mat_<double>(1, 5) << 
    -0.423530558996000, 0.197897793816823, -0.000431572543977505, 
    -0.00159243587269625, 0.0673104450517612);

// Right camera intrinsic matrix
cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3) << 
    696.214795245276, 0.0, 332.575401506334,
    0.0, 697.690484333038, 254.476510158679,
    0.0, 0.0, 1.0);

// Right camera distortion coefficients [k1, k2, p1, p2, k3]
cv::Mat right_distortion = (cv::Mat_<double>(1, 5) << 
    -0.446997760685779, 0.334596216137961, -0.00156518872343852, 
    0.000556305187002041, -0.245497405770906);

// Rotation matrix (describes right camera rotation relative to left camera)
cv::Mat R = (cv::Mat_<double>(3, 3) << 
    0.999997797146582, -0.000841951628713670, 0.00192271148090808,
    0.000845960639244048, 0.999997468098366, -0.00208522120015922,
    -0.00192095095740603, 0.00208684314495605, 0.999995977508464);

// Translation vector (describes right camera position relative to left camera, in mm)
cv::Mat T = (cv::Mat_<double>(3, 1) << 
    -60.1855543356205, 0.0302533537233422, -0.307971817904289);

// Image size (width, height)
cv::Size size(640, 480);

// Stereo rectification parameters
cv::Mat R1, R2;      // Rectification transforms
cv::Mat P1, P2;      // Projection matrices
cv::Mat Q;           // Disparity-to-depth mapping matrix
cv::Rect validRoi1, validRoi2;  // Valid pixel ROI

// Rectification maps
cv::Mat left_map1, left_map2;
cv::Mat right_map1, right_map2;

// Initialize stereo rectification parameters
void initRectificationMaps() {
    // Compute stereo rectification
    cv::stereoRectify(
        left_camera_matrix, left_distortion,
        right_camera_matrix, right_distortion,
        size, R, T, R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY, 0, size, &validRoi1, &validRoi2);
    
    // Compute rectification maps
    cv::initUndistortRectifyMap(
        left_camera_matrix, left_distortion, R1, P1, size, CV_16SC2,
        left_map1, left_map2);
    
    cv::initUndistortRectifyMap(
        right_camera_matrix, right_distortion, R2, P2, size, CV_16SC2,
        right_map1, right_map2);
    
    ROS_INFO("Stereo rectification maps initialized");
}

// Load camera parameters from ROS parameter server
void loadFromParamServer(ros::NodeHandle& nh) {
    bool params_loaded = false;
    
    // Try to load parameters from the parameter server
    if (nh.hasParam("/vision/camera_config/left_camera_matrix")) {
        // Parameters exist, try to load them
        std::vector<double> left_matrix, right_matrix, left_dist, right_dist;
        std::vector<double> r_matrix, t_vector;
        int width, height;
        
        nh.getParam("/vision/camera_config/left_camera_matrix", left_matrix);
        nh.getParam("/vision/camera_config/right_camera_matrix", right_matrix);
        nh.getParam("/vision/camera_config/left_distortion", left_dist);
        nh.getParam("/vision/camera_config/right_distortion", right_dist);
        nh.getParam("/vision/camera_config/R", r_matrix);
        nh.getParam("/vision/camera_config/T", t_vector);
        nh.getParam("/vision/camera_config/width", width);
        nh.getParam("/vision/camera_config/height", height);
        
        // Convert to OpenCV matrices
        if (left_matrix.size() == 9 && right_matrix.size() == 9 &&
            left_dist.size() == 5 && right_dist.size() == 5 &&
            r_matrix.size() == 9 && t_vector.size() == 3) {
            
            left_camera_matrix = cv::Mat(3, 3, CV_64F, left_matrix.data()).clone();
            right_camera_matrix = cv::Mat(3, 3, CV_64F, right_matrix.data()).clone();
            left_distortion = cv::Mat(1, 5, CV_64F, left_dist.data()).clone();
            right_distortion = cv::Mat(1, 5, CV_64F, right_dist.data()).clone();
            R = cv::Mat(3, 3, CV_64F, r_matrix.data()).clone();
            T = cv::Mat(3, 1, CV_64F, t_vector.data()).clone();
            size = cv::Size(width, height);
            
            params_loaded = true;
            ROS_INFO("Camera parameters loaded from parameter server");
        }
    }
    
    if (!params_loaded) {
        ROS_WARN("Failed to load camera parameters from parameter server, using defaults");
    }
    
    // Initialize rectification maps
    initRectificationMaps();
    
    // Store parameters on the parameter server for future use
    std::vector<double> left_matrix(left_camera_matrix.begin<double>(), left_camera_matrix.end<double>());
    std::vector<double> right_matrix(right_camera_matrix.begin<double>(), right_camera_matrix.end<double>());
    std::vector<double> left_dist(left_distortion.begin<double>(), left_distortion.end<double>());
    std::vector<double> right_dist(right_distortion.begin<double>(), right_distortion.end<double>());
    std::vector<double> r_matrix(R.begin<double>(), R.end<double>());
    std::vector<double> t_vector(T.begin<double>(), T.end<double>());
    
    nh.setParam("/vision/camera_config/left_camera_matrix", left_matrix);
    nh.setParam("/vision/camera_config/right_camera_matrix", right_matrix);
    nh.setParam("/vision/camera_config/left_distortion", left_dist);
    nh.setParam("/vision/camera_config/right_distortion", right_dist);
    nh.setParam("/vision/camera_config/R", r_matrix);
    nh.setParam("/vision/camera_config/T", t_vector);
    nh.setParam("/vision/camera_config/width", size.width);
    nh.setParam("/vision/camera_config/height", size.height);
}

} // namespace camera_config

} // namespace vision 