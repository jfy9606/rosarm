#ifndef STEREO_VISION_PROCESSOR_H
#define STEREO_VISION_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "stereo_vision/DetectionControl.h"
#include "stereo_vision/SetViewMode.h"

namespace stereo_vision {

struct DetectedObject {
    std::string id;              // Object ID
    std::string class_name;      // Object class name
    std::string type;            // Object type
    double confidence;           // Detection confidence
    cv::Rect bbox;               // Bounding box in image
    geometry_msgs::Pose pose;    // Object pose in space
    cv::Scalar color;            // Display color
    double x;                    // X coordinate (cm)
    double y;                    // Y coordinate (cm)
    double z;                    // Z coordinate (cm)
};

class VisionProcessor {
public:
    VisionProcessor(ros::NodeHandle& nh);
    ~VisionProcessor();

    // Image processing methods
    void detectObjects();
    cv::Mat computeDepthMap(const cv::Mat& left_image, const cv::Mat& right_image);
    
    // 3D conversion methods
    cv::Point3f imagePointTo3D(const cv::Point2f& image_point, float depth = 1.0);
    cv::Point2f point3DToImage(const cv::Point3f& point_3d);
    float getDepthAtPoint(const cv::Point2f& image_point);
    
    // Camera calibration
    void loadCameraParameters();
    
    // Control methods
    void enableObjectDetection(bool enable);
    void setCameraViewMode(int mode); // 0=left, 1=right, 2=depth
    
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    
    // Publishers
    image_transport::Publisher current_view_pub_;
    image_transport::Publisher depth_image_pub_;
    image_transport::Publisher detection_image_pub_;
    ros::Publisher detection_poses_pub_;
    
    // Subscribers
    image_transport::Subscriber stereo_merged_sub_;
    image_transport::Subscriber python_detection_image_sub_;
    ros::Subscriber python_detection_poses_sub_;
    message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;
    
    // Synchronizer for stereo pair
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> StereoSyncPolicy;
    typedef message_filters::Synchronizer<StereoSyncPolicy> StereoSynchronizer;
    boost::shared_ptr<StereoSynchronizer> stereo_sync_;
    
    // Services
    ros::ServiceServer detection_control_service_;
    ros::ServiceServer view_mode_service_;
    
    // Camera parameters
    cv::Mat camera_matrix_;
    cv::Mat camera_extrinsic_;
    double stereo_baseline_;
    
    // Image processing state
    cv::Mat left_image_;
    cv::Mat right_image_;
    cv::Mat current_image_;
    cv::Mat depth_image_;
    int view_mode_;
    bool is_camera_available_;
    bool detection_enabled_;
    int stereo_camera_error_count_;
    std::string stereo_method_;
    
    // Detected objects
    std::vector<DetectedObject> detected_objects_;
    
    // Callback methods
    void stereoMergedCallback(const sensor_msgs::ImageConstPtr& msg);
    void stereoPairCallback(const sensor_msgs::ImageConstPtr& left_msg, 
                           const sensor_msgs::ImageConstPtr& right_msg);
    void pythonDetectionImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void pythonDetectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void handleCameraError(const std::string& error_msg);
    void publishCurrentView();
    void createPlaceholderImage();
    
    // Camera extrinsic update
    void updateCameraExtrinsic(const geometry_msgs::Pose& camera_pose);
    
    // Camera utility
    bool findAvailableCamera();
    
    // Service handlers
    bool handleDetectionControl(stereo_vision::DetectionControl::Request& req, 
                              stereo_vision::DetectionControl::Response& res);
    bool handleSetViewMode(stereo_vision::SetViewMode::Request& req, 
                         stereo_vision::SetViewMode::Response& res);
};

} // namespace stereo_vision

#endif // STEREO_VISION_PROCESSOR_H 