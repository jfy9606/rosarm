#ifndef VISION_STEREO_NODE_H
#define VISION_STEREO_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include "vision/stereo_control.h"
#include "vision/detector_control.h"

namespace vision {

/**
 * @brief Stereo node class for handling ROS communication
 * 
 * This class implements the node part of the node+control architecture
 * for the stereo vision system.
 */
class StereoNode {
public:
    /**
     * @brief Constructor
     * 
     * @param nh ROS node handle
     */
    StereoNode(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~StereoNode();
    
    /**
     * @brief Initialize the node
     * 
     * @return True if initialization was successful, false otherwise
     */
    bool init();
    
private:
    ros::NodeHandle nh_;
    std::unique_ptr<StereoControl> stereo_control_;
    std::unique_ptr<DetectorControl> detector_control_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // ROS subscribers
    ros::Subscriber left_image_sub_;
    ros::Subscriber right_image_sub_;
    ros::Subscriber camera_sub_;
    ros::Subscriber view_mode_sub_;
    
    // ROS publishers
    ros::Publisher detection_pub_;
    ros::Publisher result_image_pub_;
    ros::Publisher pose_array_pub_;
    ros::Publisher left_image_pub_;
    ros::Publisher right_image_pub_;
    ros::Publisher gui_detection_pub_;
    ros::Publisher gui_poses_pub_;
    
    // ROS parameters
    std::string model_path_;
    bool use_split_feed_;
    std::string left_image_topic_;
    std::string right_image_topic_;
    bool auto_download_;
    
    // Current images
    cv::Mat left_image_;
    cv::Mat right_image_;
    
    // Current view mode (0=left, 1=right)
    int view_mode_;
    
    // Callback functions
    void leftImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rightImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void viewModeCallback(const std_msgs::HeaderConstPtr& msg);
    
    // Processing functions
    void processStereo();
    void publishTF(const std::string& frame_id, float x, float y, float z);
};

} // namespace vision

#endif // VISION_STEREO_NODE_H 