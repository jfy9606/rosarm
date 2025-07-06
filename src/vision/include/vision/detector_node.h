#ifndef VISION_DETECTOR_NODE_H
#define VISION_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include "vision/detector_control.h"

namespace vision {

/**
 * @brief Detector node class for handling ROS communication
 * 
 * This class implements the node part of the node+control architecture
 * for the object detection system.
 */
class DetectorNode {
public:
    /**
     * @brief Constructor
     * 
     * @param nh ROS node handle
     */
    DetectorNode(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~DetectorNode();
    
    /**
     * @brief Initialize the node
     * 
     * @return True if initialization was successful, false otherwise
     */
    bool init();
    
private:
    ros::NodeHandle nh_;
    std::unique_ptr<DetectorControl> detector_control_;
    
    // ROS subscribers
    ros::Subscriber image_sub_;
    ros::Subscriber control_sub_;
    
    // ROS publishers
    ros::Publisher detection_image_pub_;
    ros::Publisher poses_pub_;
    ros::Publisher status_pub_;
    
    // ROS services
    ros::ServiceServer enable_service_;
    
    // ROS parameters
    std::string model_path_;
    float conf_threshold_;
    std::string image_topic_;
    bool detection_enabled_;
    bool auto_download_;
    
    // Callback functions
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void controlCallback(const std_msgs::BoolConstPtr& msg);
    bool enableServiceCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    
    // Helper functions
    void publishStatus();
};

} // namespace vision

#endif // VISION_DETECTOR_NODE_H 