#ifndef VISION_CAMERA_NODE_H
#define VISION_CAMERA_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include "vision/camera_control.h"

namespace vision {

/**
 * @brief Camera node class for handling ROS communication
 * 
 * This class implements the node part of the node+control architecture
 * for the stereo camera system.
 */
class CameraNode {
public:
    /**
     * @brief Constructor
     * 
     * @param nh ROS node handle
     */
    CameraNode(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~CameraNode();
    
    /**
     * @brief Initialize the node
     * 
     * @return True if initialization was successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Run the node
     */
    void run();
    
    /**
     * @brief Publish camera images
     */
    void publishImages();
    
private:
    ros::NodeHandle nh_;
    std::unique_ptr<CameraControl> camera_control_;
    ros::Publisher left_image_pub_;
    ros::Publisher right_image_pub_;
    ros::Timer timer_;
    
    // ROS parameters
    int device_id_;
    int width_;
    int height_;
    int fps_;
    double publish_rate_;
    
    // Callback functions
    void timerCallback(const ros::TimerEvent& event);
};

} // namespace vision

#endif // VISION_CAMERA_NODE_H 