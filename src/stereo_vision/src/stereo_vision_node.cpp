#include "stereo_vision/vision_processor.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_vision_processor");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting stereo vision processor...");
    
    // Create the vision processor
    stereo_vision::VisionProcessor processor(nh);
    
    // Spin to handle callbacks
    ros::spin();
    
    return 0;
} 