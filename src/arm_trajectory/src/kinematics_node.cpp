#include "arm_trajectory/kinematics_service.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_trajectory_kinematics");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting arm_trajectory kinematics service...");
    
    // Create the kinematics service
    arm_trajectory::KinematicsService service(nh);
    
    // Spin to handle service calls
    ros::spin();
    
    return 0;
} 