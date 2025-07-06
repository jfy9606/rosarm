#include "arm_trajectory/kinematics_control.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_trajectory_kinematics");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting arm_trajectory kinematics node...");
    
    // Create the kinematics controller directly
    arm_trajectory::KinematicsControl control;
    
    // Set up services
    ros::ServiceServer fk_service = nh.advertiseService("forward_kinematics", 
        &arm_trajectory::KinematicsControl::handleForwardKinematics, &control);
    
    ros::ServiceServer ik_service = nh.advertiseService("inverse_kinematics", 
        &arm_trajectory::KinematicsControl::handleInverseKinematics, &control);
    
    ROS_INFO("Kinematics services initialized.");
    
    // Spin to handle service calls
    ros::spin();
    
    return 0;
} 