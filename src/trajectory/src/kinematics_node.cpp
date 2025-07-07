#include "trajectory/kinematics_control.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting arm_trajectory kinematics node...");
    
    // Create the kinematics controller directly
    trajectory::KinematicsControl control;
    
    // Set up services
    ros::ServiceServer fk_service = nh.advertiseService("forward_kinematics", 
        &trajectory::KinematicsControl::handleForwardKinematics, &control);
    
    ros::ServiceServer ik_service = nh.advertiseService("inverse_kinematics", 
        &trajectory::KinematicsControl::handleInverseKinematics, &control);
    
    ROS_INFO("Kinematics services initialized.");
    
    // Spin to handle service calls
    ros::spin();
    
    return 0;
} 