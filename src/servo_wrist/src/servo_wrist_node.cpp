#include "servo_wrist/arm_controller.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_wrist_controller");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting servo_wrist arm controller...");
    
    // Create the arm controller
    servo_wrist::ArmController controller(nh);
    
    // Spin to handle service calls
    ros::spin();
    
    return 0;
} 