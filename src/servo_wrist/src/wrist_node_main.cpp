#include <servo_wrist/arm_node.h>
#include <ros/ros.h>
#include <memory>

/**
 * @brief Main entry point for the servo wrist controller node
 * 
 * This node initializes and runs the arm controller
 * in a unified node.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_wrist_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("Starting servo_wrist controller node...");
    
    // 获取参数
    std::string arm_port = "/dev/ttyACM0";
    int arm_baudrate = 115200;
    
    // 从参数服务器获取参数
    pnh.param("arm_port", arm_port, arm_port);
    pnh.param("arm_baudrate", arm_baudrate, arm_baudrate);
    
    // 创建控制器
    servo_wrist::ArmNode arm_node(nh);
    
    ROS_INFO("Arm controller initialized. Ready to control the robot arm.");
    ROS_INFO("Note: Wrist and vacuum nodes should be started separately.");
    
    // 处理ROS消息
    ros::spin();
    
    return 0;
} 