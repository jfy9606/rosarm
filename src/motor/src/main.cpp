#include <motor/motor_node.h>
#include <ros/ros.h>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_node");
    
    // 获取串口设备名称
    std::string port = "/dev/ttyUSB0";  // 默认串口
    int baudrate = 115200;  // 默认波特率
    
    // 从参数服务器获取参数
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("port", port, port);
    private_nh.param("baudrate", baudrate, baudrate);
    
    // 创建电机控制节点
    motor::MotorNode node(port, baudrate);
    
    // 进入ROS循环
    ros::spin();
    
    return 0;
} 