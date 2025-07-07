#include <ros/ros.h>
#include <servo/SerControl.h>  // Replace with the actual message type

#include "SCServo.h"

SMS_STS sm_st;

//设置波特率
int baud_rate = 1000000;

//定义了一个回调函数msgCallback，当接收到指定话题的消息时，这个函数会被调用
void msgCallback(const servo::SerControl::ConstPtr& msg) {
    // Process the received message and call WritePosEx
    int servo_id = msg->servo_id;
    int target_position = msg->target_position;
    int velocity = msg->velocity;
    int acceleration = msg->acceleration;

    sm_st.WritePosEx(servo_id, target_position, velocity, acceleration);

    // Optionally, print the received values
    ROS_INFO("Received msg - Servo ID: %d, Target Position: %d, Velocity: %d, Acceleration: %d", 
              servo_id, target_position, velocity, acceleration);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        ROS_ERROR("Usage: %s <serial_port>", argv[0]);
        return 1;
    }
    
    // Initialize ROS
    ros::init(argc, argv, "servo_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 获取串口设备路径，如果命令行参数提供，则使用命令行参数
    std::string serial_port = argv[1];
    ROS_INFO("Using serial port: %s", serial_port.c_str());
    
    // 尝试打开串口并初始化舵机
    ROS_INFO("Serial speed %d", baud_rate);
    if (!sm_st.begin(baud_rate, serial_port.c_str())) {
        ROS_ERROR("Failed to init sms/sts motor on port %s!", serial_port.c_str());
        return 1;
    }
    
    ROS_INFO("SMS/STS motor initialized successfully on port %s", serial_port.c_str());

    // Subscribe to the topic that will receive the servo control message
    ros::Subscriber sub = nh.subscribe("servo_control_topic", 10, msgCallback);

    // ROS main loop
    ros::spin();

    // Cleanup and end
    sm_st.end();
    return 0;
}
