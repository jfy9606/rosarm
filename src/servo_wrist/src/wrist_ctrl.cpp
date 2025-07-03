#include <ros/ros.h>
#include <std_msgs/String.h>
#include <servo_wrist/SerControl.h>
#include <string>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

// 包含SCServo库
#include "SCServo.h"

class ServoController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber servo_sub_;
    std::string port_;
    int baudrate_;
    bool initialized_;
    SMS_STS sms_sts;  // 使用SMS_STS类

public:
    ServoController(const std::string& port, int baudrate = 1000000) 
        : port_(port), baudrate_(baudrate), initialized_(false) {
        
        // 订阅舵机控制话题
        servo_sub_ = nh_.subscribe("/servo_control_topic", 10, &ServoController::servoCallback, this);
        
        // 初始化舵机控制器
        if (!initServo()) {
            ROS_ERROR("Failed to init sms/sts motor on port %s!", port_.c_str());
            return;
        }
        
        initialized_ = true;
        ROS_INFO("SMS/STS motor initialized successfully on port %s", port_.c_str());
    }
    
    ~ServoController() {
        // 关闭舵机控制器
        if (initialized_) {
            sms_sts.end();
        }
    }
    
    bool initServo() {
        ROS_INFO("Using serial port: %s", port_.c_str());
        ROS_INFO("Serial speed %d", baudrate_);
        
        // 初始化舵机控制器，注意参数顺序：波特率在前，串口名在后
        if (sms_sts.begin(baudrate_, port_.c_str())) {
            return true;
        }
        
        return false;
    }
    
    void servoCallback(const servo_wrist::SerControl::ConstPtr& msg) {
        if (!initialized_) {
            ROS_WARN("Servo controller not initialized!");
            return;
        }
        
        uint8_t servo_id = msg->servo_id;
        uint16_t position = msg->target_position;
        uint16_t time = msg->velocity;  // 使用velocity字段作为时间
        uint8_t acc = msg->acceleration;
        
        ROS_INFO("Moving servo ID %d to position %d with time %d and acc %d", 
                 servo_id, position, time, acc);
        
        // 发送舵机控制命令
        int result = sms_sts.WritePosEx(servo_id, position, time, acc);
        if (result != 1) {
            ROS_WARN("Failed to control servo ID %d, error: %d", servo_id, result);
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "servo_controller");
    
    // 获取串口设备名称
    std::string port = "/dev/ttyUSB0";  // 默认串口
    int baudrate = 1000000;  // 默认波特率
    
    if (argc > 1) {
        port = argv[1];
    }
    
    // 从参数服务器获取波特率
    ros::NodeHandle private_nh("~");
    private_nh.param("baudrate", baudrate, 1000000);
    
    // 创建舵机控制器
    ServoController controller(port, baudrate);
    
    // 进入ROS循环
    ros::spin();
    
    return 0;
} 