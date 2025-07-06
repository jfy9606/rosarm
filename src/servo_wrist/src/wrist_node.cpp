#include <servo_wrist/wrist_node.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "SCServo.h"

namespace servo_wrist {

WristNode::WristNode(const std::string& port, int baudrate) 
    : port_(port), baudrate_(baudrate), initialized_(false), sms_sts_ptr_(nullptr) {
    
    // 创建SMS_STS对象
    sms_sts_ptr_ = new SMS_STS();
    
    // 订阅舵机控制话题
    servo_sub_ = nh_.subscribe("/servo_control_topic", 10, &WristNode::servoCallback, this);
    
    // 初始化舵机控制器
    if (!initServo()) {
        ROS_ERROR("Failed to init sms/sts motor on port %s!", port_.c_str());
        return;
    }
    
    initialized_ = true;
    ROS_INFO("SMS/STS motor initialized successfully on port %s", port_.c_str());
}

WristNode::~WristNode() {
    // 关闭舵机控制器
    if (initialized_ && sms_sts_ptr_) {
        SMS_STS* sms_sts = static_cast<SMS_STS*>(sms_sts_ptr_);
        sms_sts->end();
    }
    
    // 释放内存
    if (sms_sts_ptr_) {
        delete static_cast<SMS_STS*>(sms_sts_ptr_);
        sms_sts_ptr_ = nullptr;
    }
}

bool WristNode::initServo() {
    ROS_INFO("Using serial port: %s", port_.c_str());
    ROS_INFO("Serial speed %d", baudrate_);
    
    if (!sms_sts_ptr_) {
        return false;
    }
    
    SMS_STS* sms_sts = static_cast<SMS_STS*>(sms_sts_ptr_);
    
    // 初始化舵机控制器，注意参数顺序：波特率在前，串口名在后
    if (sms_sts->begin(baudrate_, port_.c_str())) {
        return true;
    }
    
    return false;
}

void WristNode::servoCallback(const servo_wrist::SerControl::ConstPtr& msg) {
    if (!initialized_) {
        ROS_WARN("Servo controller not initialized!");
        return;
    }
    
    uint8_t servo_id = msg->servo_id;
    uint16_t position = msg->target_position;
    uint16_t time = msg->velocity;  // 使用velocity字段作为时间
    uint8_t acc = msg->acceleration;
    
    // 首先更新WristControl的内部状态
    double angle_rad = wrist_control_.positionToAngle(servo_id, position);
    wrist_control_.setServoAngle(servo_id, angle_rad);
    
    // 发送命令到实际硬件
    if (!sendServoCommand(servo_id, position, time, acc)) {
        ROS_WARN("Failed to control servo ID %d", servo_id);
    } else {
        ROS_INFO("Moving servo ID %d to position %d with time %d and acc %d", 
                 servo_id, position, time, acc);
    }
}

bool WristNode::sendServoCommand(uint8_t servo_id, uint16_t position, uint16_t time, uint8_t acc) {
    if (!initialized_ || !sms_sts_ptr_) {
        return false;
    }
    
    SMS_STS* sms_sts = static_cast<SMS_STS*>(sms_sts_ptr_);
    
    // 发送舵机控制命令
    int result = sms_sts->WritePosEx(servo_id, position, time, acc);
    return (result == 1);
}

bool WristNode::controlServoByAngle(int servo_id, double angle_rad, uint16_t time, uint8_t acc) {
    // 首先更新WristControl的内部状态
    if (!wrist_control_.setServoAngle(servo_id, angle_rad)) {
        return false;  // 无效的伺服ID或角度
    }
    
    // 将角度转换为位置值
    int position = wrist_control_.angleToPosition(servo_id, angle_rad);
    
    // 发送命令到实际硬件
    return sendServoCommand(servo_id, position, time, acc);
}

} // namespace servo_wrist

// 主函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "wrist_node");
    
    // 获取串口设备名称
    std::string port = "/dev/ttyUSB0";  // 默认串口
    int baudrate = 1000000;  // 默认波特率
    
    if (argc > 1) {
        port = argv[1];
    }
    
    // 从参数服务器获取波特率
    ros::NodeHandle private_nh("~");
    private_nh.param("baudrate", baudrate, 1000000);
    
    // 创建手腕控制器
    servo_wrist::WristNode node(port, baudrate);
    
    // 进入ROS循环
    ros::spin();
    
    return 0;
} 