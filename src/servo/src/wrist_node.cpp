#include <servo/wrist_node.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "SCServo_Linux/SCServo.h"

namespace servo {

WristNode::WristNode(const std::string& port, int baudrate) 
    : port_(port), baudrate_(baudrate), initialized_(false), sms_sts_ptr_(nullptr) {
    
    // 创建SMS_STS对象
    sms_sts_ptr_ = new SMS_STS();
    
    // 订阅舵机控制话题
    servo_sub_ = nh_.subscribe("/servo_control_topic", 10, &WristNode::servoCallback, this);
    
    // 订阅兼容liancheng_socket的话题
    motor_order_sub_ = nh_.subscribe("/Controller_motor_order", 10, &WristNode::motorOrderCallback, this);
    
    // 从ArmNode合并：创建关节控制发布器
    joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/arm_controller/command", 10);
    
    // 从ArmNode合并：创建关节状态订阅器
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &WristNode::updateJointState, this);
    
    // 创建关节控制服务
    joint_control_service_ = nh_.advertiseService("joint_control", &WristNode::jointControlServiceCallback, this);
    
    // 初始化舵机控制器
    if (!initServo()) {
        ROS_ERROR("Failed to init sms/sts motor on port %s!", port_.c_str());
        return;
    }
    
    initialized_ = true;
    ROS_INFO("SMS/STS motor initialized successfully on port %s", port_.c_str());
    ROS_INFO("WristNode (including ArmNode functionality) initialized");
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
    
    // 从ArmNode合并：关闭发布器和订阅器
    joint_pub_.shutdown();
    joint_state_sub_.shutdown();
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

// 新增函数：处理兼容liancheng_socket的消息
void WristNode::motorOrderCallback(const servo::MotorOrder::ConstPtr& msg) {
    if (!initialized_) {
        ROS_WARN("Servo controller not initialized!");
        return;
    }
    
    // 处理兼容liancheng_socket的消息
    for (size_t i = 0; i < msg->station_num.size(); ++i) {
        uint8_t station_num = msg->station_num[i];
        uint8_t form = msg->form[i];
        int16_t vel = msg->vel[i];
        uint16_t vel_ac = msg->vel_ac[i];
        uint16_t vel_de = msg->vel_de[i];
        bool pos_mode = msg->pos_mode[i];
        int32_t pos = msg->pos[i];
        uint16_t pos_thr = msg->pos_thr[i];
        
        ROS_INFO("Processing motor order: station=%d, form=%d, vel=%d, pos=%d", 
                station_num, form, vel, pos);
        
        // 处理电机命令
        if (form == 0 || form == 100) {
            // 位置模式 (对应test_talk.cpp中的位置控制逻辑)
            uint16_t servoPos = 2048 + pos;  // 转换为伺服电机位置值
            uint16_t time = 1000;  // 默认时间
            uint8_t acc = 50;      // 默认加速度
            
            sendServoCommand(station_num, servoPos, time, acc);
            
        } else if (form == 1 || form == 11) {
            // 相对位移模式 (对应pinch motor控制逻辑)
            uint16_t servoPos = 2048 + pos * 10;  // 转换为伺服电机位置值
            uint16_t time = 1000;  // 默认时间
            uint8_t acc = 50;      // 默认加速度
            
            sendServoCommand(station_num, servoPos, time, acc);
        }
    }
}

void WristNode::servoCallback(const servo::SerControl::ConstPtr& msg) {
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

// 从ArmNode合并的方法实现

void WristNode::setJointPositions(const std::vector<double>& positions) {
    // 使用WristControl设置关节位置
    wrist_control_.setJointPositions(positions);
    
    // 创建并发布关节位置命令
    std_msgs::Float64MultiArray msg;
    msg.data = positions;
    joint_pub_.publish(msg);
    
    ROS_DEBUG("已发送关节位置命令");
}

std::vector<double> WristNode::getJointPositions() const {
    // 从WristControl获取当前关节位置
    return wrist_control_.getJointPositions();
}

void WristNode::updateJointState(const sensor_msgs::JointState::ConstPtr& msg) {
    // 获取关节名称列表
    const auto& joint_names = wrist_control_.getJointNames();
    
    // 创建临时数组存储更新的关节位置
    std::vector<double> updated_positions(joint_names.size(), 0.0);
    bool found_positions = false;
    
    // 遍历收到的关节状态
    for (size_t i = 0; i < msg->name.size(); ++i) {
        // 查找对应的关节索引
        for (size_t j = 0; j < joint_names.size(); ++j) {
            if (msg->name[i] == joint_names[j] && i < msg->position.size()) {
                updated_positions[j] = msg->position[i];
                found_positions = true;
                break;
            }
        }
    }
    
    // 如果找到了有效的关节位置，更新WristControl
    if (found_positions) {
        wrist_control_.updateJointState(updated_positions);
    }
}

bool WristNode::jointControlServiceCallback(servo::JointControl::Request& req,
                                          servo::JointControl::Response& res) {
    // 验证关节位置数组的长度
    if (req.position.empty()) {
        res.success = false;
        res.message = "关节位置数组为空";
        return true;
    }
    
    try {
        // 设置关节位置
        setJointPositions(req.position);
        res.success = true;
        res.message = "关节控制命令已执行";
    } catch (const std::exception& e) {
        res.success = false;
        res.message = std::string("关节控制失败: ") + e.what();
    }
    
    return true;
}

} // namespace servo

// 主函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "wrist_node");
    
    // 获取串口设备名称
    std::string port = "/dev/ttyUSB1";  // 默认串口
    int baudrate = 1000000;  // 默认波特率
    
    if (argc > 1) {
        port = argv[1];
    }
    
    // 从参数服务器获取波特率
    ros::NodeHandle private_nh("~");
    private_nh.param("baudrate", baudrate, 1000000);
    private_nh.param<std::string>("port", port, port);
    
    // 创建手腕控制器
    servo::WristNode node(port, baudrate);
    
    // 进入ROS循环
    ros::spin();
    
    return 0;
} 