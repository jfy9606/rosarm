#ifndef SERVO_WRIST_WRIST_NODE_H
#define SERVO_WRIST_WRIST_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <servo_wrist/SerControl.h>
#include <servo_wrist/wrist_control.h>
#include <string>

namespace servo_wrist {

/**
 * @brief 手腕控制器节点类，处理ROS通信并使用WristControl进行核心功能
 */
class WristNode {
public:
    /**
     * @brief 构造函数
     * @param port 串口名称
     * @param baudrate 波特率
     */
    WristNode(const std::string& port, int baudrate);
    
    /**
     * @brief 析构函数
     */
    ~WristNode();

    /**
     * @brief 伺服控制回调函数
     * @param msg 伺服控制消息
     */
    void servoCallback(const servo_wrist::SerControl::ConstPtr& msg);
    
    /**
     * @brief 初始化伺服电机控制器
     * @return 初始化是否成功
     */
    bool initServo();

private:
    ros::NodeHandle nh_;
    ros::Subscriber servo_sub_;
    std::string port_;
    int baudrate_;
    bool initialized_;
    
    // 伺服电机通信类
    void* sms_sts_ptr_;  // SMS_STS类指针（使用void*避免在头文件中包含SCServo.h）
    
    // 核心控制器
    WristControl wrist_control_;
    
    /**
     * @brief 发送伺服电机控制命令
     * @param servo_id 伺服电机ID
     * @param position 目标位置
     * @param time 运动时间
     * @param acc 加速度
     * @return 操作是否成功
     */
    bool sendServoCommand(uint8_t servo_id, uint16_t position, uint16_t time, uint8_t acc);
    
    /**
     * @brief 通过角度控制伺服电机
     * @param servo_id 伺服电机ID
     * @param angle_rad 角度（弧度）
     * @param time 运动时间
     * @param acc 加速度
     * @return 操作是否成功
     */
    bool controlServoByAngle(int servo_id, double angle_rad, uint16_t time = 500, uint8_t acc = 50);
};

} // namespace servo_wrist

#endif // SERVO_WRIST_WRIST_NODE_H 