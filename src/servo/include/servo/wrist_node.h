#ifndef SERVO_WRIST_NODE_H
#define SERVO_WRIST_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <servo/SerControl.h>
#include <servo/wrist_control.h>
#include <servo/JointControl.h>
#include <string>

// 使用servo_wrist包中的MotorOrder消息
#include <servo/MotorOrder.h>

namespace servo {

/**
 * @brief 手腕控制器节点类，处理ROS通信并使用WristControl进行核心功能
 * 已整合原ArmNode功能
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
    void servoCallback(const servo::SerControl::ConstPtr& msg);
    
    /**
     * @brief MotorOrder兼容回调函数，处理liancheng_socket风格的消息
     * @param msg 电机控制消息
     */
    void motorOrderCallback(const servo::MotorOrder::ConstPtr& msg);
    
    /**
     * @brief 初始化伺服电机控制器
     * @return 初始化是否成功
     */
    bool initServo();
    
    // 从ArmNode合并的方法
    
    /**
     * @brief 设置关节位置
     * @param positions 关节位置数组
     */
    void setJointPositions(const std::vector<double>& positions);
    
    /**
     * @brief 获取当前关节位置
     * @return 关节位置数组
     */
    std::vector<double> getJointPositions() const;
    
    /**
     * @brief 更新关节状态回调函数
     * @param msg 关节状态消息
     */
    void updateJointState(const sensor_msgs::JointState::ConstPtr& msg);

    /**
     * @brief 关节控制服务回调函数
     * @param req 服务请求
     * @param res 服务响应
     * @return 操作是否成功
     */
    bool jointControlServiceCallback(servo::JointControl::Request& req, 
                                   servo::JointControl::Response& res);

private:
    ros::NodeHandle nh_;
    ros::Subscriber servo_sub_;
    ros::Subscriber motor_order_sub_;  // 添加兼容liancheng_socket的订阅者
    std::string port_;
    int baudrate_;
    bool initialized_;
    
    // 从ArmNode合并的成员
    ros::Publisher joint_pub_;
    ros::Subscriber joint_state_sub_;
    
    // 服务
    ros::ServiceServer joint_control_service_;
    
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

} // namespace servo

#endif // SERVO_WRIST_NODE_H 