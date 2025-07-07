#ifndef MOTOR_MOTOR_NODE_H
#define MOTOR_MOTOR_NODE_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <motor/MotorOrder.h>
#include <motor/motor_control.h>
#include <string>
#include <vector>

/**
 * @brief 电机控制节点类，负责处理ROS消息和服务
 */
namespace motor {

class MotorNode {
public:
    /**
     * @brief 构造函数
     * @param port 串口设备名称
     * @param baudrate 波特率
     */
    MotorNode(const std::string& port, int baudrate);
    
    /**
     * @brief 析构函数
     */
    ~MotorNode();

private:
    ros::NodeHandle nh_;
    
    // 电机控制对象
    MotorControl motor_control_;
    
    // 订阅者
    ros::Subscriber motor_order_sub_;
    
    // 电机参数
    int serial_port_;
    std::string port_name_;
    int baudrate_;
    int max_velocity_;
    
    // 重连定时器
    ros::Timer reconnect_timer_;
    
    /**
     * @brief 查找可用的串口设备
     * @param preferred_port 首选端口
     * @return 可用的串口设备路径
     */
    std::string findSerialDevice(const std::string& preferred_port);
    
    /**
     * @brief 初始化串口
     * @param port 串口设备名称
     * @param baudrate 波特率
     * @return 初始化是否成功
     */
    bool initializeSerialPort(const std::string& port, int baudrate);
    
    /**
     * @brief 电机命令回调函数
     * @param msg 电机命令消息
     */
    void motorOrderCallback(const motor::MotorOrder::ConstPtr& msg);
    
    /**
     * @brief 发送电机命令
     * @param motor_id 电机ID
     * @param form 命令类型
     * @param velocity 速度
     * @param acceleration 加速度
     * @param deceleration 减速度
     * @param is_position_mode 是否为位置模式
     * @param position 位置
     * @param position_threshold 位置阈值
     * @return 命令是否成功发送
     */
    bool sendMotorCommand(uint8_t motor_id, uint8_t form, int16_t velocity,
                         uint16_t acceleration, uint16_t deceleration,
                         bool is_position_mode, int32_t position,
                         uint16_t position_threshold);
};

} // namespace motor

#endif // MOTOR_MOTOR_NODE_H 