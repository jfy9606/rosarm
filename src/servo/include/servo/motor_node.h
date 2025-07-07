#ifndef SERVO_MOTOR_NODE_H
#define SERVO_MOTOR_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <servo/MotorOrder.h>
#include <servo/motor_control.h>
#include <string>
#include <vector>

namespace servo {

/**
 * @brief 电机控制节点类，处理ROS通信并使用MotorControl进行核心功能
 */
class MotorNode {
public:
    /**
     * @brief 构造函数
     * @param port 串口名称
     * @param baudrate 波特率
     */
    MotorNode(const std::string& port, int baudrate);
    
    /**
     * @brief 析构函数
     */
    ~MotorNode();

    /**
     * @brief 电机命令回调函数
     * @param msg 电机命令消息
     */
    void motorOrderCallback(const servo::MotorOrder::ConstPtr& msg);
    
    /**
     * @brief 初始化串口连接
     * @param port 串口名称
     * @param baudrate 波特率
     * @return 初始化是否成功
     */
    bool initializeSerialPort(const std::string& port, int baudrate);
    
    /**
     * @brief 查找串口设备
     * @param preferred_port 首选端口
     * @return 找到的串口设备路径
     */
    std::string findSerialDevice(const std::string& preferred_port);
    
    /**
     * @brief 电机控制服务回调函数
     * @param req 服务请求
     * @param res 服务响应
     * @return 操作是否成功
     */
    bool motorControlServiceCallback(servo::MotorControl::Request& req, 
                                   servo::MotorControl::Response& res);

private:
    ros::NodeHandle nh_;
    ros::Subscriber motor_order_sub_;
    ros::ServiceServer motor_control_service_;
    
    int serial_port_;
    std::string port_name_;
    int baudrate_;
    ros::Timer reconnect_timer_;
    
    // 电机控制核心类
    MotorControl motor_control_;
    
    // 电机控制参数
    int max_velocity_;
    
    /**
     * @brief 发送电机命令
     * @param motor_id 电机ID
     * @param form 命令形式
     * @param velocity 速度
     * @param acceleration 加速度
     * @param deceleration 减速度
     * @param is_position_mode 是否为位置模式
     * @param position 目标位置
     * @param position_threshold 位置阈值
     * @return 操作是否成功
     */
    bool sendMotorCommand(uint8_t motor_id, uint8_t form, int16_t velocity,
                         uint16_t acceleration, uint16_t deceleration,
                         bool is_position_mode, int32_t position,
                         uint16_t position_threshold);
};

} // namespace servo

#endif // SERVO_MOTOR_NODE_H 