#ifndef SERVO_WRIST_VACUUM_NODE_H
#define SERVO_WRIST_VACUUM_NODE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <servo_wrist/VacuumCmd.h>
#include <servo_wrist/vacuum_control.h>
#include <string>

namespace servo_wrist {

/**
 * @brief 真空控制器节点类，处理ROS通信并使用VacuumControl进行核心功能
 */
class VacuumNode {
public:
    /**
     * @brief 构造函数
     * @param port 串口名称
     * @param baudrate 波特率
     */
    VacuumNode(const std::string& port, int baudrate);
    
    /**
     * @brief 析构函数
     */
    ~VacuumNode();

    /**
     * @brief ROS命令回调
     * @param msg 真空命令消息
     */
    void vacuumCommandCallback(const std_msgs::Bool::ConstPtr& msg);
    
    /**
     * @brief ROS功率回调
     * @param msg 功率设置消息
     */
    void vacuumPowerCallback(const std_msgs::Int32::ConstPtr& msg);
    
    /**
     * @brief ROS服务回调
     * @param req 服务请求
     * @param res 服务响应
     * @return 服务调用是否成功
     */
    bool vacuumServiceCallback(servo_wrist::VacuumCmd::Request& req, 
                              servo_wrist::VacuumCmd::Response& res);
    
    /**
     * @brief 设置真空状态
     * @param enable 是否启用
     * @param power 功率百分比
     * @return 操作是否成功
     */
    bool setVacuum(bool enable, int power);

private:
    /**
     * @brief 查找串口设备
     * @param preferred_port 首选端口
     * @return 找到的串口设备路径
     */
    std::string findSerialDevice(const std::string& preferred_port);
    
    /**
     * @brief 初始化串口
     * @param port 串口名称
     * @param baudrate 波特率
     * @return 初始化是否成功
     */
    bool initializeSerialPort(const std::string& port, int baudrate);

    ros::NodeHandle nh_;
    ros::Subscriber vacuum_cmd_sub_;
    ros::Subscriber vacuum_power_sub_;
    ros::ServiceServer vacuum_service_;
    ros::Timer reconnect_timer_;
    
    int serial_port_;
    int max_power_;
    int default_power_;
    std::string port_name_;
    
    // 核心控制器
    VacuumControl vacuum_control_;
};

} // namespace servo_wrist

#endif // SERVO_WRIST_VACUUM_NODE_H 