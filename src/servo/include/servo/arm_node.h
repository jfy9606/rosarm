#ifndef SERVO_ARM_NODE_H
#define SERVO_ARM_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <servo/arm_control.h>
#include <servo/JointControl.h>
#include <servo/MotorOrder.h>
#include <servo/SerControl.h>
#include <servo/HomePosition.h>
#include <string>
#include <vector>

namespace servo {

/**
 * @brief 机械臂节点类，处理ROS通信并使用ArmControl进行高级控制
 */
class ArmNode {
public:
    /**
     * @brief 构造函数
     * @param motor_port 电机串口名称
     * @param servo_port 舵机串口名称
     * @param motor_baudrate 电机串口波特率
     * @param servo_baudrate 舵机串口波特率
     */
    ArmNode(const std::string& motor_port, const std::string& servo_port,
           int motor_baudrate = 115200, int servo_baudrate = 115200);
    
    /**
     * @brief 析构函数
     */
    ~ArmNode();
    
    /**
     * @brief 运行节点
     */
    void run();

private:
    ros::NodeHandle nh_;
    
    // 订阅者
    ros::Subscriber joint_state_sub_;
    ros::Subscriber joint_command_sub_;
    ros::Subscriber pose_command_sub_;
    ros::Subscriber vacuum_cmd_sub_;
    ros::Subscriber vacuum_power_sub_;
    ros::Subscriber gripper_cmd_sub_;
    ros::Subscriber motor_order_sub_;
    ros::Subscriber servo_control_sub_;
    ros::Subscriber home_position_sub_;
    
    // 发布者
    ros::Publisher joint_state_pub_;
    ros::Publisher end_effector_pose_pub_;
    ros::Publisher status_pub_;
    
    // 服务
    ros::ServiceServer joint_control_service_;
    ros::ServiceServer home_position_service_;
    
    // 电机和舵机节点
    std::string motor_port_;
    std::string servo_port_;
    int motor_baudrate_;
    int servo_baudrate_;
    
    // 高级控制器
    ArmControl arm_control_;
    
    // 当前状态
    std::vector<double> current_joint_positions_;
    geometry_msgs::Pose current_end_effector_pose_;
    bool is_vacuum_on_;
    int vacuum_power_;
    bool is_gripper_open_;
    int gripper_position_;
    
    // 回调函数
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void poseCommandCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void vacuumCommandCallback(const std_msgs::Bool::ConstPtr& msg);
    void vacuumPowerCallback(const std_msgs::Int32::ConstPtr& msg);
    void gripperCommandCallback(const std_msgs::Bool::ConstPtr& msg);
    void motorOrderCallback(const servo::MotorOrder::ConstPtr& msg);
    void servoControlCallback(const servo::SerControl::ConstPtr& msg);
    void homePositionCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // 服务回调
    bool jointControlServiceCallback(servo::JointControl::Request& req, 
                                   servo::JointControl::Response& res);
    bool homePositionServiceCallback(servo::HomePosition::Request& req, 
                                   servo::HomePosition::Response& res);
    
    // 辅助函数
    void publishJointState();
    void publishEndEffectorPose();
    void publishStatus(const std::string& status);
};

} // namespace servo

#endif // SERVO_ARM_NODE_H 