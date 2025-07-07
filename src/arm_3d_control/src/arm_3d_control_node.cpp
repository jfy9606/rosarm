#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <servo_wrist/SerControl.h>
#include <string>
#include <cmath>

#include "arm_3d_control/arm_parameters.h"
#include "arm_3d_control/inverse_kinematics.h"

using namespace arm_3d_control;

// 处理3D位置控制的回调函数
void positionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    ROS_INFO("Received target position: x=%f, y=%f, z=%f", msg->x, msg->y, msg->z);
    
    // 计算逆运动学
    double baseAngle, shoulderAngle, elbowAngle;
    if (!InverseKinematics::calculateAngles(*msg, baseAngle, shoulderAngle, elbowAngle)) {
        ROS_ERROR("Failed to calculate inverse kinematics");
        return;
    }
    
    ROS_INFO("Calculated joint angles: base=%f, shoulder=%f, elbow=%f", 
             baseAngle, shoulderAngle, elbowAngle);
    
    // 创建3个舵机的控制消息
    ros::NodeHandle nh;
    ros::Publisher servo_pub = nh.advertise<servo_wrist::SerControl>("servo_control_topic", 10);
    
    // 设置底座舵机角度
    servo_wrist::SerControl baseMsg;
    baseMsg.servo_id = ArmParameters::BASE_SERVO_ID;
    baseMsg.target_position = InverseKinematics::angleToPulse(baseAngle);
    baseMsg.velocity = ArmParameters::DEFAULT_VELOCITY;
    baseMsg.acceleration = ArmParameters::DEFAULT_ACCELERATION;
    
    // 设置肩部舵机角度
    servo_wrist::SerControl shoulderMsg;
    shoulderMsg.servo_id = ArmParameters::SHOULDER_SERVO_ID;
    shoulderMsg.target_position = InverseKinematics::angleToPulse(shoulderAngle);
    shoulderMsg.velocity = ArmParameters::DEFAULT_VELOCITY;
    shoulderMsg.acceleration = ArmParameters::DEFAULT_ACCELERATION;
    
    // 设置肘部舵机角度
    servo_wrist::SerControl elbowMsg;
    elbowMsg.servo_id = ArmParameters::ELBOW_SERVO_ID;
    elbowMsg.target_position = InverseKinematics::angleToPulse(elbowAngle);
    elbowMsg.velocity = ArmParameters::DEFAULT_VELOCITY;
    elbowMsg.acceleration = ArmParameters::DEFAULT_ACCELERATION;
    
    // 依次发送控制消息
    servo_pub.publish(baseMsg);
    ros::Duration(0.1).sleep(); // 小延时，确保消息能依次处理
    
    servo_pub.publish(shoulderMsg);
    ros::Duration(0.1).sleep();
    
    servo_pub.publish(elbowMsg);
    
    ROS_INFO("Sent control commands to servos");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_3d_control_node");
    ros::NodeHandle nh;
    
    // 创建订阅者，接收3D位置指令
    ros::Subscriber position_sub = nh.subscribe("arm_target_position", 10, positionCallback);
    
    ROS_INFO("Arm 3D control node started. Waiting for position commands...");
    
    ros::spin();
    
    return 0;
} 