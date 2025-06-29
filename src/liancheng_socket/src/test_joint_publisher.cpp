#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_joint_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/arm1/joint_states", 10);
    
    ros::Rate rate(10); // 10Hz
    double time = 0;
    
    while (ros::ok()) {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        
        // 添加关节名称
        joint_state.name.push_back("joint1"); // 底座旋转
        joint_state.name.push_back("joint2"); // 伸缩关节
        joint_state.name.push_back("joint3"); // 肩部关节
        joint_state.name.push_back("joint4"); // 肘部关节
        joint_state.name.push_back("joint5"); // 固定关节
        joint_state.name.push_back("joint6"); // 末端伸缩

        // 设置关节位置 - 使用简单的正弦函数让关节动起来
        joint_state.position.push_back(0.5 * sin(time));            // joint1: -0.5~0.5 rad
        joint_state.position.push_back(20.0 + 10.0 * sin(time));    // joint2: 10~30 cm
        joint_state.position.push_back(0.3 * sin(time));            // joint3: -0.3~0.3 rad
        joint_state.position.push_back(0.7 + 0.5 * sin(time));      // joint4: 0.2~1.2 rad
        joint_state.position.push_back(M_PI/2);                     // joint5: 固定在 π/2
        joint_state.position.push_back(10.0 + 3.0 * sin(time));     // joint6: 7~13 cm
        
        // 发布关节状态
        joint_state_pub.publish(joint_state);
        
        // 更新时间
        time += 0.1;
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
} 