#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "liancheng_socket/MotorOrder.h"
#include "std_msgs/String.h"
#include <string>
#include <cmath>
#include <vector>

class JointStatePublisher {
public:
    JointStatePublisher(ros::NodeHandle& nh) : nh_(nh) {
        // 初始化关节状态
        for (int i = 0; i < 6; i++) {
            current_joint_values_.push_back(0.0);
        }

        // 创建发布器
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm1/joint_states", 10);
        
        // 订阅电机命令
        motor_order_sub_ = nh_.subscribe("/Controller_motor_order", 10, 
                                         &JointStatePublisher::motorOrderCallback, this);
        
        // 设置定时器，定期发布关节状态
        timer_ = nh_.createTimer(ros::Duration(0.05), // 20Hz
                                &JointStatePublisher::publishJointStates, this);
        
        ROS_INFO("Joint state publisher initialized");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber motor_order_sub_;
    ros::Timer timer_;
    std::vector<double> current_joint_values_;

    // 处理电机命令
    void motorOrderCallback(const liancheng_socket::MotorOrder::ConstPtr& msg) {
        // 遍历所有接收到的命令
        for (size_t i = 0; i < msg->station_num.size(); i++) {
            uint8_t station_num = msg->station_num[i];
            int32_t pos = msg->pos[i];
            
            // 根据station_num分配到不同的关节
            // 假设station_num 1 = 关节1，station_num 2 = 关节2，以此类推
            if (station_num >= 1 && station_num <= 6) {
                // 将电机位置转换为关节角度 - 这里需要根据实际的映射关系进行调整
                // 下面是一个简单的示例映射
                if (station_num == 1) {  // 例如：底座旋转关节
                    current_joint_values_[0] = pos * M_PI / 180.0; // 假设pos是度数，转换为弧度
                }
                else if (station_num == 2) { // 伸缩关节
                    current_joint_values_[1] = pos;
                }
                // ... 对其他关节也进行类似处理
            }
        }
    }

    // 发布关节状态
    void publishJointStates(const ros::TimerEvent&) {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        
        // 添加关节名称
        joint_state.name.push_back("joint1"); // 底座旋转
        joint_state.name.push_back("joint2"); // 伸缩关节
        joint_state.name.push_back("joint3"); // 肩部关节
        joint_state.name.push_back("joint4"); // 肘部关节
        joint_state.name.push_back("joint5"); // 固定关节
        joint_state.name.push_back("joint6"); // 末端伸缩
        
        // 添加关节位置
        for (size_t i = 0; i < current_joint_values_.size(); i++) {
            joint_state.position.push_back(current_joint_values_[i]);
        }
        
        // 发布关节状态
        joint_state_pub_.publish(joint_state);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;
    
    JointStatePublisher joint_state_publisher(nh);
    
    ros::spin();
    return 0;
} 