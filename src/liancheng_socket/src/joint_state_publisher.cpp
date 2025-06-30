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
        // 初始化关节状态 - 设置一组有意义的初始值，避免空数据
        current_joint_values_ = {
            0.0,    // joint1 (底座旋转关节)
            10.0,   // joint2 (伸缩关节) - 设置一个初始值以避免关节为0
            0.0,    // joint3 (肩部关节)
            45.0 * M_PI / 180.0, // joint4 (肘部关节) - 初始45度角，转换为弧度
            0.0,    // joint5 (固定关节)
            10.0    // joint6 (末端伸缩) - 设置一个初始值以避免关节为0
        };

        // 创建发布器
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm1/joint_states", 10);
        
        // 订阅电机命令
        motor_order_sub_ = nh_.subscribe("/Controller_motor_order", 10, 
                                         &JointStatePublisher::motorOrderCallback, this);
        
        // 设置定时器，定期发布关节状态
        timer_ = nh_.createTimer(ros::Duration(0.05), // 20Hz
                                &JointStatePublisher::publishJointStates, this);
        
        ROS_INFO("Joint state publisher initialized with default joint values");
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
                switch(station_num) {
                    case 1:  // 底座旋转关节
                        current_joint_values_[0] = pos * M_PI / 180.0; // 转换为弧度
                        break;
                    case 2:  // 伸缩关节
                        current_joint_values_[1] = pos;
                        break;
                    case 3:  // 肩部关节
                        current_joint_values_[2] = pos * M_PI / 180.0; // 转换为弧度
                        break;
                    case 4:  // 肘部关节
                        current_joint_values_[3] = pos * M_PI / 180.0; // 转换为弧度
                        break;
                    case 5:  // 固定关节
                        current_joint_values_[4] = pos * M_PI / 180.0; // 转换为弧度
                        break;
                    case 6:  // 末端伸缩
                        current_joint_values_[5] = pos;
                        break;
                }
                
                // 记录接收到的关节位置，便于调试
                ROS_DEBUG("Updated joint %d to position: %f", station_num, 
                          (station_num == 2 || station_num == 6) ? pos : pos * M_PI / 180.0);
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