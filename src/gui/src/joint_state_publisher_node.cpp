#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <cmath>

class JointStatePublisher {
public:
    JointStatePublisher() {
        ros::NodeHandle nh;
        
        // 初始化关节值 - 当前只有四个可用关节，其他关节处于未部署状态
        // 以下顺序：1=俯仰电机, 2=直线进给电机, 3=绕轴旋转电机, 4=第二俯仰电机, 5&6=未部署
        current_joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_joint_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        // 创建发布器
        joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/arm1/joint_states", 10);
        
        // 创建订阅器，接收电机位置信息用于初始化
        motor1_sub_ = nh.subscribe("/motor1/position", 10, &JointStatePublisher::motor1Callback, this);
        motor2_sub_ = nh.subscribe("/motor2/position", 10, &JointStatePublisher::motor2Callback, this);
        motor3_sub_ = nh.subscribe("/motor3/position", 10, &JointStatePublisher::motor3Callback, this);
        motor4_sub_ = nh.subscribe("/motor4/position", 10, &JointStatePublisher::motor4Callback, this);
        
        // 5和6号电机未部署，但保留接口以便将来扩展
        motor5_sub_ = nh.subscribe("/motor5/position", 10, &JointStatePublisher::motor5Callback, this);
        motor6_sub_ = nh.subscribe("/motor6/position", 10, &JointStatePublisher::motor6Callback, this);
        
        // 订阅关节控制命令话题
        joint_command_sub_ = nh.subscribe("/arm1/joint_command", 10, &JointStatePublisher::jointCommandCallback, this);
        
        // 启动定时器，定期发布关节状态
        timer_ = nh.createTimer(ros::Duration(0.1), &JointStatePublisher::timerCallback, this);  // 10Hz
        
        // 立即发布一次初始关节状态，确保GUI能获取到数据
        publishJointState();
        
        ROS_INFO("Joint State Publisher 已初始化，当前有4个可用关节（1-4号），5-6号未部署");
    }
    
    // 电机回调函数 - 第一个俯仰关节
    void motor1Callback(const std_msgs::Int32::ConstPtr& msg) {
        // 将电机位置转换为关节角度（弧度）
        // 电机位置范围通常是800-2200
        float pos = mapValue(msg->data, 800, 2200, -M_PI/2, M_PI/2);
        current_joint_positions_[0] = pos;
    }
    
    // 直线进给电机
    void motor2Callback(const std_msgs::Int32::ConstPtr& msg) {
        // 将电机位置转换为伸缩长度（厘米）
        float pos = mapValue(msg->data, 800, 2200, 0, 43);
        current_joint_positions_[1] = pos;
    }
    
    // 绕轴旋转电机
    void motor3Callback(const std_msgs::Int32::ConstPtr& msg) {
        float pos = mapValue(msg->data, 800, 2200, -M_PI, M_PI);
        current_joint_positions_[2] = pos;
    }
    
    // 第二个俯仰电机
    void motor4Callback(const std_msgs::Int32::ConstPtr& msg) {
        float pos = mapValue(msg->data, 800, 2200, -M_PI/2, M_PI/2);
        current_joint_positions_[3] = pos;
    }
    
    // 未部署关节5
    void motor5Callback(const std_msgs::Int32::ConstPtr& msg) {
        // 这个关节未部署，保持默认值
        current_joint_positions_[4] = 0;
    }
    
    // 未部署关节6
    void motor6Callback(const std_msgs::Int32::ConstPtr& msg) {
        // 这个关节未部署，保持默认值
        current_joint_positions_[5] = 0;
    }
    
    // 处理关节控制命令
    void jointCommandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 更新关节位置
        if (msg->position.size() >= 4) {  // 至少需要4个可用关节的值
            for (size_t i = 0; i < 4 && i < msg->position.size(); i++) {
                current_joint_positions_[i] = msg->position[i];
            }
        }
        
        // 如果有速度信息，也进行更新
        if (msg->velocity.size() >= 4) {
            for (size_t i = 0; i < 4 && i < msg->velocity.size(); i++) {
                current_joint_velocities_[i] = msg->velocity[i];
            }
        }
    }
    
    // 添加一个新的函数，封装关节状态发布逻辑
    void publishJointState() {
        sensor_msgs::JointState joint_state;
        
        // 设置时间戳和帧ID
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "arm1_base_link";
        
        // 设置关节名称
        joint_state.name = {
            "arm1_joint1",  // 俯仰电机
            "arm1_joint2",  // 直线进给电机
            "arm1_joint3",  // 绕轴旋转电机
            "arm1_joint4",  // 第二俯仰电机
            "arm1_joint5",  // 未部署
            "arm1_joint6"   // 未部署
        };
        
        // 设置关节位置
        joint_state.position = current_joint_positions_;
        
        // 设置关节速度
        joint_state.velocity = current_joint_velocities_;
        
        // 发布关节状态
        joint_state_pub_.publish(joint_state);
    }
    
    // 定时器回调，发布当前关节状态
    void timerCallback(const ros::TimerEvent&) {
        publishJointState();
    }
    
private:
    ros::Publisher joint_state_pub_;
    ros::Subscriber motor1_sub_;
    ros::Subscriber motor2_sub_;
    ros::Subscriber motor3_sub_;
    ros::Subscriber motor4_sub_;
    ros::Subscriber motor5_sub_;
    ros::Subscriber motor6_sub_;
    ros::Subscriber joint_command_sub_;
    ros::Timer timer_;
    
    std::vector<double> current_joint_positions_;
    std::vector<double> current_joint_velocities_;
    
    // 映射函数，将值从一个范围映射到另一个范围
    float mapValue(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    
    JointStatePublisher joint_state_publisher;
    
    ros::spin();
    return 0;
} 