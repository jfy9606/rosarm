#include <servo/arm_node.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

namespace servo {

ArmNode::ArmNode(const std::string& motor_port, const std::string& servo_port,
                int motor_baudrate, int servo_baudrate)
    : motor_port_(motor_port), servo_port_(servo_port),
      motor_baudrate_(motor_baudrate), servo_baudrate_(servo_baudrate),
      is_vacuum_on_(false), vacuum_power_(0), is_gripper_open_(false), gripper_position_(0) {
    
    // 初始化关节位置
    current_joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // 设置发布者
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    end_effector_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/end_effector_pose", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/arm_status", 10);
    
    // 设置订阅者
    joint_command_sub_ = nh_.subscribe("/joint_command", 10, &ArmNode::jointCommandCallback, this);
    pose_command_sub_ = nh_.subscribe("/pose_command", 10, &ArmNode::poseCommandCallback, this);
    vacuum_cmd_sub_ = nh_.subscribe("/vacuum/command", 10, &ArmNode::vacuumCommandCallback, this);
    vacuum_power_sub_ = nh_.subscribe("/vacuum/power", 10, &ArmNode::vacuumPowerCallback, this);
    gripper_cmd_sub_ = nh_.subscribe("/gripper/command", 10, &ArmNode::gripperCommandCallback, this);
    motor_order_sub_ = nh_.subscribe("/Controller_motor_order", 10, &ArmNode::motorOrderCallback, this);
    servo_control_sub_ = nh_.subscribe("/servo_control_topic", 10, &ArmNode::servoControlCallback, this);
    home_position_sub_ = nh_.subscribe("/home_position", 10, &ArmNode::homePositionCallback, this);
    
    // 设置服务
    joint_control_service_ = nh_.advertiseService("/joint_control", &ArmNode::jointControlServiceCallback, this);
    home_position_service_ = nh_.advertiseService("/home_position_service", &ArmNode::homePositionServiceCallback, this);
    
    ROS_INFO("Arm node initialized, motor port: %s, servo port: %s", motor_port.c_str(), servo_port.c_str());
}

ArmNode::~ArmNode() {
    // 发布状态
    publishStatus("Arm node closed");
}

void ArmNode::run() {
    // 发布状态
    publishStatus("Arm node started");
    
    // 发布关节状态
    ros::Rate rate(10);  // 10 Hz
    
    while (ros::ok()) {
        // 发布关节状态
        publishJointState();
        
        // 发布末端执行器位姿
        publishEndEffectorPose();
        
        // 处理回调
        ros::spinOnce();
        
        // 等待
        rate.sleep();
    }
}

void ArmNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // 更新关节状态
    if (msg->position.size() >= 6) {
        for (size_t i = 0; i < 6 && i < msg->position.size(); ++i) {
            current_joint_positions_[i] = msg->position[i];
        }
        
        // 更新末端执行器位姿
        current_end_effector_pose_ = arm_control_.computeForwardKinematics(current_joint_positions_);
    }
}

void ArmNode::jointCommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    // 处理关节命令
    if (msg->data.size() >= 6) {
        std::vector<double> joint_values(msg->data.begin(), msg->data.begin() + 6);
        
        // 设置关节位置
        if (arm_control_.setJointPositions(joint_values)) {
            publishStatus("Joint control successful");
        } else {
            publishStatus("Joint control failed");
        }
    } else {
        publishStatus("Insufficient joint command data");
    }
}

void ArmNode::poseCommandCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // 处理位姿命令
    if (arm_control_.setEndEffectorPose(*msg)) {
        publishStatus("Pose control successful");
    } else {
        publishStatus("Pose control failed");
    }
}

void ArmNode::vacuumCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
    // 处理真空吸盘命令
    is_vacuum_on_ = msg->data;
    
    // 实际应用中这里应调用真空控制函数
    publishStatus(is_vacuum_on_ ? "Vacuum suction enabled" : "Vacuum suction disabled");
}

void ArmNode::vacuumPowerCallback(const std_msgs::Int32::ConstPtr& msg) {
    // 处理真空吸盘功率命令
    vacuum_power_ = std::max(0, std::min(100, msg->data));
    
    // 如果真空吸盘已开启，更新功率
    if (is_vacuum_on_) {
        // 实际应用中这里应调用真空控制函数
        publishStatus("Vacuum power updated: " + std::to_string(vacuum_power_) + "%");
    }
}

void ArmNode::gripperCommandCallback(const std_msgs::Bool::ConstPtr& msg) {
    // 处理夹爪命令
    is_gripper_open_ = msg->data;
    
    // 更新状态
    publishStatus(is_gripper_open_ ? "Gripper opened" : "Gripper closed");
}

void ArmNode::motorOrderCallback(const servo::MotorOrder::ConstPtr& msg) {
    // 处理兼容liancheng_socket风格的电机命令
    publishStatus("Motor command received");
    
    // 实际应用中这里应将命令转发给电机控制器
    // 由于该命令通常控制大臂的两个电机，这里需要实现对应的控制逻辑
    
    // 此处为简化实现
    if (!msg->station_num.empty() && !msg->pos.empty()) {
        uint8_t motor_id = msg->station_num[0];
        int32_t position = msg->pos[0];
        
        // 将电机位置转换为关节角度
        double joint_angle = position / 1000.0;  // 简化转换
        
        // 仅更新对应的关节
        if (motor_id < 2) {  // 大臂电机
            current_joint_positions_[motor_id] = joint_angle;
            
            // 设置关节位置
            arm_control_.setJointPositions(current_joint_positions_);
        }
    }
}

void ArmNode::servoControlCallback(const servo::SerControl::ConstPtr& msg) {
    // 处理舵机控制命令
    publishStatus("Servo command received");
    
    // 舵机ID到关节索引的映射
    int joint_index = -1;
    switch (msg->servo_id) {
        case 1: joint_index = 2; break;  // 肘部抬升
        case 2: joint_index = 3; break;  // 腕部旋转
        case 3: joint_index = 4; break;  // 腕部俯仰
        case 4: joint_index = 5; break;  // 夹爪
        default: break;
    }
    
    if (joint_index >= 0) {
        // 将舵机位置转换为关节角度
        double joint_angle = (msg->target_position - 2048) / 2048.0 * M_PI;  // 简化转换
        
        // 更新对应的关节
        current_joint_positions_[joint_index] = joint_angle;
        
        // 设置关节位置
        arm_control_.setJointPositions(current_joint_positions_);
    }
}

void ArmNode::homePositionCallback(const std_msgs::Bool::ConstPtr& msg) {
    // 处理回到初始位置命令
    if (msg->data) {
        if (arm_control_.goHome()) {
            publishStatus("Arm returned to home position");
        } else {
            publishStatus("Failed to return arm to home position");
        }
    }
}

bool ArmNode::jointControlServiceCallback(servo::JointControl::Request& req, 
                                        servo::JointControl::Response& res) {
    // 处理关节控制服务
    if (req.position.size() < 6) {
        res.success = false;
        res.message = "Insufficient joint data, 6 joint values required";
        return true;
    }
    
    std::vector<double> joint_values(req.position.begin(), req.position.begin() + 6);
    
    // 设置关节位置
    bool success = arm_control_.setJointPositions(joint_values);
    
    // 设置响应
    res.success = success;
    res.message = success ? "Joint control successful" : "Joint control failed";
    
    return true;
}

bool ArmNode::homePositionServiceCallback(servo::HomePosition::Request& req, 
                                        servo::HomePosition::Response& res) {
    // 处理回到初始位置服务
    bool success = arm_control_.goHome();
    
    // 设置响应
    res.success = success;
    res.message = success ? "Arm returned to home position" : "Failed to return arm to home position";
    
    return true;
}

void ArmNode::publishJointState() {
    // 发布关节状态
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    
    // 设置关节名称
    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    
    // 设置关节位置
    joint_state.position = current_joint_positions_;
    
    // 发布
    joint_state_pub_.publish(joint_state);
}

void ArmNode::publishEndEffectorPose() {
    // 发布末端执行器位姿
    end_effector_pose_pub_.publish(current_end_effector_pose_);
}

void ArmNode::publishStatus(const std::string& status) {
    // 发布状态消息
    std_msgs::String status_msg;
    status_msg.data = status;
    status_pub_.publish(status_msg);
    
    // 同时输出到控制台
    ROS_INFO("%s", status.c_str());
}

} // namespace servo

// 主函数
int main(int argc, char** argv) {
    // 初始化ROS
    ros::init(argc, argv, "arm_node");
    
    // 获取参数
    ros::NodeHandle nh("~");
    std::string motor_port = "/dev/ttyUSB0";
    std::string servo_port = "/dev/ttyUSB1";
    int motor_baudrate = 115200;
    int servo_baudrate = 115200;
    
    // 从参数服务器获取参数
    nh.param<std::string>("motor_port", motor_port, motor_port);
    nh.param<std::string>("servo_port", servo_port, servo_port);
    nh.param<int>("motor_baudrate", motor_baudrate, motor_baudrate);
    nh.param<int>("servo_baudrate", servo_baudrate, servo_baudrate);
    
    // 创建节点
    servo::ArmNode arm_node(motor_port, servo_port, motor_baudrate, servo_baudrate);
    
    // 运行节点
    arm_node.run();
    
    return 0;
} 