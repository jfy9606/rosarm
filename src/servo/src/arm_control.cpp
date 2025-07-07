#include <servo/arm_control.h>
#include <algorithm>
#include <cmath>
#include <map>

namespace servo {

// 机械臂关节的枚举
enum ArmJoint {
    BASE_ROTATION = 0,   // 底座旋转（大臂电机1）
    SHOULDER_LIFT = 1,   // 肩部抬升（大臂电机2）
    ELBOW_LIFT = 2,      // 肘部抬升（小臂舵机1）
    WRIST_ROTATION = 3,  // 腕部旋转（小臂舵机2）
    WRIST_PITCH = 4,     // 腕部俯仰（小臂舵机3）
    GRIPPER = 5          // 夹爪（小臂舵机4）
};

ArmControl::ArmControl() {
    // 设置机械臂参数
    setupArmParameters();
    
    // 初始化机械臂状态
    state_.joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    state_.is_initialized = false;
    
    // 初始化末端位姿
    state_.end_effector_pose.position.x = 0.3; // 默认位置
    state_.end_effector_pose.position.y = 0.0;
    state_.end_effector_pose.position.z = 0.3;
    state_.end_effector_pose.orientation.x = 0.0;
    state_.end_effector_pose.orientation.y = 0.0;
    state_.end_effector_pose.orientation.z = 0.0;
    state_.end_effector_pose.orientation.w = 1.0;
}

ArmControl::~ArmControl() {
    // 无需特殊清理
}

void ArmControl::setupArmParameters() {
    // 设置连杆长度（单位：米）
    params_.link_lengths[0] = 0.2; // 大臂长度
    params_.link_lengths[1] = 0.2; // 小臂长度
    params_.link_lengths[2] = 0.1; // 腕部长度
    params_.link_lengths[3] = 0.05; // 末端执行器长度
    
    // 设置关节限位（单位：弧度）
    params_.joint_limits[0][0] = -M_PI;    // 底座旋转最小值
    params_.joint_limits[0][1] = M_PI;     // 底座旋转最大值
    params_.joint_limits[1][0] = -M_PI/2;  // 肩部抬升最小值
    params_.joint_limits[1][1] = M_PI/2;   // 肩部抬升最大值
    params_.joint_limits[2][0] = -M_PI/2;  // 肘部抬升最小值
    params_.joint_limits[2][1] = M_PI/2;   // 肘部抬升最大值
    params_.joint_limits[3][0] = -M_PI;    // 腕部旋转最小值
    params_.joint_limits[3][1] = M_PI;     // 腕部旋转最大值
}

bool ArmControl::setJointPositions(const std::vector<double>& joint_positions) {
    // 验证输入
    if (joint_positions.size() < 4) {
        ROS_ERROR("关节数量不足，需要至少4个关节值");
        return false;
    }
    
    // 检查关节限位
    if (!validateJointPositions(joint_positions)) {
        ROS_ERROR("关节值超出限位范围");
        return false;
    }
    
    // 更新内部状态
    state_.joint_positions = joint_positions;
    updateEndEffectorPose();
    
        return true;
}

std::vector<double> ArmControl::getJointPositions() const {
    return state_.joint_positions;
}

bool ArmControl::setEndEffectorPose(const geometry_msgs::Pose& pose) {
    // 计算逆运动学
    std::vector<double> joint_positions = computeInverseKinematics(pose);
    
    // 如果逆运动学失败，返回空数组
    if (joint_positions.empty()) {
        ROS_ERROR("逆运动学计算失败或结果超出关节限位");
        return false;
    }
    
    // 设置关节位置
    return setJointPositions(joint_positions);
}

geometry_msgs::Pose ArmControl::getEndEffectorPose() const {
    return state_.end_effector_pose;
}

bool ArmControl::isInitialized() const {
    return state_.is_initialized;
}

bool ArmControl::initialize() {
    // 初始化机械臂，将所有关节设置为零位
    std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0};
    bool success = setJointPositions(home_position);
    
    if (success) {
        state_.is_initialized = true;
    }
    
    return success;
}

bool ArmControl::goHome() {
    // 回到初始位置
    return initialize();
}

bool ArmControl::validateJointPositions(const std::vector<double>& joint_positions) const {
    // 检查关节数量
    if (joint_positions.size() < 4) {
        return false;
    }
    
    // 检查关节限位
    for (size_t i = 0; i < 4 && i < joint_positions.size(); ++i) {
        if (joint_positions[i] < params_.joint_limits[i][0] || 
            joint_positions[i] > params_.joint_limits[i][1]) {
            return false;
        }
    }
    
    return true;
}

void ArmControl::updateEndEffectorPose() {
    // 更新末端执行器位姿
    state_.end_effector_pose = computeForwardKinematics(state_.joint_positions);
}

std::vector<double> ArmControl::computeInverseKinematics(const geometry_msgs::Pose& pose, 
                                                      const std::vector<double>& initial_guess) const {
    // 简单版本的逆运动学
    // 实际应用中应使用正式的IK算法
    
    // 初始化输出
    std::vector<double> joint_positions(4, 0.0);
    
    // 这里只是一个简单的示例实现
    // 只处理简单的位置控制，忽略方向
    const double x = pose.position.x;
    const double y = pose.position.y;
    const double z = pose.position.z;
    
    // 计算底座旋转角度
    joint_positions[0] = std::atan2(y, x);
    
    // 简单的肩部和肘部角度计算
    const double r = std::sqrt(x*x + y*y);
    const double shoulder_angle = std::atan2(z, r);
    
    joint_positions[1] = shoulder_angle;
    joint_positions[2] = (z > 0.3) ? 0.3 : 0.0; // 简单规则
    
    // 保持腕部中立
    joint_positions[3] = -joint_positions[2] - joint_positions[1];
    
    // 检查关节限位
    if (!validateJointPositions(joint_positions)) {
        return std::vector<double>(); // 返回空数组表示失败
    }
    
    return joint_positions;
}

geometry_msgs::Pose ArmControl::computeForwardKinematics(const std::vector<double>& joint_values) const {
    // 简单版本的正向运动学
    // 实际应用中应使用正式的FK算法
    
    geometry_msgs::Pose pose;
    
    // 检查关节数量
    if (joint_values.size() < 4) {
        return pose;
    }
    
    // 这里只是一个简单的示例实现
    const double base_angle = joint_values[0];
    const double shoulder_angle = joint_values[1];
    const double elbow_angle = joint_values[2];
    
    // 简化的机械臂模型
    const double L1 = params_.link_lengths[0]; // 大臂长度
    const double L2 = params_.link_lengths[1]; // 小臂长度
    
    // 平面投影距离
    const double r = L1 * std::cos(shoulder_angle) + L2 * std::cos(shoulder_angle + elbow_angle);
    
    // 计算末端位置
    pose.position.x = r * std::cos(base_angle);
    pose.position.y = r * std::sin(base_angle);
    pose.position.z = L1 * std::sin(shoulder_angle) + L2 * std::sin(shoulder_angle + elbow_angle);
    
    // 简化方向计算，假设末端执行器垂直向下
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    
    return pose;
}

} // namespace servo 