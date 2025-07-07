#include <servo/wrist_control.h>
#include <algorithm>
#include <cmath>

namespace servo {

WristControl::WristControl() {
    // 初始化伺服电机配置
    initServoConfig();
    
    // 初始化当前角度为默认值
    current_angles_.resize(servo_configs_.size());
    for (size_t i = 0; i < servo_configs_.size(); ++i) {
        current_angles_[i] = servo_configs_[i].default_angle;
    }
    
    // 从ArmControl合并的初始化代码
    // 初始化关节名称
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    
    // 初始化关节位置
    current_joint_positions_.resize(joint_names_.size(), 0.0);
}

WristControl::~WristControl() {
    // 无需特殊清理
}

void WristControl::initServoConfig() {
    servo_configs_.clear();
    
    // 添加伺服电机配置
    // 手腕伺服1（旋转）
    ServoConfig servo1;
    servo1.id = 1;
    servo1.min_angle = -M_PI/2;  // -90度
    servo1.max_angle = M_PI/2;   // 90度
    servo1.default_angle = 0.0;  // 0度
    servo1.offset = 0.0;
    servo1.min_pos = 0;
    servo1.max_pos = 4095;
    servo1.inverted = false;
    servo_configs_.push_back(servo1);
    
    // 手腕伺服2（俯仰）
    ServoConfig servo2;
    servo2.id = 2;
    servo2.min_angle = -M_PI/4;  // -45度
    servo2.max_angle = M_PI/4;   // 45度
    servo2.default_angle = 0.0;  // 0度
    servo2.offset = 0.0;
    servo2.min_pos = 0;
    servo2.max_pos = 4095;
    servo2.inverted = false;
    servo_configs_.push_back(servo2);
    
    // 夹爪伺服
    ServoConfig servo3;
    servo3.id = 3;
    servo3.min_angle = 0.0;      // 0度（关闭）
    servo3.max_angle = M_PI/2;   // 90度（打开）
    servo3.default_angle = 0.0;  // 默认关闭
    servo3.offset = 0.0;
    servo3.min_pos = 0;
    servo3.max_pos = 4095;
    servo3.inverted = false;
    servo_configs_.push_back(servo3);
}

bool WristControl::setServoAngle(int servo_id, double angle_rad) {
    // 检查伺服ID有效性
    int index = -1;
    for (size_t i = 0; i < servo_configs_.size(); ++i) {
        if (servo_configs_[i].id == servo_id) {
            index = static_cast<int>(i);
            break;
        }
    }
    
    if (index < 0 || index >= static_cast<int>(servo_configs_.size())) {
        return false;  // 无效的伺服ID
    }
    
    // 限制角度在有效范围内
    double clamped_angle = clampAngle(index, angle_rad);
    
    // 更新当前角度
    current_angles_[index] = clamped_angle;
    
    return true;
}

double WristControl::getServoAngle(int servo_id) const {
    // 检查伺服ID有效性
    int index = -1;
    for (size_t i = 0; i < servo_configs_.size(); ++i) {
        if (servo_configs_[i].id == servo_id) {
            index = static_cast<int>(i);
            break;
        }
    }
    
    if (index < 0 || index >= static_cast<int>(servo_configs_.size())) {
        return 0.0;  // 无效的伺服ID，返回默认值
    }
    
    return current_angles_[index];
}

bool WristControl::setAllServoAngles(const std::vector<double>& angles_rad) {
    // 检查角度数组大小
    if (angles_rad.size() != servo_configs_.size()) {
        return false;
    }
    
    // 设置所有伺服电机角度
    for (size_t i = 0; i < angles_rad.size(); ++i) {
        current_angles_[i] = clampAngle(i, angles_rad[i]);
    }
    
    return true;
}

std::vector<double> WristControl::getAllServoAngles() const {
    return current_angles_;
}

int WristControl::angleToPosition(int servo_id, double angle_rad) const {
    // 检查伺服ID有效性
    int index = -1;
    for (size_t i = 0; i < servo_configs_.size(); ++i) {
        if (servo_configs_[i].id == servo_id) {
            index = static_cast<int>(i);
            break;
        }
    }
    
    if (index < 0 || index >= static_cast<int>(servo_configs_.size())) {
        return 0;  // 无效的伺服ID，返回默认值
    }
    
    // 限制角度在有效范围内
    double clamped_angle = clampAngle(index, angle_rad);
    
    // 获取伺服配置
    const ServoConfig& config = servo_configs_[index];
    
    // 应用偏移
    clamped_angle += config.offset;
    
    // 计算角度范围和位置范围
    double angle_range = config.max_angle - config.min_angle;
    double pos_range = static_cast<double>(config.max_pos - config.min_pos);
    
    // 计算归一化角度（0-1范围）
    double normalized_angle = (clamped_angle - config.min_angle) / angle_range;
    
    // 反转方向（如果需要）
    if (config.inverted) {
        normalized_angle = 1.0 - normalized_angle;
    }
    
    // 计算位置值
    int position = static_cast<int>(config.min_pos + normalized_angle * pos_range + 0.5);
    
    // 确保位置在有效范围内
    return std::max(config.min_pos, std::min(config.max_pos, position));
}

double WristControl::positionToAngle(int servo_id, int position) const {
    // 检查伺服ID有效性
    int index = -1;
    for (size_t i = 0; i < servo_configs_.size(); ++i) {
        if (servo_configs_[i].id == servo_id) {
            index = static_cast<int>(i);
            break;
        }
    }
    
    if (index < 0 || index >= static_cast<int>(servo_configs_.size())) {
        return 0.0;  // 无效的伺服ID，返回默认值
    }
    
    // 获取伺服配置
    const ServoConfig& config = servo_configs_[index];
    
    // 限制位置在有效范围内
    position = std::max(config.min_pos, std::min(config.max_pos, position));
    
    // 计算位置范围和角度范围
    double pos_range = static_cast<double>(config.max_pos - config.min_pos);
    double angle_range = config.max_angle - config.min_angle;
    
    // 计算归一化位置（0-1范围）
    double normalized_pos = static_cast<double>(position - config.min_pos) / pos_range;
    
    // 反转方向（如果需要）
    if (config.inverted) {
        normalized_pos = 1.0 - normalized_pos;
    }
    
    // 计算角度
    double angle = config.min_angle + normalized_pos * angle_range;
    
    // 应用偏移
    angle -= config.offset;
    
    return angle;
}

const std::vector<ServoConfig>& WristControl::getServoConfig() const {
    return servo_configs_;
}

size_t WristControl::getServoCount() const {
    return servo_configs_.size();
}

bool WristControl::goHome() {
    // 将所有伺服电机设置为默认位置
    for (size_t i = 0; i < servo_configs_.size(); ++i) {
        current_angles_[i] = servo_configs_[i].default_angle;
    }
    
    return true;
}

double WristControl::clampAngle(int index, double angle_rad) const {
    if (index < 0 || index >= static_cast<int>(servo_configs_.size())) {
        return 0.0;  // 无效的索引，返回默认值
    }
    
    const ServoConfig& config = servo_configs_[index];
    return std::max(config.min_angle, std::min(config.max_angle, angle_rad));
}

// 从ArmControl合并的方法实现

void WristControl::setJointPositions(const std::vector<double>& positions) {
    // 检查位置数量是否匹配
    if (positions.size() != joint_names_.size()) {
        return;
    }
    
    // 更新当前关节位置
    current_joint_positions_ = positions;
}

std::vector<double> WristControl::getJointPositions() const {
    return current_joint_positions_;
}

void WristControl::updateJointState(const std::vector<double>& joint_positions) {
    // 检查位置数量是否匹配
    if (joint_positions.size() != current_joint_positions_.size()) {
        return;
    }
    
    // 更新当前关节位置
    current_joint_positions_ = joint_positions;
}

const std::vector<std::string>& WristControl::getJointNames() const {
    return joint_names_;
}

} // namespace servo 