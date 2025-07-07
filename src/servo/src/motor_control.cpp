#include <servo/motor_control.h>
#include <algorithm>
#include <cmath>

namespace servo {

MotorControl::MotorControl() {
    // 初始化电机状态列表，预设4个电机（可根据实际需求调整）
    motor_states_.resize(4);
    for (auto& state : motor_states_) {
        state.position = 0;
        state.velocity = 0;
        state.is_initialized = false;
        state.is_active = false;
    }
}

MotorControl::~MotorControl() {
    // 析构时尝试停止所有电机
    for (size_t i = 0; i < motor_states_.size(); ++i) {
        if (motor_states_[i].is_active) {
            stopMotor(i);
        }
    }
}

bool MotorControl::setMotorPosition(uint8_t motor_id, int32_t position,
                                   int16_t velocity,
                                   uint16_t acceleration,
                                   uint16_t deceleration,
                                   bool is_position_mode,
                                   uint16_t position_threshold) {
    // 检查motor_id是否有效
    if (motor_id >= motor_states_.size()) {
        ROS_ERROR("Invalid motor ID: %d", motor_id);
        return false;
    }

    // 限制位置范围
    position = std::max(MIN_POSITION, std::min(position, MAX_POSITION));
    
    // 限制速度范围
    velocity = std::max(MIN_VELOCITY, std::min(velocity, MAX_VELOCITY));
    
    // 更新电机状态
    motor_states_[motor_id].position = position;
    motor_states_[motor_id].velocity = velocity;
    motor_states_[motor_id].is_active = true;
    
    ROS_DEBUG("Setting motor %d position to %d with velocity %d", motor_id, position, velocity);
    
    // 实际应用中，这里会发送命令到电机硬件
    // 此处为示例实现，没有实际控制硬件
    return true;
}

bool MotorControl::setMotorVelocity(uint8_t motor_id, int16_t velocity,
                                  uint16_t acceleration,
                                  uint16_t deceleration) {
    // 速度模式设置
    return setMotorPosition(motor_id, 0, velocity, acceleration, deceleration, false, 0);
}

bool MotorControl::stopMotor(uint8_t motor_id) {
    // 检查motor_id是否有效
    if (motor_id >= motor_states_.size()) {
        ROS_ERROR("Invalid motor ID: %d", motor_id);
        return false;
    }
    
    // 停止电机
    motor_states_[motor_id].velocity = 0;
    motor_states_[motor_id].is_active = false;
    
    ROS_DEBUG("Stopping motor %d", motor_id);
    
    // 实际应用中，这里会发送停止命令到电机硬件
    return true;
}

int32_t MotorControl::getMotorPosition(uint8_t motor_id) const {
    // 检查motor_id是否有效
    if (motor_id >= motor_states_.size()) {
        ROS_ERROR("Invalid motor ID: %d", motor_id);
        return 0;
    }
    
    // 返回当前位置
    return motor_states_[motor_id].position;
}

bool MotorControl::initializeMotor(uint8_t motor_id) {
    // 检查motor_id是否有效
    if (motor_id >= motor_states_.size()) {
        ROS_ERROR("Invalid motor ID: %d", motor_id);
        return false;
    }
    
    // 初始化电机
    motor_states_[motor_id].position = 0;
    motor_states_[motor_id].velocity = 0;
    motor_states_[motor_id].is_initialized = true;
    motor_states_[motor_id].is_active = false;
    
    ROS_INFO("Initialized motor %d", motor_id);
    
    // 实际应用中，这里会发送初始化命令到电机硬件
    return true;
}

} // namespace servo 