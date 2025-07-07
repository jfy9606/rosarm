#ifndef SERVO_MOTOR_CONTROL_H
#define SERVO_MOTOR_CONTROL_H

#include <string>
#include <vector>
#include <ros/ros.h>

namespace servo {

/**
 * @brief 电机控制类，包含电机控制的核心逻辑
 * 负责大臂俯仰和进给电机的基础控制
 */
class MotorControl {
public:
    /**
     * @brief 构造函数
     */
    MotorControl();
    
    /**
     * @brief 析构函数
     */
    ~MotorControl();
    
    /**
     * @brief 设置电机位置
     * @param motor_id 电机ID
     * @param position 目标位置
     * @param velocity 速度
     * @param acceleration 加速度
     * @param deceleration 减速度
     * @param is_position_mode 是否为位置模式(true=绝对位置，false=速度控制)
     * @param position_threshold 位置阈值
     * @return 操作是否成功
     */
    bool setMotorPosition(uint8_t motor_id, int32_t position, 
                         int16_t velocity = 0, 
                         uint16_t acceleration = 0, 
                         uint16_t deceleration = 0,
                         bool is_position_mode = true,
                         uint16_t position_threshold = 10);
    
    /**
     * @brief 设置电机速度
     * @param motor_id 电机ID
     * @param velocity 目标速度
     * @param acceleration 加速度
     * @param deceleration 减速度
     * @return 操作是否成功
     */
    bool setMotorVelocity(uint8_t motor_id, int16_t velocity,
                         uint16_t acceleration = 0,
                         uint16_t deceleration = 0);
    
    /**
     * @brief 停止电机
     * @param motor_id 电机ID
     * @return 操作是否成功
     */
    bool stopMotor(uint8_t motor_id);
    
    /**
     * @brief 获取电机当前位置
     * @param motor_id 电机ID
     * @return 当前位置
     */
    int32_t getMotorPosition(uint8_t motor_id) const;

    /**
     * @brief 初始化电机（回到零位）
     * @param motor_id 电机ID
     * @return 操作是否成功
     */
    bool initializeMotor(uint8_t motor_id);

private:
    struct MotorState {
        int32_t position;
        int16_t velocity;
        bool is_initialized;
        bool is_active;
    };
    
    std::vector<MotorState> motor_states_;
    
    // 电机参数的限制范围
    static const int16_t MAX_VELOCITY = 1000;
    static const int16_t MIN_VELOCITY = -1000;
    static const int32_t MAX_POSITION = 10000;
    static const int32_t MIN_POSITION = -10000;
};

} // namespace servo

#endif // SERVO_MOTOR_CONTROL_H 