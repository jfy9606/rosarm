#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <string>
#include <vector>
#include <iostream>
#include <cmath>

namespace servo {

/**
 * @brief 单个伺服电机的配置
 */
struct ServoConfig {
    int id;                 // 电机ID
    double min_angle;       // 最小角度（弧度）
    double max_angle;       // 最大角度（弧度）
    double default_angle;   // 默认角度（弧度）
    double offset;          // 角度偏移（弧度）
    int min_pos;            // 最小位置值
    int max_pos;            // 最大位置值
    bool inverted;          // 是否反向
    
    ServoConfig() : id(0), min_angle(0), max_angle(M_PI), default_angle(M_PI/2),
                  offset(0), min_pos(0), max_pos(1000), inverted(false) {}
};

/**
 * @brief 舵机控制类，用于控制小臂舵机
 */
class WristControl {
public:
    /**
     * @brief 构造函数
     */
    WristControl();

    /**
     * @brief 析构函数
     */
    ~WristControl();

    /**
     * @brief 初始化舵机
     * @param port 串口设备名
     * @param baudrate 波特率
     * @return 是否初始化成功
     */
    bool init(const std::string& port, int baudrate);

    /**
     * @brief 关闭舵机连接
     */
    void close();

    /**
     * @brief 设置舵机ID
     * @param id 舵机ID
     */
    void setServoID(int id);

    /**
     * @brief 设置舵机位置
     * @param id 舵机ID
     * @param position 目标位置
     * @param speed 舵机速度
     * @return 是否设置成功
     */
    bool setPosition(int id, int position, int speed);

    /**
     * @brief 同时控制多个舵机
     * @param ids 舵机ID列表
     * @param positions 舵机位置列表
     * @param speeds 舵机速度列表
     * @return 是否设置成功
     */
    bool syncWritePosition(const std::vector<int>& ids, 
                           const std::vector<int>& positions, 
                           const std::vector<int>& speeds);

    /**
     * @brief 读取舵机位置
     * @param id 舵机ID
     * @return 当前位置
     */
    int readPosition(int id);

    /**
     * @brief 读取舵机负载
     * @param id 舵机ID
     * @return 当前负载
     */
    int readLoad(int id);

    /**
     * @brief 读取舵机电压
     * @param id 舵机ID
     * @return 当前电压
     */
    int readVoltage(int id);

    /**
     * @brief 读取舵机温度
     * @param id 舵机ID
     * @return 当前温度
     */
    int readTemperature(int id);

    /**
     * @brief 读取舵机是否移动
     * @param id 舵机ID
     * @return 是否在移动
     */
    bool isMoving(int id);

    /**
     * @brief 设置舵机到零位
     * @param id 舵机ID
     * @return 是否设置成功
     */
    bool setZeroPosition(int id);

    /**
     * @brief 角度转换为舵机内部位置值
     * @param angle_deg 角度
     * @return 舵机位置值
     */
    int angleToPulse(double angle_deg);

    /**
     * @brief 舵机内部位置值转换为角度
     * @param pulse 舵机位置值
     * @return 角度
     */
    double pulseToAngle(int pulse);

    /**
     * @brief 获取舵机连接状态
     * @return 是否已连接
     */
    bool isConnected() const { return is_connected_; }

    /**
     * @brief 获取最近的错误
     * @return 错误信息
     */
    std::string getLastError() const { return last_error_; }

    /**
     * @brief 初始化伺服电机配置
     */
    void initServoConfig();
    
    /**
     * @brief 设置伺服电机角度
     * @param servo_id 伺服电机ID
     * @param angle_rad 角度（弧度）
     * @return 是否成功
     */
    bool setServoAngle(int servo_id, double angle_rad);
    
    /**
     * @brief 获取伺服电机角度
     * @param servo_id 伺服电机ID
     * @return 角度（弧度）
     */
    double getServoAngle(int servo_id) const;
    
    /**
     * @brief 设置所有伺服电机角度
     * @param angles_rad 角度数组（弧度）
     * @return 是否成功
     */
    bool setAllServoAngles(const std::vector<double>& angles_rad);
    
    /**
     * @brief 获取所有伺服电机角度
     * @return 角度数组（弧度）
     */
    std::vector<double> getAllServoAngles() const;
    
    /**
     * @brief 将角度转换为位置值
     * @param servo_id 伺服电机ID
     * @param angle_rad 角度（弧度）
     * @return 位置值
     */
    int angleToPosition(int servo_id, double angle_rad) const;
    
    /**
     * @brief 将位置值转换为角度
     * @param servo_id 伺服电机ID
     * @param position 位置值
     * @return 角度（弧度）
     */
    double positionToAngle(int servo_id, int position) const;
    
    /**
     * @brief 获取伺服电机配置
     * @return 伺服电机配置数组
     */
    const std::vector<ServoConfig>& getServoConfig() const;
    
    /**
     * @brief 获取伺服电机数量
     * @return 伺服电机数量
     */
    size_t getServoCount() const;
    
    /**
     * @brief 设置到初始位置
     * @return 是否成功
     */
    bool goHome();

    /**
     * @brief 设置关节位置
     * @param positions 关节位置数组
     */
    void setJointPositions(const std::vector<double>& positions);
    
    /**
     * @brief 获取当前关节位置
     * @return 关节位置数组
     */
    std::vector<double> getJointPositions() const;
    
    /**
     * @brief 更新关节状态
     * @param joint_positions 新的关节位置
     */
    void updateJointState(const std::vector<double>& joint_positions);
    
    /**
     * @brief 获取关节名称
     * @return 关节名称数组
     */
    const std::vector<std::string>& getJointNames() const;

private:
    int servo_id_;                // 当前舵机ID
    bool is_connected_;           // 连接状态
    std::string port_;            // 串口设备名
    int baudrate_;                // 波特率
    std::string last_error_;      // 最近的错误信息
    
    // SCServo API 调用
    void* servoHandler_;           // 舵机句柄，实际上是SCSCL*
    
    // 错误处理
    void setError(const std::string& error);
    void clearError();
    
    // 舵机角度范围转换
    static constexpr double SERVO_MIN_ANGLE = 0.0;      // 最小角度(度)
    static constexpr double SERVO_MAX_ANGLE = 270.0;    // 最大角度(度)
    static constexpr int SERVO_MIN_PULSE = 0;           // 最小位置值
    static constexpr int SERVO_MAX_PULSE = 4095;        // 最大位置值

    std::vector<ServoConfig> servo_configs_;   // 伺服电机配置
    std::vector<double> current_angles_;       // 当前伺服电机角度
    
    // 从ArmControl合并的属性
    std::vector<double> current_joint_positions_;  // 当前关节位置
    std::vector<std::string> joint_names_;         // 关节名称
    
    /**
     * @brief 限制角度在有效范围内
     * @param servo_id 伺服电机ID
     * @param angle_rad 角度（弧度）
     * @return 限制后的角度（弧度）
     */
    double clampAngle(int servo_id, double angle_rad) const;
};

} // namespace servo

#endif // SERVO_CONTROL_H 