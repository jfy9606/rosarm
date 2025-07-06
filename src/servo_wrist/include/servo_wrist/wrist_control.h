#ifndef SERVO_WRIST_WRIST_CONTROL_H
#define SERVO_WRIST_WRIST_CONTROL_H

#include <string>
#include <vector>
#include <cmath>

namespace servo_wrist {

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
 * @brief 手腕控制类，包含手腕运动控制的核心逻辑，与ROS无关
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

private:
    std::vector<ServoConfig> servo_configs_;   // 伺服电机配置
    std::vector<double> current_angles_;       // 当前伺服电机角度
    
    /**
     * @brief 限制角度在有效范围内
     * @param servo_id 伺服电机ID
     * @param angle_rad 角度（弧度）
     * @return 限制后的角度（弧度）
     */
    double clampAngle(int servo_id, double angle_rad) const;
};

} // namespace servo_wrist

#endif // SERVO_WRIST_WRIST_CONTROL_H 