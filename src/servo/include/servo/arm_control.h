#ifndef SERVO_ARM_CONTROL_H
#define SERVO_ARM_CONTROL_H

#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace servo {

/**
 * @brief 机械臂控制类，提供机械臂的运动学和控制功能
 */
class ArmControl {
public:
    /**
     * @brief 构造函数
     */
    ArmControl();
    
    /**
     * @brief 析构函数
     */
    ~ArmControl();

    /**
     * @brief 设置关节位置
     * @param joint_positions 关节位置数组
     * @return 操作是否成功
     */
    bool setJointPositions(const std::vector<double>& joint_positions);
    
    /**
     * @brief 获取关节位置
     * @return 当前关节位置数组
     */
    std::vector<double> getJointPositions() const;
    
    /**
     * @brief 设置末端执行器位姿
     * @param pose 目标位姿
     * @return 操作是否成功
     */
    bool setEndEffectorPose(const geometry_msgs::Pose& pose);
    
    /**
     * @brief 获取末端执行器位姿
     * @return 当前末端执行器位姿
     */
    geometry_msgs::Pose getEndEffectorPose() const;
    
    /**
     * @brief 返回机械臂是否已初始化
     * @return 初始化状态
     */
    bool isInitialized() const;
    
    /**
     * @brief 初始化机械臂（回到零位）
     * @return 操作是否成功
     */
    bool initialize();
    
    /**
     * @brief 让机械臂回到初始位置
     * @return 操作是否成功
     */
    bool goHome();
    
    /**
     * @brief 计算正向运动学
     * @param joint_values 关节角度值
     * @return 末端执行器位姿
     */
    geometry_msgs::Pose computeForwardKinematics(const std::vector<double>& joint_values) const;
    
    /**
     * @brief 计算逆向运动学
     * @param pose 目标位姿
     * @param initial_guess 初始猜测值（可选）
     * @return 关节角度值
     */
    std::vector<double> computeInverseKinematics(const geometry_msgs::Pose& pose, 
                                               const std::vector<double>& initial_guess = std::vector<double>()) const;

private:
    // 机械臂参数
    struct ArmParameters {
        double link_lengths[4];  // 连杆长度
        double joint_limits[4][2];  // 关节限位
    };
    
    // 机械臂状态
    struct ArmState {
        std::vector<double> joint_positions;
        geometry_msgs::Pose end_effector_pose;
        bool is_initialized;
    };
    
    ArmParameters params_;
    ArmState state_;
    
    /**
     * @brief 设置机械臂参数
     */
    void setupArmParameters();
    
    /**
     * @brief 验证关节位置是否在限位内
     * @param joint_positions 关节位置数组
     * @return 是否有效
     */
    bool validateJointPositions(const std::vector<double>& joint_positions) const;
    
    /**
     * @brief 更新末端执行器位姿
     */
    void updateEndEffectorPose();
};

} // namespace servo

#endif // SERVO_ARM_CONTROL_H 