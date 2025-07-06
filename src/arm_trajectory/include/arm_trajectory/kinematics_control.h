#ifndef ARM_TRAJECTORY_KINEMATICS_CONTROL_H
#define ARM_TRAJECTORY_KINEMATICS_CONTROL_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <utility>
#include <geometry_msgs/Pose.h>

#include "arm_trajectory/ForwardKinematics.h"
#include "arm_trajectory/InverseKinematics.h"

namespace arm_trajectory {

/**
 * @brief DH参数结构，用于表示机械臂链接的运动学参数
 */
struct DHParameters {
    std::string joint_type; // "revolute"或"prismatic"
    double theta;  // joint angle (variable for revolute joints)
    double d;      // link offset
    double a;      // link length
    double alpha;  // link twist

    // 默认构造函数
    DHParameters() : joint_type("revolute"), theta(0), d(0), a(0), alpha(0) {}
    
    // 带参数的构造函数
    DHParameters(const std::string& type, double d_val, double theta_val, double a_val, double alpha_val)
        : joint_type(type), theta(theta_val), d(d_val), a(a_val), alpha(alpha_val) {}
};

/**
 * @brief 机械臂运动学控制类，包含核心运动学计算，与ROS无关
 */
class KinematicsControl {
public:
    KinematicsControl();
    ~KinematicsControl();

    /**
     * @brief 计算正向运动学
     * @param joint_values 关节值
     * @return 末端执行器的位姿矩阵
     */
    Eigen::Matrix4d computeForwardKinematics(const std::vector<double>& joint_values);
    
    /**
     * @brief 计算逆向运动学
     * @param target_pose 目标位姿矩阵
     * @param initial_guess 初始关节值猜测
     * @return 关节值
     */
    std::vector<double> computeInverseKinematics(const Eigen::Matrix4d& target_pose, 
                                              const std::vector<double>& initial_guess = {});
    
    /**
     * @brief 设置DH参数
     * @param arm_id 机械臂ID
     */
    void setupDHParameters(const std::string& arm_id);
    
    /**
     * @brief 设置关节限制
     * @param arm_id 机械臂ID
     */
    void setupJointLimits(const std::string& arm_id);
    
    /**
     * @brief 检查关节限制
     * @param joint_values 关节值
     * @return 是否在限制范围内
     */
    bool checkJointLimits(const std::vector<double>& joint_values);
    
    /**
     * @brief 获取DH参数数量
     * @return DH参数数量
     */
    size_t getNumberOfJoints() const;
    
    /**
     * @brief 获取关节限制
     * @return 关节限制数组
     */
    const std::vector<std::pair<double, double>>& getJointLimits() const;

    // ROS消息转换函数
    geometry_msgs::Pose forwardKinematics(const std::vector<double>& joint_values);
    std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                         const std::vector<double>& initial_guess = {});
    
    // 服务回调函数
    bool handleForwardKinematics(arm_trajectory::ForwardKinematics::Request& req,
                               arm_trajectory::ForwardKinematics::Response& res);
    bool handleInverseKinematics(arm_trajectory::InverseKinematics::Request& req,
                               arm_trajectory::InverseKinematics::Response& res);

private:
    std::vector<DHParameters> dh_params_;
    std::vector<std::pair<double, double>> joint_limits_;  // min, max
    
    /**
     * @brief 计算DH变换矩阵
     * @param theta 关节角
     * @param d 偏移距离
     * @param a 链接长度
     * @param alpha 扭转角
     * @return 变换矩阵
     */
    Eigen::Matrix4d computeDHTransform(double theta, double d, double a, double alpha);
};

} // namespace arm_trajectory

#endif // ARM_TRAJECTORY_KINEMATICS_CONTROL_H 