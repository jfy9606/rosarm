#ifndef ARM_TRAJECTORY_KINEMATICS_SERVICE_H
#define ARM_TRAJECTORY_KINEMATICS_SERVICE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <utility>

#include "arm_trajectory/ForwardKinematics.h"
#include "arm_trajectory/InverseKinematics.h"
#include "arm_trajectory/kinematics_control.h"

namespace arm_trajectory {

/**
 * @brief 运动学服务类，处理ROS服务请求并使用KinematicsControl进行核心计算
 */
class KinematicsService {
public:
    KinematicsService(ros::NodeHandle& nh);
    ~KinematicsService();

    /**
     * @brief 计算正向运动学并返回ROS姿态消息
     * @param joint_values 关节值
     * @return 末端执行器的姿态
     */
    geometry_msgs::Pose forwardKinematics(const std::vector<double>& joint_values);
    
    /**
     * @brief 计算逆向运动学
     * @param target_pose 目标姿态
     * @param initial_guess 初始关节值猜测
     * @return 关节值
     */
    std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                         const std::vector<double>& initial_guess);
    
    /**
     * @brief 处理正向运动学服务请求
     */
    bool handleForwardKinematics(arm_trajectory::ForwardKinematics::Request& req,
                                arm_trajectory::ForwardKinematics::Response& res);
    
    /**
     * @brief 处理逆向运动学服务请求
     */
    bool handleInverseKinematics(arm_trajectory::InverseKinematics::Request& req,
                                arm_trajectory::InverseKinematics::Response& res);

private:
    ros::NodeHandle nh_;
    ros::ServiceServer fk_service_;
    ros::ServiceServer ik_service_;
    
    // 核心运动学控制器
    KinematicsControl kinematics_control_;
};

} // namespace arm_trajectory

#endif // ARM_TRAJECTORY_KINEMATICS_SERVICE_H
