#ifndef TRAJECTORY_KINEMATICS_SERVICE_H
#define TRAJECTORY_KINEMATICS_SERVICE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <utility>

#include "trajectory/ForwardKinematics.h"
#include "trajectory/InverseKinematics.h"
#include "trajectory/kinematics_control.h"

namespace trajectory {

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
    bool handleForwardKinematics(trajectory::ForwardKinematics::Request& req,
                                trajectory::ForwardKinematics::Response& res);
    
    /**
     * @brief 处理逆向运动学服务请求
     */
    bool handleInverseKinematics(trajectory::InverseKinematics::Request& req,
                                trajectory::InverseKinematics::Response& res);

private:
    ros::NodeHandle nh_;
    ros::ServiceServer fk_service_;
    ros::ServiceServer ik_service_;
    
    // 核心运动学控制器
    KinematicsControl kinematics_control_;
};

} // namespace trajectory

#endif // TRAJECTORY_KINEMATICS_SERVICE_H
