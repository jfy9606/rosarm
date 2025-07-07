#include "trajectory/kinematics_control.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace trajectory {

KinematicsControl::KinematicsControl() {
    // 初始化默认机械臂参数
    setupDHParameters("default");
    setupJointLimits("default");
}

KinematicsControl::~KinematicsControl() {
    // 清理资源（如有需要）
}

Eigen::Matrix4d KinematicsControl::computeForwardKinematics(const std::vector<double>& joint_values) {
    // 检查关节数量是否正确
    if (joint_values.size() != dh_params_.size()) {
        std::stringstream ss;
        ss << "Wrong number of joint values: " << joint_values.size() 
           << " instead of " << dh_params_.size();
        throw std::invalid_argument(ss.str());
    }
    
    // 初始化变换矩阵为单位矩阵
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    
    // 累积每个关节的变换
    for (size_t i = 0; i < dh_params_.size(); ++i) {
        // 获取DH参数
        double theta = dh_params_[i].theta;
        double d = dh_params_[i].d;
        double a = dh_params_[i].a;
        double alpha = dh_params_[i].alpha;
        
        // 根据关节类型更新相应参数
        if (dh_params_[i].joint_type == "revolute") { // 旋转关节
            theta += joint_values[i]; // 更新θ
        } else if (dh_params_[i].joint_type == "prismatic") { // 移动关节
            d += joint_values[i];    // 更新d
        }
        
        // 计算此关节的变换矩阵
        Eigen::Matrix4d joint_transform = computeDHTransform(theta, d, a, alpha);
        
        // 累积变换
        transform = transform * joint_transform;
    }
    
    return transform;
}

std::vector<double> KinematicsControl::computeInverseKinematics(const Eigen::Matrix4d& target_pose, 
                                                             const std::vector<double>& initial_guess) {
    // 使用数值优化方法（梯度下降）求解逆运动学
    
    // 如果没有提供初始猜测值，使用关节限制的中点值
    std::vector<double> current_joints;
    if (initial_guess.empty() || initial_guess.size() != dh_params_.size()) {
        current_joints.resize(dh_params_.size());
        for (size_t i = 0; i < dh_params_.size(); i++) {
            current_joints[i] = (joint_limits_[i].first + joint_limits_[i].second) / 2.0;
        }
    } else {
        current_joints = initial_guess;
    }
    
    // 确保初始猜测值符合关节限制
    for (size_t i = 0; i < current_joints.size(); ++i) {
        current_joints[i] = std::max(joint_limits_[i].first, 
                           std::min(joint_limits_[i].second, current_joints[i]));
    }
    
    // 设置优化参数
    const int max_iterations = 100;
    const double convergence_threshold = 0.1; // 厘米
    const double learning_rate = 0.1;
    
    // 提取目标位置（厘米）
    Eigen::Vector3d target_position(target_pose(0, 3), target_pose(1, 3), target_pose(2, 3));
    
    // 提取目标姿态（旋转矩阵）
    Eigen::Matrix3d target_rotation = target_pose.block<3, 3>(0, 0);
    
    // 迭代优化
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 计算当前关节角度下的末端位姿
        Eigen::Matrix4d current_pose = computeForwardKinematics(current_joints);
        
        // 计算位置误差（厘米）
        Eigen::Vector3d current_position(current_pose(0, 3), current_pose(1, 3), current_pose(2, 3));
        Eigen::Vector3d position_error = target_position - current_position;
        
        // 计算旋转误差（简化，仅使用位置误差）
        // 此处可以添加姿态误差计算
        
        // 检查是否收敛
        double error_magnitude = position_error.norm();
        if (error_magnitude < convergence_threshold) {
            return current_joints;
        }
        
        // 计算雅可比矩阵（数值微分）
        std::vector<std::vector<double>> jacobian(3, std::vector<double>(current_joints.size(), 0.0));
        
        const double delta = 0.01; // 数值微分的步长
        
        for (size_t j = 0; j < current_joints.size(); ++j) {
            // 原关节值
            double original_value = current_joints[j];
            
            // 微小扰动
            current_joints[j] += delta;
            
            // 计算扰动后的位姿
            Eigen::Matrix4d perturbed_pose = computeForwardKinematics(current_joints);
            
            // 恢复关节值
            current_joints[j] = original_value;
            
            // 计算偏导数
            jacobian[0][j] = (perturbed_pose(0, 3) - current_position[0]) / delta;
            jacobian[1][j] = (perturbed_pose(1, 3) - current_position[1]) / delta;
            jacobian[2][j] = (perturbed_pose(2, 3) - current_position[2]) / delta;
        }
        
        // 使用转置雅可比矩阵计算关节角度更新
        for (size_t j = 0; j < current_joints.size(); ++j) {
            double update = 0.0;
            for (size_t k = 0; k < 3; ++k) {
                update += jacobian[k][j] * position_error[k];
            }
            
            // 更新关节角度
            current_joints[j] += learning_rate * update;
            
            // 确保关节角度在限制范围内
            current_joints[j] = std::max(joint_limits_[j].first, 
                               std::min(joint_limits_[j].second, current_joints[j]));
        }
    }
    
    // 如果未收敛，返回最佳近似
    return current_joints;
}

Eigen::Matrix4d KinematicsControl::computeDHTransform(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    // 预计算三角函数
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    
    // 第一行
    T(0, 0) = cos_theta;
    T(0, 1) = -sin_theta * cos_alpha;
    T(0, 2) = sin_theta * sin_alpha;
    T(0, 3) = a * cos_theta;
    
    // 第二行
    T(1, 0) = sin_theta;
    T(1, 1) = cos_theta * cos_alpha;
    T(1, 2) = -cos_theta * sin_alpha;
    T(1, 3) = a * sin_theta;
    
    // 第三行
    T(2, 0) = 0;
    T(2, 1) = sin_alpha;
    T(2, 2) = cos_alpha;
    T(2, 3) = d;
    
    // 第四行是 [0, 0, 0, 1]（已由Identity()设置）
    
    return T;
}

bool KinematicsControl::checkJointLimits(const std::vector<double>& joint_values) {
    if (joint_values.size() != joint_limits_.size()) {
        return false;
    }
    
    for (size_t i = 0; i < joint_values.size(); i++) {
        if (joint_values[i] < joint_limits_[i].first || 
            joint_values[i] > joint_limits_[i].second) {
            return false;
        }
    }
    
    return true;
}

void KinematicsControl::setupDHParameters(const std::string& arm_id) {
    // 清除现有参数
    dh_params_.clear();
    
    if (arm_id == "default") {
        // 初始化机械臂参数（单位：cm）
        double a1 = 13.0; // 第一个连杆长度
        double a2 = 13.0; // 第二个连杆长度
        double a3 = 25.0; // 第三个连杆长度
        
        // 添加所有关节的DH参数
        DHParameters param1;
        param1.joint_type = "revolute";
        param1.theta = 0.0;
        param1.d = 0.0;
        param1.a = a1;
        param1.alpha = M_PI/2;
        dh_params_.push_back(param1);  // 底座旋转关节(θ1)
        
        DHParameters param2;
        param2.joint_type = "prismatic";
        param2.theta = M_PI/4;
        param2.d = 0.0;
        param2.a = 0.0;
        param2.alpha = 0.0;
        dh_params_.push_back(param2); // 伸缩关节(d2)
        
        DHParameters param3;
        param3.joint_type = "revolute";
        param3.theta = 0.0;
        param3.d = a2;
        param3.a = 0.0;
        param3.alpha = M_PI/2;
        dh_params_.push_back(param3);  // 肩部关节(θ3)
        
        DHParameters param4;
        param4.joint_type = "revolute";
        param4.theta = 0.0;
        param4.d = 0.0;
        param4.a = a3;
        param4.alpha = M_PI/2;
        dh_params_.push_back(param4);  // 肘部关节(θ4)
        
        DHParameters param5;
        param5.joint_type = "revolute";
        param5.theta = M_PI/2;
        param5.d = 0.0;
        param5.a = 0.0;
        param5.alpha = M_PI/2;
        dh_params_.push_back(param5); // 固定关节(θ5)
        
        DHParameters param6;
        param6.joint_type = "prismatic";
        param6.theta = M_PI/2;
        param6.d = 0.0;
        param6.a = 0.0;
        param6.alpha = 0.0;
        dh_params_.push_back(param6);  // 末端伸缩(d6)
    } else {
        // 使用默认参数
        setupDHParameters("default");
    }
}

void KinematicsControl::setupJointLimits(const std::string& arm_id) {
    // 清除现有限制
    joint_limits_.clear();
    
    if (arm_id == "default") {
        // 格式: (min, max)
        // joint1: θ1 (rad) ±180°
        joint_limits_.push_back(std::make_pair(-M_PI, M_PI));
        
        // joint2: d2 (cm) 0-43cm
        joint_limits_.push_back(std::make_pair(0.0, 43.0));
        
        // joint3: θ3 (rad) ±90°
        joint_limits_.push_back(std::make_pair(-M_PI/2, M_PI/2));
        
        // joint4: θ4 (rad) 0-180°
        joint_limits_.push_back(std::make_pair(0.0, M_PI));
        
        // joint5: θ5 (rad) 固定在90°
        joint_limits_.push_back(std::make_pair(M_PI/2, M_PI/2));
        
        // joint6: d6 (cm) 5-15cm
        joint_limits_.push_back(std::make_pair(5.0, 15.0));
    } else {
        // 使用默认限制
        setupJointLimits("default");
    }
}

size_t KinematicsControl::getNumberOfJoints() const {
    return dh_params_.size();
}

const std::vector<std::pair<double, double>>& KinematicsControl::getJointLimits() const {
    return joint_limits_;
}

// 添加从kinematics_service.cpp迁移的代码 -- ROS服务处理函数

geometry_msgs::Pose KinematicsControl::forwardKinematics(const std::vector<double>& joint_values) {
    // 调用核心运动学计算函数
    Eigen::Matrix4d transform = computeForwardKinematics(joint_values);
    
    // 将Eigen矩阵转换为ROS Pose消息
    geometry_msgs::Pose result_pose;
    
    // 设置位置（从矩阵的平移部分提取）
    result_pose.position.x = transform(0, 3);
    result_pose.position.y = transform(1, 3);
    result_pose.position.z = transform(2, 3);
    
    // 从旋转矩阵提取四元数
    Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rotation_matrix);
    
    // 设置方向
    result_pose.orientation.x = q.x();
    result_pose.orientation.y = q.y();
    result_pose.orientation.z = q.z();
    result_pose.orientation.w = q.w();
    
    return result_pose;
}

std::vector<double> KinematicsControl::inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                                      const std::vector<double>& initial_guess) {
    // 将ROS Pose转换为Eigen矩阵
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    
    // 设置旋转部分
    Eigen::Quaterniond q(
        target_pose.orientation.w,
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z
    );
    transform.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    
    // 设置平移部分
    transform(0, 3) = target_pose.position.x;
    transform(1, 3) = target_pose.position.y;
    transform(2, 3) = target_pose.position.z;
    
    // 调用核心逆运动学计算函数
    return computeInverseKinematics(transform, initial_guess);
}

bool KinematicsControl::handleForwardKinematics(trajectory::ForwardKinematics::Request& req,
                                             trajectory::ForwardKinematics::Response& res) {
    // 检查输入
    if (req.joint_values.size() < getNumberOfJoints()) {
        ROS_ERROR("Forward kinematics request has insufficient joint values");
        return false;
    }
    
    // 计算正向运动学
    geometry_msgs::Pose result_pose = forwardKinematics(req.joint_values);
    
    // 设置响应
    res.pose = result_pose;
    
    return true;
}

bool KinematicsControl::handleInverseKinematics(trajectory::InverseKinematics::Request& req,
                                             trajectory::InverseKinematics::Response& res) {
    // 计算逆向运动学
    std::vector<double> result_joints = inverseKinematics(req.pose, req.initial_guess);
    
    // 设置响应
    res.joint_values = result_joints;
    res.success = !result_joints.empty();
    
    return true;
}

} // namespace trajectory 