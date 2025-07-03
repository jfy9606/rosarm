#include "arm_trajectory/kinematics_service.h"
#include "arm_trajectory/ForwardKinematics.h"
#include "arm_trajectory/InverseKinematics.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>
#include <string>

namespace arm_trajectory {

KinematicsService::KinematicsService(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize service servers
    fk_service_ = nh_.advertiseService("forward_kinematics", 
                                       &KinematicsService::handleForwardKinematics, this);
    ik_service_ = nh_.advertiseService("inverse_kinematics", 
                                       &KinematicsService::handleInverseKinematics, this);
    
    // Load default arm configuration
    setupDHParameters("arm1");
    setupJointLimits("arm1");
    
    ROS_INFO("Kinematics service initialized");
}

KinematicsService::~KinematicsService() {
    // Cleanup if necessary
}

geometry_msgs::Pose KinematicsService::forwardKinematics(const std::vector<double>& joint_values) {
    // 检查关节数量是否正确
    if (joint_values.size() != dh_params_.size()) {
        ROS_WARN("Forward kinematics called with wrong number of joint values: %zu instead of %zu",
                 joint_values.size(), dh_params_.size());
        
        // 返回一个默认姿态
        geometry_msgs::Pose default_pose;
        default_pose.position.x = 0.0;
        default_pose.position.y = 0.0;
        default_pose.position.z = 0.0;
        default_pose.orientation.w = 1.0;
        return default_pose;
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
    
    // 从变换矩阵提取位姿
    geometry_msgs::Pose pose;
    
    // 设置位置（单位：米）
    pose.position.x = transform(0, 3) / 100.0; // 转换为米
    pose.position.y = transform(1, 3) / 100.0;
    pose.position.z = transform(2, 3) / 100.0;
    
    // 从旋转矩阵提取四元数
    Eigen::Matrix3d rotation_matrix = transform.block<3, 3>(0, 0);
    Eigen::Quaterniond quat(rotation_matrix);
    
    // 设置方向
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    
    return pose;
}

std::vector<double> KinematicsService::inverseKinematics(const geometry_msgs::Pose& target_pose, 
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
    const double convergence_threshold = 0.001; // 米
    const double learning_rate = 0.1;
    
    // 目标位置（厘米）
    Eigen::Vector3d target_position(
        target_pose.position.x * 100.0, 
        target_pose.position.y * 100.0, 
        target_pose.position.z * 100.0
    );
    
    // 目标姿态
    Eigen::Quaterniond target_orientation(
        target_pose.orientation.w, 
        target_pose.orientation.x, 
        target_pose.orientation.y, 
        target_pose.orientation.z
    );
    
    // 迭代优化
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 计算当前关节角度下的末端位姿
        geometry_msgs::Pose current_pose = forwardKinematics(current_joints);
        
        // 计算位置误差（厘米）
        Eigen::Vector3d current_position(
            current_pose.position.x * 100.0, 
            current_pose.position.y * 100.0, 
            current_pose.position.z * 100.0
        );
        
        Eigen::Vector3d position_error = target_position - current_position;
        
        // 计算姿态误差（简化，仅使用位置误差）
        
        // 检查是否收敛
        double error_magnitude = position_error.norm();
        if (error_magnitude < convergence_threshold) {
            ROS_INFO("IK converged after %d iterations, error: %.3f cm", iter, error_magnitude);
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
            geometry_msgs::Pose perturbed_pose = forwardKinematics(current_joints);
            
            // 恢复关节值
            current_joints[j] = original_value;
            
            // 计算偏导数
            jacobian[0][j] = (perturbed_pose.position.x * 100.0 - current_position[0]) / delta;
            jacobian[1][j] = (perturbed_pose.position.y * 100.0 - current_position[1]) / delta;
            jacobian[2][j] = (perturbed_pose.position.z * 100.0 - current_position[2]) / delta;
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
    
    ROS_WARN("IK failed to converge, returning best approximation");
    return current_joints;
}

Eigen::Matrix4d KinematicsService::computeDHTransform(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    // Pre-compute trig functions
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    
    // First row
    T(0, 0) = cos_theta;
    T(0, 1) = -sin_theta * cos_alpha;
    T(0, 2) = sin_theta * sin_alpha;
    T(0, 3) = a * cos_theta;
    
    // Second row
    T(1, 0) = sin_theta;
    T(1, 1) = cos_theta * cos_alpha;
    T(1, 2) = -cos_theta * sin_alpha;
    T(1, 3) = a * sin_theta;
    
    // Third row
    T(2, 0) = 0;
    T(2, 1) = sin_alpha;
    T(2, 2) = cos_alpha;
    T(2, 3) = d;
    
    // Fourth row is [0, 0, 0, 1] (already set by Identity())
    
    return T;
}

bool KinematicsService::checkJointLimits(const std::vector<double>& joint_values) {
    if (joint_values.size() != joint_limits_.size()) {
        ROS_WARN("Joint limit check called with wrong number of joint values");
        return false;
    }
    
    for (size_t i = 0; i < joint_values.size(); i++) {
        if (joint_values[i] < joint_limits_[i].first || 
            joint_values[i] > joint_limits_[i].second) {
            ROS_WARN("Joint %zu out of limits: %.3f not in [%.3f, %.3f]", 
                     i, joint_values[i], joint_limits_[i].first, joint_limits_[i].second);
            return false;
        }
    }
    
    return true;
}

void KinematicsService::setupDHParameters(const std::string& arm_id) {
    // Clear existing parameters
    dh_params_.clear();
    
    if (arm_id == "arm1") {
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
        ROS_WARN("Unknown arm ID: %s, using default parameters", arm_id.c_str());
        // Use default parameters
        setupDHParameters("arm1");
    }
}

void KinematicsService::setupJointLimits(const std::string& arm_id) {
    // Clear existing limits
    joint_limits_.clear();
    
    if (arm_id == "arm1") {
        // Format: (min, max)
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
        ROS_WARN("Unknown arm ID: %s, using default joint limits", arm_id.c_str());
        // Use default limits
        setupJointLimits("arm1");
    }
}

bool KinematicsService::handleForwardKinematics(arm_trajectory::ForwardKinematics::Request& req,
                                                arm_trajectory::ForwardKinematics::Response& res) {
    ROS_INFO("Received forward kinematics request for arm %s", req.arm_id.c_str());
    
    // Load DH parameters for the requested arm
    setupDHParameters(req.arm_id);
    setupJointLimits(req.arm_id);
    
    try {
        // Check that we have the right number of joint values
        if (req.joint_values.size() != dh_params_.size()) {
            std::stringstream ss;
            ss << "Wrong number of joint values: " << req.joint_values.size() 
               << " instead of " << dh_params_.size();
            res.success = false;
            res.message = ss.str();
            return true;
        }
        
        // Check joint limits
        if (!checkJointLimits(req.joint_values)) {
            res.success = false;
            res.message = "Joint values exceed limits";
            return true;
        }
        
        // Compute forward kinematics
        res.end_effector_pose = forwardKinematics(req.joint_values);
        res.success = true;
        res.message = "Forward kinematics computed successfully";
    } catch (const std::exception& e) {
        res.success = false;
        res.message = std::string("Error computing forward kinematics: ") + e.what();
    }
    
    return true;
}

bool KinematicsService::handleInverseKinematics(arm_trajectory::InverseKinematics::Request& req,
                                                arm_trajectory::InverseKinematics::Response& res) {
    ROS_INFO("Received inverse kinematics request for arm %s", req.arm_id.c_str());
    
    // Load DH parameters for the requested arm
    setupDHParameters(req.arm_id);
    setupJointLimits(req.arm_id);
    
    try {
        std::vector<double> initial_guess;
        if (req.use_initial_guess && req.initial_guess.size() == dh_params_.size()) {
            initial_guess = req.initial_guess;
        } else {
            // Use default joint values if no initial guess is provided
            initial_guess.resize(dh_params_.size());
            for (size_t i = 0; i < dh_params_.size(); i++) {
                // Use the middle of the joint range as default
                initial_guess[i] = (joint_limits_[i].first + joint_limits_[i].second) / 2.0;
            }
        }
        
        // Compute inverse kinematics
        res.joint_values = inverseKinematics(req.target_pose, initial_guess);
        
        // Check if the solution is valid
        if (checkJointLimits(res.joint_values)) {
            // Verify the solution by computing forward kinematics
            geometry_msgs::Pose forward_pose = forwardKinematics(res.joint_values);
            
            // Calculate fitness score based on position error
            double x_diff = forward_pose.position.x - req.target_pose.position.x;
            double y_diff = forward_pose.position.y - req.target_pose.position.y;
            double z_diff = forward_pose.position.z - req.target_pose.position.z;
            res.fitness_score = sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff);
            
            res.success = true;
            res.message = "Inverse kinematics computed successfully";
        } else {
            res.success = false;
            res.message = "Inverse kinematics solution exceeds joint limits";
            res.fitness_score = 1.0;  // Bad fitness
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.message = std::string("Error computing inverse kinematics: ") + e.what();
        res.fitness_score = 1.0;  // Bad fitness
    }
    
    return true;
}

} // namespace arm_trajectory 