#include "arm_trajectory/kinematics_utils.h"
#include "arm_trajectory/ForwardKinematics.h"
#include "arm_trajectory/InverseKinematics.h"
#include <cmath>
#include <QMatrix3x3>

namespace arm_trajectory {

KinematicsUtils::KinematicsUtils(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize service clients
    fk_client_ = nh_.serviceClient<arm_trajectory::ForwardKinematics>("/forward_kinematics");
    ik_client_ = nh_.serviceClient<arm_trajectory::InverseKinematics>("/inverse_kinematics");
    
    // Load default arm configuration
    setupDHParameters();
    setupJointLimits();
    
    ROS_INFO("KinematicsUtils initialized");
}

KinematicsUtils::~KinematicsUtils() {
    // Cleanup if necessary
}

geometry_msgs::Pose KinematicsUtils::forwardKinematics(const std::vector<double>& joint_values) {
    // Try to use the ROS service first
    if (fk_client_.exists()) {
        arm_trajectory::ForwardKinematics srv;
        srv.request.joint_values = joint_values;
        srv.request.arm_id = "arm1";  // Default arm ID
        
        if (fk_client_.call(srv)) {
            return srv.response.end_effector_pose;
        } else {
            ROS_WARN("Forward kinematics service call failed, using local computation");
        }
    }
    
    // Fall back to local computation
    return localForwardKinematics(joint_values);
}

std::vector<double> KinematicsUtils::inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                                     const std::vector<double>& initial_guess) {
    // Try to use the ROS service first
    if (ik_client_.exists()) {
        arm_trajectory::InverseKinematics srv;
        srv.request.target_pose = target_pose;
        srv.request.arm_id = "arm1";  // Default arm ID
        
        // Use initial guess if provided
        if (!initial_guess.empty()) {
            srv.request.initial_guess = initial_guess;
            srv.request.use_initial_guess = true;
        } else {
            srv.request.use_initial_guess = false;
        }
        
        if (ik_client_.call(srv)) {
            if (srv.response.success) {
                return srv.response.joint_values;
            } else {
                ROS_WARN("Inverse kinematics calculation failed: %s", srv.response.message.c_str());
            }
        } else {
            ROS_WARN("Inverse kinematics service call failed, using local computation");
        }
    }
    
    // Fall back to local computation
    return localInverseKinematics(target_pose, initial_guess);
}

bool KinematicsUtils::checkJointLimits(const std::vector<double>& joint_values) {
    if (joint_values.size() != joint_limits_.size()) {
        ROS_WARN("Joint limit check called with wrong number of joint values: %zu vs %zu",
                 joint_values.size(), joint_limits_.size());
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

void KinematicsUtils::setupDHParameters(const std::string& arm_id) {
    // Clear existing parameters
    dh_params_.clear();
    
    if (arm_id == "arm1") {
        // 添加DH参数，参考arm_params.yaml中的定义
        // 格式: [theta, d, a, alpha, joint_type]
        // joint_type: 0=revolute（旋转关节）, 1=prismatic（移动关节）
        
        // joint1: ['revolute', 0, 0, 13, 1.5708]      # 底座旋转关节
        dh_params_.push_back(DHParam(0, 0, 13, M_PI/2, 0));
        
        // joint2: ['prismatic', 0, 0.7854, 0, 0]      # 伸缩关节
        dh_params_.push_back(DHParam(M_PI/4, 0, 0, 0, 1));
        
        // joint3: ['revolute', 13, 0, 0, 1.5708]      # 肩部关节
        dh_params_.push_back(DHParam(0, 13, 0, M_PI/2, 0));
        
        // joint4: ['revolute', 0, 0, 25, 1.5708]      # 肘部关节
        dh_params_.push_back(DHParam(0, 0, 25, M_PI/2, 0));
        
        // joint5: ['revolute', 0, 1.5708, 0, 1.5708]  # 固定关节
        dh_params_.push_back(DHParam(M_PI/2, 0, 0, M_PI/2, 0));
        
        // joint6: ['prismatic', 0, 1.5708, 0, 0]      # 末端伸缩
        dh_params_.push_back(DHParam(M_PI/2, 0, 0, 0, 1));
    } else {
        ROS_WARN("Unknown arm ID: %s, using default parameters", arm_id.c_str());
        // Use default parameters
        setupDHParameters("arm1");
    }
}

void KinematicsUtils::setupJointLimits(const std::string& arm_id) {
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

QMatrix4x4 KinematicsUtils::computeDHTransform(double theta, double d, double a, double alpha) {
    QMatrix4x4 T;
    T.setToIdentity();
    
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

double KinematicsUtils::degToRad(double deg) {
    return deg * M_PI / 180.0;
}

double KinematicsUtils::radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

geometry_msgs::Pose KinematicsUtils::localForwardKinematics(const std::vector<double>& joint_values) {
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
    QMatrix4x4 T;
    T.setToIdentity();
    
    // 累积每个关节的变换
    for (size_t i = 0; i < dh_params_.size() && i < joint_values.size(); ++i) {
        // 获取DH参数
        double theta = dh_params_[i].theta;
        double d = dh_params_[i].d;
        double a = dh_params_[i].a;
        double alpha = dh_params_[i].alpha;
        
        // 根据关节类型更新参数
        if (dh_params_[i].joint_type == 0) {  // 旋转关节
            theta += joint_values[i]; // 更新θ
        } else {  // 移动关节
            d += joint_values[i];    // 更新d
        }
        
        // 计算此关节的变换矩阵
        QMatrix4x4 Ti = computeDHTransform(theta, d, a, alpha);
        
        // 累积变换
        T = T * Ti;
    }
    
    // 从变换矩阵中提取位姿
    geometry_msgs::Pose pose;
    
    // 设置位置（单位：米）
    pose.position.x = T(0, 3) / 100.0; // 转换为米
    pose.position.y = T(1, 3) / 100.0;
    pose.position.z = T(2, 3) / 100.0;
    
    // 从旋转矩阵提取四元数
    QMatrix3x3 rotation_matrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation_matrix(i, j) = T(i, j);
        }
    }
    
    QQuaternion quat = QQuaternion::fromRotationMatrix(rotation_matrix);
    
    // 设置方向
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.scalar();
    
    return pose;
}

std::vector<double> KinematicsUtils::localInverseKinematics(const geometry_msgs::Pose& target_pose, 
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
    QVector3D target_position(
        target_pose.position.x * 100.0, 
        target_pose.position.y * 100.0, 
        target_pose.position.z * 100.0
    );
    
    // 目标姿态
    QQuaternion target_orientation(
        target_pose.orientation.w, 
        target_pose.orientation.x, 
        target_pose.orientation.y, 
        target_pose.orientation.z
    );
    
    // 迭代优化
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 计算当前关节角度下的末端位姿
        geometry_msgs::Pose current_pose = localForwardKinematics(current_joints);
        
        // 计算位置误差（厘米）
        QVector3D current_position(
            current_pose.position.x * 100.0, 
            current_pose.position.y * 100.0, 
            current_pose.position.z * 100.0
        );
        
        QVector3D position_error = target_position - current_position;
        
        // 计算姿态误差（简化，仅使用位置误差）
        
        // 检查是否收敛
        double error_magnitude = position_error.length();
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
            geometry_msgs::Pose perturbed_pose = localForwardKinematics(current_joints);
            
            // 恢复关节值
            current_joints[j] = original_value;
            
            // 计算偏导数
            jacobian[0][j] = (perturbed_pose.position.x * 100.0 - current_position.x()) / delta;
            jacobian[1][j] = (perturbed_pose.position.y * 100.0 - current_position.y()) / delta;
            jacobian[2][j] = (perturbed_pose.position.z * 100.0 - current_position.z()) / delta;
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

} // namespace arm_trajectory 