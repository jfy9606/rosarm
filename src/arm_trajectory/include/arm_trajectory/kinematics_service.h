#ifndef ARM_TRAJECTORY_KINEMATICS_SERVICE_H
#define ARM_TRAJECTORY_KINEMATICS_SERVICE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <utility>

#include "arm_trajectory/ForwardKinematics.h"
#include "arm_trajectory/InverseKinematics.h"

namespace arm_trajectory {

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

class KinematicsService {
public:
    KinematicsService(ros::NodeHandle& nh);
    ~KinematicsService();

    geometry_msgs::Pose forwardKinematics(const std::vector<double>& joint_values);
    std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose, const std::vector<double>& initial_guess);
    
    bool handleForwardKinematics(arm_trajectory::ForwardKinematics::Request& req,
                                arm_trajectory::ForwardKinematics::Response& res);
    bool handleInverseKinematics(arm_trajectory::InverseKinematics::Request& req,
                                arm_trajectory::InverseKinematics::Response& res);

private:
    ros::NodeHandle nh_;
    ros::ServiceServer fk_service_;
    ros::ServiceServer ik_service_;
    
    std::vector<DHParameters> dh_params_;
    std::vector<std::pair<double, double>> joint_limits_;  // min, max
    
    Eigen::Matrix4d computeDHTransform(double theta, double d, double a, double alpha);
    bool checkJointLimits(const std::vector<double>& joint_values);
    void setupDHParameters(const std::string& arm_id);
    void setupJointLimits(const std::string& arm_id);
};

} // namespace arm_trajectory

#endif // ARM_TRAJECTORY_KINEMATICS_SERVICE_H
