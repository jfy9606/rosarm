#ifndef TRAJECTORY_KINEMATICS_UTILS_H
#define TRAJECTORY_KINEMATICS_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <utility>
#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>

namespace trajectory {

/**
 * @brief DH parameter structure for a single joint
 */
struct DHParam {
    double theta;  // joint angle
    double d;      // link offset
    double a;      // link length
    double alpha;  // link twist
    int joint_type; // 0 for revolute, 1 for prismatic

    // Default constructor
    DHParam() : theta(0), d(0), a(0), alpha(0), joint_type(0) {}
    
    // Constructor with parameters
    DHParam(double theta_val, double d_val, double a_val, double alpha_val, int type)
        : theta(theta_val), d(d_val), a(a_val), alpha(alpha_val), joint_type(type) {}
};

/**
 * @brief Utility class for kinematics calculations
 * 
 * This class provides methods for forward and inverse kinematics calculations
 * that can be used by other components like the GUI.
 */
class KinematicsUtils {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     */
    KinematicsUtils(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~KinematicsUtils();
    
    /**
     * @brief Calculate forward kinematics
     * @param joint_values Joint values
     * @return End-effector pose
     */
    geometry_msgs::Pose forwardKinematics(const std::vector<double>& joint_values);
    
    /**
     * @brief Calculate inverse kinematics
     * @param target_pose Target pose
     * @param initial_guess Initial guess for joint values
     * @return Joint values
     */
    std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                        const std::vector<double>& initial_guess);
    
    /**
     * @brief Check if joint values are within limits
     * @param joint_values Joint values to check
     * @return True if all joints are within limits
     */
    bool checkJointLimits(const std::vector<double>& joint_values);
    
    /**
     * @brief Set up DH parameters
     * @param arm_id Arm ID
     */
    void setupDHParameters(const std::string& arm_id = "arm1");
    
    /**
     * @brief Set up joint limits
     * @param arm_id Arm ID
     */
    void setupJointLimits(const std::string& arm_id = "arm1");
    
    /**
     * @brief Compute DH transformation matrix
     * @param theta Joint angle
     * @param d Link offset
     * @param a Link length
     * @param alpha Link twist
     * @return 4x4 transformation matrix
     */
    QMatrix4x4 computeDHTransform(double theta, double d, double a, double alpha);
    
    /**
     * @brief Convert degrees to radians
     * @param deg Angle in degrees
     * @return Angle in radians
     */
    double degToRad(double deg);
    
    /**
     * @brief Convert radians to degrees
     * @param rad Angle in radians
     * @return Angle in degrees
     */
    double radToDeg(double rad);
    
    /**
     * @brief Get the current DH parameters
     * @return Vector of DH parameters
     */
    const std::vector<DHParam>& getDHParameters() const { return dh_params_; }
    
    /**
     * @brief Get the current joint limits
     * @return Vector of joint limits (min, max)
     */
    const std::vector<std::pair<double, double>>& getJointLimits() const { return joint_limits_; }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient fk_client_;
    ros::ServiceClient ik_client_;
    
    std::vector<DHParam> dh_params_;
    std::vector<std::pair<double, double>> joint_limits_;  // min, max
    
    // Local implementation of forward kinematics
    geometry_msgs::Pose localForwardKinematics(const std::vector<double>& joint_values);
    
    // Local implementation of inverse kinematics
    std::vector<double> localInverseKinematics(const geometry_msgs::Pose& target_pose, 
                                             const std::vector<double>& initial_guess);
};

} // namespace trajectory

#endif // TRAJECTORY_KINEMATICS_UTILS_H 