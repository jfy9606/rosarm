#ifndef ARM_3D_CONTROL_INVERSE_KINEMATICS_H
#define ARM_3D_CONTROL_INVERSE_KINEMATICS_H

#include <geometry_msgs/Point.h>
#include <cmath>
#include <ros/ros.h>
#include "arm_3d_control/arm_parameters.h"

namespace arm_3d_control {

class InverseKinematics {
public:
    // 逆运动学计算
    static bool calculateAngles(const geometry_msgs::Point& target, 
                                double& baseAngle, 
                                double& shoulderAngle, 
                                double& elbowAngle) {
        double x = target.x;
        double y = target.y;
        double z = target.z;
        
        // 计算底座旋转角度（绕z轴）
        baseAngle = atan2(y, x) * 180.0 / M_PI;
        
        // 计算在xz平面的投影距离
        double projXY = sqrt(x*x + y*y);
        
        // 计算肩部和肘部角度
        double d = sqrt(projXY*projXY + (z - ArmParameters::LINK1_LENGTH)*(z - ArmParameters::LINK1_LENGTH)); // 从第一个关节到目标点的距离
        
        // 检查目标是否超出机械臂可达范围
        if (d > (ArmParameters::LINK2_LENGTH + ArmParameters::LINK3_LENGTH)) {
            ROS_ERROR("Target position out of reach: distance %f exceeds arm length %f", 
                    d, ArmParameters::LINK2_LENGTH + ArmParameters::LINK3_LENGTH);
            return false;
        }
        
        // 使用余弦定理计算肘部角度
        double cosElbow = (ArmParameters::LINK2_LENGTH*ArmParameters::LINK2_LENGTH + 
                        ArmParameters::LINK3_LENGTH*ArmParameters::LINK3_LENGTH - d*d) / 
                        (2*ArmParameters::LINK2_LENGTH*ArmParameters::LINK3_LENGTH);
        if (cosElbow < -1.0 || cosElbow > 1.0) {
            ROS_ERROR("Inverse kinematics calculation error: invalid cos value %f", cosElbow);
            return false;
        }
        elbowAngle = acos(cosElbow) * 180.0 / M_PI;
        
        // 计算肩部角度
        double gamma = atan2(z - ArmParameters::LINK1_LENGTH, projXY) * 180.0 / M_PI;
        double alpha = acos((ArmParameters::LINK2_LENGTH*ArmParameters::LINK2_LENGTH + d*d - 
                        ArmParameters::LINK3_LENGTH*ArmParameters::LINK3_LENGTH) / 
                        (2*ArmParameters::LINK2_LENGTH*d)) * 180.0 / M_PI;
        shoulderAngle = gamma + alpha;
        
        // 确保角度在合理范围内
        if (abs(baseAngle) > 180.0 || abs(shoulderAngle) > 180.0 || abs(elbowAngle) > 180.0) {
            ROS_ERROR("Joint angle exceeds limit: base=%f, shoulder=%f, elbow=%f", 
                    baseAngle, shoulderAngle, elbowAngle);
            return false;
        }
        
        return true;
    }
    
    // 角度转舵机位置值
    static int angleToPulse(double angle) {
        // 假设舵机的角度范围是[-180, 180]对应[0, 4095]
        return static_cast<int>(((angle + 180.0) / 360.0) * ArmParameters::MAX_POSITION);
    }
};

} // namespace arm_3d_control

#endif // ARM_3D_CONTROL_INVERSE_KINEMATICS_H 