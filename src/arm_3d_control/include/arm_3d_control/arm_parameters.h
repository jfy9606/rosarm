#ifndef ARM_3D_CONTROL_PARAMETERS_H
#define ARM_3D_CONTROL_PARAMETERS_H

namespace arm_3d_control {

// 机械臂参数（单位：mm）
struct ArmParameters {
    // 机械臂尺寸参数
    static constexpr double LINK1_LENGTH = 100.0; // 底座到第一个关节的距离
    static constexpr double LINK2_LENGTH = 120.0; // 第一个关节到第二个关节的距离
    static constexpr double LINK3_LENGTH = 140.0; // 第二个关节到末端执行器的距离

    // 舵机ID定义
    static constexpr int BASE_SERVO_ID = 1;       // 底座旋转舵机ID
    static constexpr int SHOULDER_SERVO_ID = 2;   // 肩部舵机ID 
    static constexpr int ELBOW_SERVO_ID = 3;      // 肘部舵机ID

    // 舵机参数
    static constexpr int MAX_POSITION = 4095;     // 舵机最大位置值
    static constexpr int CENTER_POSITION = 2047;  // 舵机中心位置值
    static constexpr int DEFAULT_VELOCITY = 500;  // 默认速度
    static constexpr int DEFAULT_ACCELERATION = 30; // 默认加速度
};

} // namespace arm_3d_control

#endif // ARM_3D_CONTROL_PARAMETERS_H 