#ifndef SERVO_VACUUM_CONTROL_H
#define SERVO_VACUUM_CONTROL_H

#include <string>
#include <vector>

namespace servo {

/**
 * @brief 真空控制类，包含真空控制的核心逻辑，与ROS无关
 */
class VacuumControl {
public:
  /**
   * @brief 构造函数
   */
  VacuumControl();
  
  /**
   * @brief 析构函数
   */
  ~VacuumControl();

  /**
   * @brief 激活真空吸盘
   * @return 操作是否成功
   */
  bool activate();
  
  /**
   * @brief 关闭真空吸盘
   * @return 操作是否成功
   */
  bool deactivate();
  
  /**
   * @brief 获取当前真空吸盘状态
   * @return 真空吸盘是否激活
   */
  bool isActive() const;
  
  /**
   * @brief 获取当前的吸力百分比
   * @return 吸力百分比(0-100)
   */
  int getSuctionPercentage() const;
  
  /**
   * @brief 设置吸力百分比
   * @param percentage 吸力百分比(0-100)
   * @return 操作是否成功
   */
  bool setSuctionPercentage(int percentage);

private:
  bool is_active_;
  int suction_percentage_;
};

} // namespace servo

#endif // SERVO_VACUUM_CONTROL_H 