#include <servo_wrist/vacuum_control.h>
#include <algorithm>

namespace servo_wrist {

VacuumControl::VacuumControl() : is_active_(false), suction_percentage_(0) {
}

VacuumControl::~VacuumControl() {
  // 确保在析构时关闭真空
  deactivate();
}

bool VacuumControl::activate() {
  // 设置真空激活状态
  is_active_ = true;
  
  // 如果吸力设置为0，则设置为默认值50%
  if (suction_percentage_ <= 0) {
    suction_percentage_ = 50;
  }
  
  return true;
}

bool VacuumControl::deactivate() {
  // 设置真空关闭状态
  is_active_ = false;
  return true;
}

bool VacuumControl::isActive() const {
  return is_active_;
}

int VacuumControl::getSuctionPercentage() const {
  return suction_percentage_;
}

bool VacuumControl::setSuctionPercentage(int percentage) {
  // 限制范围在0-100
  suction_percentage_ = std::max(0, std::min(100, percentage));
  return true;
}

} // namespace servo_wrist 