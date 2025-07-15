#ifndef FEETECH_ADAPTER_HPP
#define FEETECH_ADAPTER_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>

// 包含Feetech SDK头文件
#include "servo/feetech_sdk/port_handler.h"
#include "servo/feetech_sdk/packet_handler.h"
#include "servo/feetech_sdk/group_sync_read.h"
#include "servo/feetech_sdk/group_sync_write.h"

namespace servo_control {

// FT舵机类型枚举
enum class FTServoType {
  STS_TYPE1,  // STS 类型1舵机
  STS_TYPE2   // STS 类型2舵机
};

// FT舵机控制表地址
namespace ft_address {
  constexpr uint8_t TORQUE_ENABLE     = 40;
  constexpr uint8_t GOAL_ACC          = 41;
  constexpr uint8_t GOAL_POSITION     = 42;
  constexpr uint8_t GOAL_SPEED        = 46;
  constexpr uint8_t PRESENT_POSITION  = 56;
}

// FT舵机配置结构
struct FTServoConfig {
  uint8_t id;
  std::string name;
  FTServoType type;
  uint16_t min_position;
  uint16_t max_position;
};

class FeeTechAdapter
{
public:
  FeeTechAdapter(
    const std::string& port = "/dev/ttyUSB1", 
    int baud_rate = 1000000);
  
  virtual ~FeeTechAdapter();

  // 初始化适配器
  bool init();
  
  // 关闭连接
  void close();
  
  // 加载舵机配置
  void loadServoConfigs(const std::vector<FTServoConfig>& configs);
  
  // 设置舵机位置
  bool setPosition(uint8_t id, uint16_t position, uint16_t time = 0, uint16_t speed = 0);
  
  // 设置舵机速度
  bool setSpeed(uint8_t id, int16_t speed);
  
  // 获取舵机位置
  int getPosition(uint8_t id);
  
  // 获取舵机温度
  int getTemperature(uint8_t id);
  
  // 获取舵机电压
  float getVoltage(uint8_t id);
  
  // 使能/失能舵机扭矩
  bool setTorque(uint8_t id, bool enable);
  
  // 检查连接状态
  bool isConnected() const;
  
  // 同步读取多个舵机的位置
  std::map<uint8_t, int> syncReadPositions(const std::vector<uint8_t>& ids);
  
  // 同步写入多个舵机的位置
  bool syncWritePositions(const std::map<uint8_t, uint16_t>& positions);

private:
  // 端口处理器
  std::shared_ptr<PortHandler> port_handler_;
  
  // 数据包处理器
  std::shared_ptr<PacketHandler> packet_handler_;
  
  // 同步读取处理器
  std::shared_ptr<GroupSyncRead> sync_read_;
  
  // 同步写入处理器
  std::shared_ptr<GroupSyncWrite> sync_write_;
  
  // 舵机配置映射
  std::map<uint8_t, FTServoConfig> servo_configs_;
  
  // 串口参数
  std::string port_;
  int baud_rate_;
  
  // 连接状态
  bool connected_;
  
  // 协议结束标志 (STS/SMS=0, SCS=1)
  int protocol_end_;
  
  // 根据舵机类型调整参数
  void adjustParamsByType(uint8_t id, uint16_t& position, uint16_t& time, uint16_t& speed);
  
  // 检查位置是否在舵机限制范围内
  uint16_t clampPosition(uint8_t id, uint16_t position);
};

} // namespace servo_control

#endif // FEETECH_ADAPTER_HPP 