#include "servo/wrist_node.hpp"
#include <cmath>

namespace servo_control {

WristNode::WristNode(const rclcpp::NodeOptions & options)
: Node("wrist_node", options)
{
  // 声明并获取参数
  this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
  this->declare_parameter<int>("baudrate", 1000000);
  this->declare_parameter<int>("timeout", 100);
  this->declare_parameter<std::string>("config_file", "");
  
  device_port_ = this->get_parameter("port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  timeout_ = this->get_parameter("timeout").as_int();
  std::string config_file = this->get_parameter("config_file").as_string();
  
  RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d", device_port_.c_str(), baudrate_);
  
  // 初始化关节ID映射
  joint_id_map_["wrist_pitch"] = 1;  // 腕部俯仰关节，ID为1
  joint_id_map_["wrist_roll"] = 2;   // 腕部滚转关节，ID为2
  joint_id_map_["wrist_yaw"] = 3;    // 腕部偏航关节，ID为3
  joint_id_map_["gripper"] = 4;      // 夹爪关节，ID为4
  joint_id_map_["elbow_pitch"] = 5;  // 肘部俯仰关节，ID为5
  joint_id_map_["elbow_roll"] = 6;   // 肘部滚转关节，ID为6
  
  // 加载关节限制
  loadJointLimits();
  
  // 初始化关节位置
  for (const auto& joint : joint_id_map_) {
    joint_positions_[joint.first] = 0.0;
  }
  
  // 初始化关节状态结构体
  current_joint_state_ = std::make_shared<JointState>();
  
  // 创建服务
  joint_control_srv_ = this->create_service<servo_interfaces::srv::JointControl>(
    "wrist/joint_control",
    std::bind(&WristNode::jointControlCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 创建订阅器
  servo_control_sub_ = this->create_subscription<servo_interfaces::msg::SerControl>(
    "wrist/control", 10,
    std::bind(&WristNode::servoControlCallback, this, std::placeholders::_1));
  
  // 创建定时器，定期更新关节状态
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&WristNode::timerCallback, this));
  
  // 设置参数回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&WristNode::parametersCallback, this, std::placeholders::_1));
  
  // 初始化舵机控制
  if (!initServoControl()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize servo control");
  }
  
  // 加载配置文件
  if (!config_file.empty()) {
    if (servo_control_->loadServoConfigs(config_file)) {
      RCLCPP_INFO(this->get_logger(), "Loaded servo configurations from %s", config_file.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load servo configurations from %s", config_file.c_str());
    }
  }
}

WristNode::~WristNode()
{
  // 确保在销毁节点时关闭舵机控制
  if (servo_control_) {
    servo_control_->close();
  }
}

void WristNode::jointControlCallback(
  const std::shared_ptr<servo_interfaces::srv::JointControl::Request> request,
  std::shared_ptr<servo_interfaces::srv::JointControl::Response> response)
{
  if (!servo_control_ || !servo_control_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Servo control not connected");
    response->success = false;
    response->message = "Servo control not connected";
    return;
  }
  
  // 根据消息定义, JointControl.srv 包含 float64[] position
  // 假设数组下标对应于预定义的关节顺序
  std::vector<std::string> joint_names;
  for (const auto& pair : joint_id_map_) {
    joint_names.push_back(pair.first);
  }
  
  bool all_success = true;
  std::stringstream result_messages;
  
  // 确保提供了足够的位置数据
  if (request->position.size() < joint_names.size()) {
    response->success = false;
    response->message = "Not enough position values provided";
    return;
  }
  
  // 准备同步写入的位置映射
  std::map<uint8_t, uint16_t> positions;
  
  // 为每个关节设置位置
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const std::string& joint_name = joint_names[i];
    double position_value = request->position[i];
    uint8_t id = joint_id_map_[joint_name];
  
  // 检查角度是否在限制范围内
    if (joint_limits_.count(joint_name) > 0) {
      auto limits = joint_limits_[joint_name];
      if (position_value < limits.first || position_value > limits.second) {
      RCLCPP_ERROR(this->get_logger(), 
                  "Position %f is out of limits [%f, %f] for joint %s",
                    position_value, limits.first, limits.second, joint_name.c_str());
        all_success = false;
        result_messages << "Joint " << joint_name << " position out of limits. ";
        continue;
    }
  }
  
  // 将角度转换为舵机值
    uint16_t servo_value = angleToServoValue(position_value, joint_name);
  
    // 添加到位置映射
    positions[id] = servo_value;
  
    // 更新关节状态
    joint_positions_[joint_name] = position_value;
    RCLCPP_INFO(this->get_logger(), 
               "Set joint %s to position %f (servo value: %d)",
               joint_name.c_str(), position_value, servo_value);
  }
  
  // 同步写入所有舵机位置
  if (!positions.empty()) {
    bool result = servo_control_->syncWritePositions(positions);
    if (!result) {
      RCLCPP_ERROR(this->get_logger(), "Failed to sync write positions");
      all_success = false;
      result_messages << "Failed to sync write positions. ";
    }
  }
  
  response->success = all_success;
  response->message = all_success ? "All joints set successfully" : result_messages.str();
}

void WristNode::servoControlCallback(const servo_interfaces::msg::SerControl::SharedPtr msg)
{
  if (!servo_control_ || !servo_control_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Servo control not connected");
    return;
  }
  
  // 检查关节ID是否在范围内
  if (msg->id < 1 || msg->id > 254) {
    RCLCPP_ERROR(this->get_logger(), "Invalid servo ID: %d", msg->id);
    return;
  }
  
  // 直接设置舵机位置，使用消息中的position, time, speed
  bool result = servo_control_->setPosition(
      static_cast<uint8_t>(msg->id), 
      static_cast<uint16_t>(msg->position),
      static_cast<uint16_t>(msg->time),  // 使用time参数
      static_cast<uint16_t>(msg->speed)  // 使用speed参数
  );
  
  if (result) {
    // 尝试更新已知关节的位置
    for (const auto& joint : joint_id_map_) {
      if (joint.second == static_cast<uint8_t>(msg->id)) {
        joint_positions_[joint.first] = servoValueToAngle(
            static_cast<uint16_t>(msg->position), joint.first);
        break;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), 
               "Set servo ID %d to position %d with speed %d",
               msg->id, msg->position, msg->speed);
  } else {
    RCLCPP_ERROR(this->get_logger(), 
                "Failed to set servo ID %d to position %d",
                msg->id, msg->position);
  }
}

void WristNode::timerCallback()
{
  if (!servo_control_ || !servo_control_->isConnected()) {
    return;
  }
  
  // 更新关节状态结构体
  current_joint_state_->name.clear();
  current_joint_state_->position.clear();
  current_joint_state_->velocity.clear();
  current_joint_state_->effort.clear();
  
  // 准备要读取的舵机ID列表
  std::vector<uint8_t> ids;
  for (const auto& joint : joint_id_map_) {
    ids.push_back(joint.second);
  }
  
  // 同步读取所有舵机位置
  auto positions = servo_control_->syncReadPositions(ids);
  
  // 更新关节状态
  for (const auto& joint : joint_id_map_) {
    const std::string& joint_name = joint.first;
    uint8_t id = joint.second;
    
    // 如果成功读取到位置，更新关节状态
    if (positions.count(id) > 0) {
      double angle = servoValueToAngle(positions[id], joint_name);
      joint_positions_[joint_name] = angle;
    }
    
    // 添加到关节状态结构体
    current_joint_state_->name.push_back(joint_name);
    current_joint_state_->position.push_back(joint_positions_[joint_name]);
    current_joint_state_->velocity.push_back(0.0);  // 暂不提供速度信息
    current_joint_state_->effort.push_back(0.0);    // 暂不提供力矩信息
  }
  
  // 关节状态已更新，如需，可以通过API直接获取current_joint_state_
}

rcl_interfaces::msg::SetParametersResult WristNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  // 处理参数更新
  for (const auto & param : parameters) {
    if (param.get_name() == "port") {
      device_port_ = param.as_string();
      RCLCPP_INFO(this->get_logger(), "Updated port: %s", device_port_.c_str());
      
      // 重新初始化舵机控制
      if (servo_control_) {
        servo_control_->close();
      }
      initServoControl();
    } else if (param.get_name() == "baudrate") {
      baudrate_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated baudrate: %d", baudrate_);
      
      // 重新初始化舵机控制
      if (servo_control_) {
        servo_control_->close();
      }
      initServoControl();
    } else if (param.get_name() == "timeout") {
      timeout_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated timeout: %d", timeout_);
    } else if (param.get_name() == "config_file") {
      std::string config_file = param.as_string();
      if (!config_file.empty() && servo_control_) {
        if (servo_control_->loadServoConfigs(config_file)) {
          RCLCPP_INFO(this->get_logger(), "Loaded servo configurations from %s", config_file.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to load servo configurations from %s", config_file.c_str());
          result.successful = false;
          result.reason = "Failed to load servo configurations";
        }
      }
    }
  }
  
  return result;
}

bool WristNode::initServoControl()
{
  try {
    // 创建并初始化舵机控制对象
    servo_control_ = std::make_unique<ServoControl>(device_port_, baudrate_, timeout_);
    
    if (!servo_control_->init()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize servo control");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Servo control initialized successfully");
    
    // 为所有舵机打开扭矩
    for (const auto& joint : joint_id_map_) {
      if (!servo_control_->setTorque(joint.second, true)) {
        RCLCPP_WARN(this->get_logger(), 
                   "Failed to enable torque for joint %s (ID: %d)",
                   joint.first.c_str(), joint.second);
      }
    }
    
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), 
                "Exception during servo control initialization: %s", 
                e.what());
    return false;
  }
}

uint16_t WristNode::angleToServoValue(double angle, const std::string& joint_name)
{
  // 默认映射: 角度 [-π,π] 映射到舵机值 [0,4095]
  // 即: 舵机值 = (angle + π) * 4096 / (2π)
  // 注意：不同的舵机可能有不同的映射关系
  double normalized_angle = angle;
  uint16_t center = 2048;  // 默认中心位置
  
  // 对不同关节应用不同的映射关系
  if (joint_name == "wrist_pitch") {
    // 假设腕部俯仰关节的中心位置对应0度
    return static_cast<uint16_t>(center + angle * 1024.0 / M_PI);
  } else if (joint_name == "wrist_roll") {
    // 假设腕部滚转关节的中心位置对应0度
    return static_cast<uint16_t>(center + angle * 1024.0 / M_PI);
  } else if (joint_name == "wrist_yaw") {
    // 假设腕部偏航关节的中心位置对应0度
    return static_cast<uint16_t>(center + angle * 1024.0 / M_PI);
  } else if (joint_name == "gripper") {
    // 夹爪关节映射
    return static_cast<uint16_t>(center + angle * 1024.0 / M_PI);
  } else if (joint_name == "elbow_pitch" || joint_name == "elbow_roll") {
    // STS_TYPE2舵机的映射可能不同
    // 假设范围为[0, 3500]
    return static_cast<uint16_t>(1750 + angle * 875.0 / M_PI);
  } else {
    // 默认映射
    return static_cast<uint16_t>((normalized_angle + M_PI) * 2048.0 / M_PI);
  }
}

double WristNode::servoValueToAngle(uint16_t value, const std::string& joint_name)
{
  // 默认映射: 舵机值 [0,4095] 映射到角度 [-π,π]
  // 即: angle = (舵机值 * 2π / 4096) - π
  uint16_t center = 2048;  // 默认中心位置
  
  // 对不同关节应用不同的映射关系
  if (joint_name == "wrist_pitch") {
    // 假设腕部俯仰关节的中心位置对应0度
    return static_cast<double>(value - center) * M_PI / 1024.0;
  } else if (joint_name == "wrist_roll") {
    // 假设腕部滚转关节的中心位置对应0度
    return static_cast<double>(value - center) * M_PI / 1024.0;
  } else if (joint_name == "wrist_yaw") {
    // 假设腕部偏航关节的中心位置对应0度
    return static_cast<double>(value - center) * M_PI / 1024.0;
  } else if (joint_name == "gripper") {
    // 夹爪关节映射
    return static_cast<double>(value - center) * M_PI / 1024.0;
  } else if (joint_name == "elbow_pitch" || joint_name == "elbow_roll") {
    // STS_TYPE2舵机的映射可能不同
    // 假设范围为[0, 3500]
    return static_cast<double>(value - 1750) * M_PI / 875.0;
  } else {
    // 默认映射
    return static_cast<double>(value) * M_PI / 2048.0 - M_PI;
  }
}

void WristNode::loadJointLimits()
{
  // 设置关节限制（弧度）
  joint_limits_["wrist_pitch"] = std::make_pair(-M_PI/2, M_PI/2);  // ±90度
  joint_limits_["wrist_roll"] = std::make_pair(-M_PI, M_PI);        // ±180度
  joint_limits_["wrist_yaw"] = std::make_pair(-M_PI/2, M_PI/2);     // ±90度
  joint_limits_["gripper"] = std::make_pair(-M_PI/4, M_PI/4);       // ±45度
  joint_limits_["elbow_pitch"] = std::make_pair(-M_PI/2, M_PI/2);   // ±90度
  joint_limits_["elbow_roll"] = std::make_pair(-M_PI/2, M_PI/2);    // ±90度
  
  // 从参数服务器加载关节限制
  // 这里可以添加从参数中加载限制的代码
}

} // namespace servo_control
