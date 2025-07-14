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
  
  device_port_ = this->get_parameter("port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  timeout_ = this->get_parameter("timeout").as_int();
  
  RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d", device_port_.c_str(), baudrate_);
  
  // 初始化关节ID映射
  joint_id_map_["wrist_pitch"] = 1;  // 腕部俯仰关节，ID为1
  joint_id_map_["wrist_roll"] = 2;   // 腕部滚转关节，ID为2
  
  // 加载关节限制
  loadJointLimits();
  
  // 初始化关节位置
  for (const auto& joint : joint_id_map_) {
    joint_positions_[joint.first] = 0.0;
  }
  
  // 创建服务
  joint_control_srv_ = this->create_service<servo::srv::JointControl>(
    "wrist/joint_control",
    std::bind(&WristNode::jointControlCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 创建发布器
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "wrist/joint_states", 10);
  
  // 创建订阅器
  servo_control_sub_ = this->create_subscription<servo::msg::SerControl>(
    "wrist/control", 10,
    std::bind(&WristNode::servoControlCallback, this, std::placeholders::_1));
  
  // 创建定时器，定期发布关节状态
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
}

WristNode::~WristNode()
{
  // 确保在销毁节点时关闭舵机控制
  if (servo_control_) {
    servo_control_->close();
  }
}

void WristNode::jointControlCallback(
  const std::shared_ptr<servo::srv::JointControl::Request> request,
  std::shared_ptr<servo::srv::JointControl::Response> response)
{
  if (!servo_control_ || !servo_control_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Servo control not connected");
    response->success = false;
    return;
  }
  
  // 检查关节名称是否有效
  if (joint_id_map_.find(request->joint_name) == joint_id_map_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Unknown joint name: %s", request->joint_name.c_str());
    response->success = false;
    return;
  }
  
  uint8_t id = joint_id_map_[request->joint_name];
  
  // 检查角度是否在限制范围内
  if (joint_limits_.count(request->joint_name) > 0) {
    auto limits = joint_limits_[request->joint_name];
    if (request->position < limits.first || request->position > limits.second) {
      RCLCPP_ERROR(this->get_logger(), 
                  "Position %f is out of limits [%f, %f] for joint %s",
                  request->position, limits.first, limits.second,
                  request->joint_name.c_str());
      response->success = false;
      return;
    }
  }
  
  // 将角度转换为舵机值
  uint16_t position = angleToServoValue(request->position, request->joint_name);
  
  // 设置舵机位置
  bool result = servo_control_->setPosition(id, position, request->time, request->speed);
  
  if (result) {
    // 更新关节状态
    joint_positions_[request->joint_name] = request->position;
    RCLCPP_INFO(this->get_logger(), 
               "Set joint %s to position %f (servo value: %d)",
               request->joint_name.c_str(), request->position, position);
  } else {
    RCLCPP_ERROR(this->get_logger(), 
                "Failed to set joint %s to position %f",
                request->joint_name.c_str(), request->position);
  }
  
  response->success = result;
}

void WristNode::servoControlCallback(const servo::msg::SerControl::SharedPtr msg)
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
  
  // 直接设置舵机位置
  bool result = servo_control_->setPosition(msg->id, msg->position, msg->time, msg->speed);
  
  if (result) {
    // 尝试更新已知关节的位置
    for (const auto& joint : joint_id_map_) {
      if (joint.second == msg->id) {
        joint_positions_[joint.first] = servoValueToAngle(msg->position, joint.first);
        break;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), 
               "Set servo ID %d to position %d with time %d and speed %d",
               msg->id, msg->position, msg->time, msg->speed);
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
  
  // 创建关节状态消息
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state_msg->header.stamp = this->now();
  
  // 读取所有关节的当前位置
  for (const auto& joint : joint_id_map_) {
    int position = servo_control_->getPosition(joint.second);
    if (position >= 0) {
      // 将舵机位置转换为角度
      double angle = servoValueToAngle(position, joint.first);
      joint_positions_[joint.first] = angle;
    }
    
    // 添加到关节状态消息
    joint_state_msg->name.push_back(joint.first);
    joint_state_msg->position.push_back(joint_positions_[joint.first]);
    joint_state_msg->velocity.push_back(0.0);  // 暂不提供速度信息
    joint_state_msg->effort.push_back(0.0);    // 暂不提供力矩信息
  }
  
  // 发布关节状态
  joint_state_pub_->publish(std::move(joint_state_msg));
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
  
  // 从参数服务器加载关节限制
  // 这里可以添加从参数中加载限制的代码
}

} // namespace servo_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(servo_control::WristNode) 