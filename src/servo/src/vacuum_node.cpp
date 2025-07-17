#include "servo/vacuum_node.hpp"

namespace servo_control {

VacuumNode::VacuumNode(const rclcpp::NodeOptions & options)
: Node("vacuum_node", options),
  vacuum_enabled_(false),
  vacuum_power_(0),
  vacuum_id_(10) // 假设真空泵舵机ID为10
{
  // 声明并获取参数
  this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
  this->declare_parameter<int>("baudrate", 1000000);
  this->declare_parameter<int>("timeout", 100);
  this->declare_parameter<int>("vacuum_id", 10);
  
  device_port_ = this->get_parameter("port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  timeout_ = this->get_parameter("timeout").as_int();
  vacuum_id_ = this->get_parameter("vacuum_id").as_int();
  
  RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d, Vacuum ID: %d", 
             device_port_.c_str(), baudrate_, vacuum_id_);
  
  // 创建服务
  vacuum_cmd_srv_ = this->create_service<servo_interfaces::srv::VacuumCmd>(
    "vacuum/cmd",
    std::bind(&VacuumNode::vacuumCmdCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 创建发布器
  vacuum_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "vacuum/state", 10);
  vacuum_power_pub_ = this->create_publisher<std_msgs::msg::Int16>(
    "vacuum/power", 10);
  
  // 创建订阅器
  vacuum_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "vacuum/enable", 10,
    std::bind(&VacuumNode::vacuumEnableCallback, this, std::placeholders::_1));
  vacuum_power_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    "vacuum/set_power", 10,
    std::bind(&VacuumNode::vacuumPowerCallback, this, std::placeholders::_1));
  
  // 创建定时器，定期发布真空泵状态
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&VacuumNode::timerCallback, this));
  
  // 设置参数回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&VacuumNode::parametersCallback, this, std::placeholders::_1));
  
  // 初始化舵机控制
  if (!initServoControl()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize servo control");
  } else {
    // 初始化真空泵状态为关闭
    setVacuumState(false, 0);
  }
}

VacuumNode::~VacuumNode()
{
  // 确保在销毁节点时关闭真空泵
  if (servo_control_ && servo_control_->isConnected()) {
    setVacuumState(false, 0);
    servo_control_->close();
  }
}

void VacuumNode::vacuumCmdCallback(
  const std::shared_ptr<servo_interfaces::srv::VacuumCmd::Request> request,
  std::shared_ptr<servo_interfaces::srv::VacuumCmd::Response> response)
{
  if (!servo_control_ || !servo_control_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Servo control not connected");
    response->success = false;
    return;
  }
  
  // 设置真空吸盘状态，只使用enable状态，power保持当前值
  response->success = setVacuumState(request->enable, vacuum_power_);
  response->message = response->success ? "Vacuum state set successfully" : "Failed to set vacuum state";
}

void VacuumNode::vacuumEnableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!servo_control_ || !servo_control_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Servo control not connected");
    return;
  }
  
  // 设置真空吸盘启用状态
  setVacuumState(msg->data, vacuum_power_);
}

void VacuumNode::vacuumPowerCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  if (!servo_control_ || !servo_control_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Servo control not connected");
    return;
  }
  
  // 设置真空吸盘功率
  int power = std::max(0, std::min(100, static_cast<int>(msg->data)));
  setVacuumState(vacuum_enabled_, power);
}

void VacuumNode::timerCallback()
{
  // 发布真空泵状态
  auto state_msg = std::make_unique<std_msgs::msg::Bool>();
  state_msg->data = vacuum_enabled_;
  vacuum_state_pub_->publish(std::move(state_msg));
  
  auto power_msg = std::make_unique<std_msgs::msg::Int16>();
  power_msg->data = vacuum_power_;
  vacuum_power_pub_->publish(std::move(power_msg));
}

rcl_interfaces::msg::SetParametersResult VacuumNode::parametersCallback(
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
    } else if (param.get_name() == "vacuum_id") {
      vacuum_id_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated vacuum ID: %d", vacuum_id_);
    }
  }
  
  return result;
}

bool VacuumNode::initServoControl()
{
  try {
    // 创建并初始化舵机控制对象
    servo_control_ = std::make_unique<ServoControl>(device_port_, baudrate_, timeout_);
    
    if (!servo_control_->init()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize servo control");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Servo control initialized successfully");
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), 
                "Exception during servo control initialization: %s", 
                e.what());
    return false;
  }
}

bool VacuumNode::setVacuumState(bool enable, int power)
{
  if (!servo_control_ || !servo_control_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Servo control not connected");
    return false;
  }
  
  // 限制功率范围为0-100
  power = std::max(0, std::min(100, power));
  
  // 将功率转换为舵机值
  // 假设0对应舵机位置0，100对应舵机位置1000
  uint16_t position = enable ? static_cast<uint16_t>(power * 10) : 0;
  
  // 设置舵机位置
  bool result = servo_control_->setPosition(vacuum_id_, position);
  
  if (result) {
    // 更新状态
    vacuum_enabled_ = enable;
    vacuum_power_ = power;
    
    RCLCPP_INFO(this->get_logger(), 
               "Set vacuum state: %s with power: %d", 
               enable ? "ON" : "OFF", power);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set vacuum state");
  }
  
  return result;
}

} // namespace servo_control 