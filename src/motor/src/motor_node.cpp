#include "motor/motor_node.hpp"
#include <motor/msg/motor_order.hpp>
#include <motor/srv/motor_control.hpp>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace motor_control {

MotorNode::MotorNode(const rclcpp::NodeOptions & options)
: Node("motor_node", options)
{
  // 声明并获取参数
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 115200);
  this->declare_parameter<int>("timeout", 1000);
  this->declare_parameter<std::string>("config_file", "");
  
  device_port_ = this->get_parameter("port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  timeout_ = this->get_parameter("timeout").as_int();
  std::string config_file = this->get_parameter("config_file").as_string();
  
  RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d", device_port_.c_str(), baudrate_);
  
  // 创建发布器
  motor_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "motor/status", 10);
  
  // 创建订阅器
  motor_order_sub_ = this->create_subscription<motor::msg::MotorOrder>(
    "motor/order", 10,
    std::bind(&MotorNode::motorOrderCallback, this, std::placeholders::_1));
  
  // 创建服务
  motor_control_srv_ = this->create_service<motor::srv::MotorControl>(
    "motor/control",
    std::bind(&MotorNode::motorControlCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 创建定时器，定期发布电机状态
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MotorNode::timerCallback, this));
  
  // 设置参数回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MotorNode::parametersCallback, this, std::placeholders::_1));
  
  // 初始化电机控制
  if (!initMotorControl()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize motor control");
  }
  
  // 加载配置文件
  if (config_file.empty()) {
    // 尝试使用默认配置文件路径
    std::string package_path = ament_index_cpp::get_package_share_directory("motor");
    config_file = package_path + "/config/motor_config.yaml";
    RCLCPP_INFO(this->get_logger(), "Using default config file path: %s", config_file.c_str());
  }
  
  if (std::filesystem::exists(config_file)) {
    if (motor_control_->loadMotorConfigs(config_file)) {
      RCLCPP_INFO(this->get_logger(), "Loaded motor configurations from %s", config_file.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load motor configurations from %s", config_file.c_str());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Config file %s does not exist", config_file.c_str());
  }
}

MotorNode::~MotorNode()
{
  // 确保在销毁节点时关闭电机控制
  if (motor_control_) {
    motor_control_->close();
  }
}

void MotorNode::motorOrderCallback(const motor::msg::MotorOrder::SharedPtr msg)
{
  if (!motor_control_ || !motor_control_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Motor control not connected");
    return;
  }
  
  // 执行电机指令
  if (motor_control_->executeOrder(msg)) {
    RCLCPP_INFO(this->get_logger(), "Motor order executed successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute motor order");
  }
}

void MotorNode::motorControlCallback(
  const std::shared_ptr<motor::srv::MotorControl::Request> request,
  std::shared_ptr<motor::srv::MotorControl::Response> response)
{
  if (!motor_control_ || !motor_control_->isConnected()) {
    RCLCPP_ERROR(this->get_logger(), "Motor control not connected");
    response->success = false;
    response->message = "Motor control not connected";
    return;
  }
  
  bool result = false;
  
  // 根据控制类型执行不同的命令
  if (request->control_type == "position") {
    result = motor_control_->setPosition(
      request->station_num, request->position, request->threshold, 
      std::abs(request->velocity), request->acc);
  } else if (request->control_type == "velocity") {
    result = motor_control_->setVelocity(
      request->station_num, request->velocity, request->acc, request->dec);
  } else if (request->control_type == "enable") {
    result = motor_control_->enableMotor(
      request->station_num, request->enable);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown control type: %s", request->control_type.c_str());
    response->success = false;
    response->message = "Unknown control type";
    return;
  }
  
  response->success = result;
  response->message = result ? "Control command executed successfully" : "Failed to execute control command";
}

void MotorNode::timerCallback()
{
  if (!motor_control_ || !motor_control_->isConnected()) {
    return;
  }
  
  // 查询电机状态
  std::string status;
  for (int i = 1; i <= 4; ++i) {  // 最多支持4个电机站
    std::string motor_status = motor_control_->getStatus(i);
    if (!motor_status.empty()) {
      status += "Motor " + std::to_string(i) + ": " + motor_status + "\n";
    }
  }
  
  // 发布状态
  if (!status.empty()) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = status;
    motor_status_pub_->publish(std::move(msg));
  }
}

rcl_interfaces::msg::SetParametersResult MotorNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  // 处理参数更新
  for (const auto & param : parameters) {
    if (param.get_name() == "port") {
      device_port_ = param.as_string();
      RCLCPP_INFO(this->get_logger(), "Updated port: %s", device_port_.c_str());
      
      // 重新初始化电机控制
      if (motor_control_) {
        motor_control_->close();
      }
      initMotorControl();
    } else if (param.get_name() == "baudrate") {
      baudrate_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated baudrate: %d", baudrate_);
      
      // 重新初始化电机控制
      if (motor_control_) {
        motor_control_->close();
      }
      initMotorControl();
    } else if (param.get_name() == "timeout") {
      timeout_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated timeout: %d", timeout_);
    } else if (param.get_name() == "config_file") {
      std::string config_file = param.as_string();
      if (!config_file.empty() && motor_control_) {
        if (motor_control_->loadMotorConfigs(config_file)) {
          RCLCPP_INFO(this->get_logger(), "Loaded motor configurations from %s", config_file.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to load motor configurations from %s", config_file.c_str());
          result.successful = false;
          result.reason = "Failed to load motor configurations";
        }
      }
    }
  }
  
  return result;
}

bool MotorNode::initMotorControl()
{
  try {
    // 创建并初始化电机控制对象
    motor_control_ = std::make_unique<MotorControl>(device_port_, baudrate_, timeout_);
    
    if (!motor_control_->init()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize motor control");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Motor control initialized successfully");
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), 
                "Exception during motor control initialization: %s", 
                e.what());
    return false;
  }
}

} // namespace motor_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motor_control::MotorNode) 