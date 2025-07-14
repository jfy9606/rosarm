#include "motor/motor_node.hpp"

namespace motor_control {

MotorNode::MotorNode(const rclcpp::NodeOptions & options)
: Node("motor_node", options)
{
  // 声明并获取参数
  this->declare_parameter<std::string>("device_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 115200);
  this->declare_parameter<int>("timeout", 1000);
  
  device_port_ = this->get_parameter("device_port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  timeout_ = this->get_parameter("timeout").as_int();
  
  RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d", device_port_.c_str(), baudrate_);
  
  // 创建订阅者
  order_sub_ = this->create_subscription<motor::msg::MotorOrder>(
    "motor_order", 10, 
    std::bind(&MotorNode::motorOrderCallback, this, std::placeholders::_1));
  
  // 创建发布者
  status_pub_ = this->create_publisher<std_msgs::msg::String>("motor_status", 10);
  
  // 创建定时器，定期发布状态
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&MotorNode::timerCallback, this));
  
  // 设置参数回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MotorNode::parametersCallback, this, std::placeholders::_1));
  
  // 初始化电机控制
  if (!initMotorControl()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize motor control");
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
  
  RCLCPP_INFO(this->get_logger(), "Received motor order with %zu commands", msg->station_num.size());
  
  if (motor_control_->executeOrder(msg)) {
    RCLCPP_INFO(this->get_logger(), "Motor order executed successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute motor order");
  }
}

void MotorNode::timerCallback()
{
  if (!motor_control_ || !motor_control_->isConnected()) {
    return;
  }
  
  // 获取电机1的状态作为示例
  std::string status = motor_control_->getStatus(1);
  
  if (!status.empty()) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = status;
    status_pub_->publish(std::move(msg));
  }
}

rcl_interfaces::msg::SetParametersResult MotorNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  // 处理参数更新
  for (const auto & param : parameters) {
    if (param.get_name() == "device_port") {
      device_port_ = param.as_string();
      RCLCPP_INFO(this->get_logger(), "Updated device port: %s", device_port_.c_str());
      
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
    RCLCPP_ERROR(this->get_logger(), "Exception during motor control initialization: %s", e.what());
    return false;
  }
}

} // namespace motor_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(motor_control::MotorNode) 