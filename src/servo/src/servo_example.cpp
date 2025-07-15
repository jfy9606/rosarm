#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>
#include "servo/servo_control.hpp"

class ServoExample : public rclcpp::Node
{
public:
  ServoExample() : Node("servo_example")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Servo Example...");
    
    // Create servo control
    servo_control_ = std::make_unique<servo_control::ServoControl>("/dev/ttyUSB0", 1000000);
    
    // Initialize
    if (servo_control_->init()) {
      RCLCPP_INFO(this->get_logger(), "Servo control initialized successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize servo control");
      return;
    }
    
    // Create a timer for controlling servos
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), 
      std::bind(&ServoExample::timerCallback, this));
  }
  
  ~ServoExample()
  {
    if (servo_control_) {
      servo_control_->close();
    }
  }

private:
  void timerCallback()
  {
    static int count = 0;
    static bool direction = true;
    
    if (count % 5 == 0) {
      direction = !direction;
    }
    
    uint16_t position = direction ? 2000 : 1000;
    
    RCLCPP_INFO(this->get_logger(), "Setting servo position: %d", position);
    
    // Set position for servo ID 1
    if (servo_control_->setPosition(1, position)) {
      RCLCPP_INFO(this->get_logger(), "Successfully set position");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set position");
    }
    
    count++;
  }
  
  std::unique_ptr<servo_control::ServoControl> servo_control_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoExample>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 