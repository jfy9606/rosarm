#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include "servo/servo_control.hpp"

int main(int argc, char* argv[])
{
  std::cout << "Initializing Servo Example..." << std::endl;
  
  // Create servo control
  auto servo_control = std::make_unique<servo_control::ServoControl>("/dev/ttyUSB0", 1000000);
  
  // Initialize
  if (servo_control->init()) {
    std::cout << "Servo control initialized successfully" << std::endl;
  } else {
    std::cerr << "Failed to initialize servo control" << std::endl;
    return 1;
  }
  
  // Control loop
  int count = 0;
  bool direction = true;
  
  for (int i = 0; i < 20; i++) {
    if (count % 5 == 0) {
      direction = !direction;
    }
    
    uint16_t position = direction ? 2000 : 1000;
    
    std::cout << "Setting servo position: " << position << std::endl;
    
    // Set position for servo ID 1
    if (servo_control->setPosition(1, position)) {
      std::cout << "Successfully set position" << std::endl;
    } else {
      std::cerr << "Failed to set position" << std::endl;
    }
    
    count++;
    
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  
  // Close connection
  servo_control->close();
  
  std::cout << "Example completed." << std::endl;
  
  return 0;
} 