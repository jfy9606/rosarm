// Copyright (c) 2023
// Author: Servo SDK Team

#ifndef PORT_HANDLER_H
#define PORT_HANDLER_H

#include <serial/serial.h>
#include <memory>
#include <string>

class PortHandler {
public:
  PortHandler(const std::string& port_name);
  ~PortHandler();

  bool openPort();
  void closePort();
  bool setBaudRate(int baud_rate);
  bool isOpen() const;
  
  int readPort(uint8_t* packet, int length);
  int writePort(uint8_t* packet, int length);
  void clearPort();

private:
  std::string port_name_;
  std::shared_ptr<serial::Serial> serial_port_;
  bool is_open_;
};

#endif // PORT_HANDLER_H 