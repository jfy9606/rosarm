// Copyright (c) 2023
// Author: Servo SDK Team

#ifndef PORT_HANDLER_H
#define PORT_HANDLER_H

#include <string>
#include <cstdint>

class PortHandler {
public:
  PortHandler(const char* port_name);
  virtual ~PortHandler();

  bool openPort();
  void closePort();
  bool setBaudRate(int baudrate);
  int getBaudRate();
  
  int readPort(uint8_t* packet, int length);
  int writePort(uint8_t* packet, int length);
  
  void setPacketTimeout(uint16_t packet_length);
  void setPacketTimeout(double msec);
  bool isPacketTimeout();
  
  bool isOpen() const { return is_open_; }
  
private:
  std::string port_name_;
  int socket_fd_;
  int baudrate_;
  bool is_open_;
  double packet_start_time_;
  double packet_timeout_;
};

#endif // PORT_HANDLER_H 