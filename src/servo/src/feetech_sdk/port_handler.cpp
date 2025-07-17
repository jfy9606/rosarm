#include "servo/feetech_sdk/port_handler.h"
#include <serial/serial.h>
#include <chrono>
#include <iostream>
#include <memory>

static std::shared_ptr<serial::Serial> serial_port_;

PortHandler::PortHandler(const char* port_name)
  : port_name_(port_name), socket_fd_(-1), baudrate_(0), is_open_(false), 
    packet_start_time_(0.0), packet_timeout_(0.0)
{
}

PortHandler::~PortHandler()
{
  closePort();
}

bool PortHandler::openPort()
{
  try {
    serial_port_ = std::make_shared<serial::Serial>(
      port_name_,
      115200,  // Default baudrate
      serial::Timeout::simpleTimeout(100)
    );
    
    is_open_ = true;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error opening serial port: " << e.what() << std::endl;
    is_open_ = false;
    return false;
  }
}

void PortHandler::closePort()
{
  if (serial_port_ && serial_port_->isOpen()) {
    serial_port_->close();
  }
  is_open_ = false;
}

bool PortHandler::setBaudRate(int baudrate)
{
  if (!isOpen()) {
    return false;
  }
  
  try {
    serial_port_->setBaudrate(baudrate);
    baudrate_ = baudrate;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error setting baudrate: " << e.what() << std::endl;
    return false;
  }
}

int PortHandler::getBaudRate()
{
  return baudrate_;
}

int PortHandler::readPort(uint8_t* packet, int length)
{
  if (!isOpen()) {
    return 0;
  }
  
  try {
    size_t bytes_read = serial_port_->read(packet, length);
    return bytes_read;
  } catch (const std::exception& e) {
    std::cerr << "Error reading from port: " << e.what() << std::endl;
    return 0;
  }
}

int PortHandler::writePort(uint8_t* packet, int length)
{
  if (!isOpen()) {
    return 0;
  }
  
  try {
    size_t bytes_written = serial_port_->write(packet, length);
    return bytes_written;
  } catch (const std::exception& e) {
    std::cerr << "Error writing to port: " << e.what() << std::endl;
    return 0;
  }
}

void PortHandler::setPacketTimeout(uint16_t packet_length)
{
  // Calculate timeout based on packet length and baudrate
  // Assume 10 bits per byte (8 data bits + start bit + stop bit)
  if (baudrate_ != 0) {
    packet_timeout_ = (static_cast<double>(packet_length) * 10.0 / baudrate_ * 1000.0) + 5.0;
  } else {
    packet_timeout_ = 100.0; // Default 100ms
  }
  
  packet_start_time_ = std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now().time_since_epoch()
  ).count();
}

void PortHandler::setPacketTimeout(double msec)
{
  packet_timeout_ = msec;
  packet_start_time_ = std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now().time_since_epoch()
  ).count();
}

bool PortHandler::isPacketTimeout()
{
  double current_time = std::chrono::duration<double, std::milli>(
    std::chrono::high_resolution_clock::now().time_since_epoch()
  ).count();
  
  return (current_time - packet_start_time_) > packet_timeout_;
} 