#include "servo/feetech_sdk/port_handler.h"
#include <iostream>

PortHandler::PortHandler(const std::string& port_name)
  : port_name_(port_name), is_open_(false)
{
  serial_port_ = std::make_shared<serial::Serial>();
}

PortHandler::~PortHandler()
{
  closePort();
}

bool PortHandler::openPort()
{
  try {
    serial_port_->setPort(port_name_);
    serial_port_->setBaudrate(1000000); // Default baudrate
    serial_port_->setBytesize(serial::eightbits);
    serial_port_->setParity(serial::parity_none);
    serial_port_->setStopbits(serial::stopbits_one);
    serial_port_->setFlowcontrol(serial::flowcontrol_none);
    
    // Use the set timeout correctly - create a timeout object first
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100); // Default timeout
    serial_port_->setTimeout(timeout);
    
    serial_port_->open();
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
  if (serial_port_->isOpen()) {
    serial_port_->close();
    is_open_ = false;
  }
}

bool PortHandler::setBaudRate(int baud_rate)
{
  if (!is_open_) {
    return false;
  }
  
  try {
    serial_port_->setBaudrate(baud_rate);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error setting baudrate: " << e.what() << std::endl;
    return false;
  }
}

bool PortHandler::isOpen() const
{
  return is_open_ && serial_port_->isOpen();
}

int PortHandler::readPort(uint8_t* packet, int length)
{
  if (!isOpen()) {
    return 0;
  }
  
  try {
    return serial_port_->read(packet, length);
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
    return serial_port_->write(packet, length);
  } catch (const std::exception& e) {
    std::cerr << "Error writing to port: " << e.what() << std::endl;
    return 0;
  }
}

void PortHandler::clearPort()
{
  if (isOpen()) {
    serial_port_->flushInput();
    serial_port_->flushOutput();
  }
} 