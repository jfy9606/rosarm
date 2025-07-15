#include "servo/feetech_sdk/packet_handler.h"
#include "servo/feetech_sdk/port_handler.h"
#include <iostream>
#include <cstring>

PacketHandler::PacketHandler()
  : protocolVersion_(1)  // Default to protocol 1
{
}

PacketHandler::~PacketHandler()
{
}

int PacketHandler::ping(PortHandler* port, uint8_t id, uint8_t* error)
{
  // Implementation for ping - simplified version
  uint8_t txpacket[10] = {0};
  uint8_t rxpacket[10] = {0};
  
  txpacket[0] = 0xFF;      // Header
  txpacket[1] = 0xFF;      // Header
  txpacket[2] = id;        // ID
  txpacket[3] = 0x02;      // Length
  txpacket[4] = 0x01;      // Instruction (PING)
  txpacket[5] = calculateChecksum(txpacket, 5);  // Checksum
  
  // Send packet
  if (port->writePort(txpacket, 6) != 6) {
    return COMM_TX_FAIL;
  }
  
  // Read response
  int result = COMM_SUCCESS;
  uint8_t rx_length = 0;
  
  // Implementation simplified for example purposes
  if (port->readPort(rxpacket, 6) != 6) {
    return COMM_RX_TIMEOUT;
  }
  
  // Error handling code would go here
  if (error != nullptr) *error = rxpacket[4];
  
  return result;
}

int PacketHandler::write1ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint8_t data, uint8_t* error)
{
  uint8_t txpacket[10] = {0};
  uint8_t rxpacket[10] = {0};
  
  // Construct packet
  txpacket[0] = 0xFF;      // Header
  txpacket[1] = 0xFF;      // Header
  txpacket[2] = id;        // ID
  txpacket[3] = 0x04;      // Length
  txpacket[4] = 0x03;      // Instruction (WRITE)
  txpacket[5] = address;   // Parameter 1: Address
  txpacket[6] = data;      // Parameter 2: Data
  txpacket[7] = calculateChecksum(txpacket, 7);  // Checksum
  
  // Send packet
  if (port->writePort(txpacket, 8) != 8) {
    return COMM_TX_FAIL;
  }
  
  // Simplified - for real implementation, would need to handle various responses
  if (id == 254) {  // Broadcast ID - no response
    return COMM_SUCCESS;
  }
  
  // Read response - simplified
  if (port->readPort(rxpacket, 6) != 6) {
    return COMM_RX_TIMEOUT;
  }
  
  // Error handling code would go here
  if (error != nullptr) *error = rxpacket[4];
  
  return COMM_SUCCESS;
}

int PacketHandler::write2ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint16_t data, uint8_t* error)
{
  uint8_t txpacket[12] = {0};
  uint8_t rxpacket[10] = {0};
  
  // Construct packet
  txpacket[0] = 0xFF;      // Header
  txpacket[1] = 0xFF;      // Header
  txpacket[2] = id;        // ID
  txpacket[3] = 0x05;      // Length
  txpacket[4] = 0x03;      // Instruction (WRITE)
  txpacket[5] = address;   // Parameter 1: Address
  txpacket[6] = (data & 0xFF);       // Parameter 2: Low byte
  txpacket[7] = ((data >> 8) & 0xFF); // Parameter 3: High byte
  txpacket[8] = calculateChecksum(txpacket, 8);  // Checksum
  
  // Send packet
  if (port->writePort(txpacket, 9) != 9) {
    return COMM_TX_FAIL;
  }
  
  // Simplified - for real implementation, would need to handle various responses
  if (id == 254) {  // Broadcast ID - no response
    return COMM_SUCCESS;
  }
  
  // Read response - simplified
  if (port->readPort(rxpacket, 6) != 6) {
    return COMM_RX_TIMEOUT;
  }
  
  // Error handling code would go here
  if (error != nullptr) *error = rxpacket[4];
  
  return COMM_SUCCESS;
}

int PacketHandler::read1ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint8_t* data, uint8_t* error)
{
  uint8_t txpacket[10] = {0};
  uint8_t rxpacket[10] = {0};
  
  // Construct packet
  txpacket[0] = 0xFF;      // Header
  txpacket[1] = 0xFF;      // Header
  txpacket[2] = id;        // ID
  txpacket[3] = 0x04;      // Length
  txpacket[4] = 0x02;      // Instruction (READ)
  txpacket[5] = address;   // Parameter 1: Address
  txpacket[6] = 0x01;      // Parameter 2: Data Length
  txpacket[7] = calculateChecksum(txpacket, 7);  // Checksum
  
  // Send packet
  if (port->writePort(txpacket, 8) != 8) {
    return COMM_TX_FAIL;
  }
  
  // Read response - simplified
  if (port->readPort(rxpacket, 7) != 7) {
    return COMM_RX_TIMEOUT;
  }
  
  // Error handling code would go here
  if (error != nullptr) *error = rxpacket[4];
  
  // Extract data
  *data = rxpacket[5];
  
  return COMM_SUCCESS;
}

int PacketHandler::read2ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint16_t* data, uint8_t* error)
{
  uint8_t txpacket[10] = {0};
  uint8_t rxpacket[12] = {0};
  
  // Construct packet
  txpacket[0] = 0xFF;      // Header
  txpacket[1] = 0xFF;      // Header
  txpacket[2] = id;        // ID
  txpacket[3] = 0x04;      // Length
  txpacket[4] = 0x02;      // Instruction (READ)
  txpacket[5] = address;   // Parameter 1: Address
  txpacket[6] = 0x02;      // Parameter 2: Data Length
  txpacket[7] = calculateChecksum(txpacket, 7);  // Checksum
  
  // Send packet
  if (port->writePort(txpacket, 8) != 8) {
    return COMM_TX_FAIL;
  }
  
  // Read response - simplified
  if (port->readPort(rxpacket, 8) != 8) {
    return COMM_RX_TIMEOUT;
  }
  
  // Error handling code would go here
  if (error != nullptr) *error = rxpacket[4];
  
  // Extract data
  *data = (uint16_t)rxpacket[5] | ((uint16_t)rxpacket[6] << 8);
  
  return COMM_SUCCESS;
}

uint8_t PacketHandler::calculateChecksum(uint8_t* packet, size_t length)
{
  uint8_t checksum = 0;
  
  for (size_t i = 2; i < length; i++) {
    checksum += packet[i];
  }
  
  return ~checksum;  // Bitwise inversion
}

const char* PacketHandler::getTxRxResult(int result)
{
  switch(result) {
    case COMM_SUCCESS:
      return "Success";
    case COMM_PORT_BUSY:
      return "Port busy";
    case COMM_TX_FAIL:
      return "TX fail";
    case COMM_RX_FAIL:
      return "RX fail";
    case COMM_RX_TIMEOUT:
      return "RX timeout";
    case COMM_RX_CORRUPT:
      return "RX corrupt";
    default:
      return "Unknown error";
  }
}

const char* PacketHandler::getErrorResult(uint8_t error)
{
  if (error & 0x01) return "Input voltage error";
  if (error & 0x02) return "Angle limit error";
  if (error & 0x04) return "Overheating error";
  if (error & 0x08) return "Range error";
  if (error & 0x10) return "Checksum error";
  if (error & 0x20) return "Overload error";
  if (error & 0x40) return "Instruction error";
  
  return "No error";
} 