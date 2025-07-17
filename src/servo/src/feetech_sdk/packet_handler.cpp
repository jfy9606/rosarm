#include "servo/feetech_sdk/packet_handler.h"
#include <cstring>
#include <iostream>
#include <cstdint>

// Instruction for SCS Protocol
#define SCS_INST_PING           0x01
#define SCS_INST_READ           0x02
#define SCS_INST_WRITE          0x03
#define SCS_INST_REG_WRITE      0x04
#define SCS_INST_REG_ACTION     0x05
#define SCS_INST_SYNC_WRITE     0x83
#define SCS_INST_SYNC_READ      0x82

// Protocol
#define SCS_HEADER              0xFF
#define SCS_RESERVED            0x00

// Error for Protocol
#define SCS_ERROR_INSTRUCTION   0x40
#define SCS_ERROR_OVERLOAD      0x20
#define SCS_ERROR_CHECKSUM      0x10
#define SCS_ERROR_RANGE         0x08
#define SCS_ERROR_OVERHEAT      0x04
#define SCS_ERROR_ANGLE_LIMIT   0x02
#define SCS_ERROR_VOLTAGE       0x01

PacketHandler::PacketHandler()
  : protocol_version_(0)
{
}

PacketHandler::~PacketHandler()
{
}

int PacketHandler::ping(PortHandler* port, uint8_t id, uint8_t* error)
{
  int result = COMM_TX_FAIL;
  
  // Make packet
  tx_packet_[0] = SCS_HEADER;
  tx_packet_[1] = SCS_HEADER;
  tx_packet_[2] = id;
  tx_packet_[3] = 2;  // Length (2 = 1 byte instruction + 1 byte checksum)
  tx_packet_[4] = SCS_INST_PING;
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 2; i < 5; i++) {
    checksum += tx_packet_[i];
  }
  tx_packet_[5] = ~checksum;  // Invert for checksum
  
  // Transmit packet
  port->setPacketTimeout(static_cast<uint16_t>(6));  // 6 bytes to write
  result = txRxPacket(port);
  
  // Check result
  if (result == COMM_SUCCESS && error != nullptr) {
    *error = (rx_packet_[4] & 0x7F);  // Error byte
  }
  
  return result;
}

int PacketHandler::read1ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint8_t* data, uint8_t* error)
{
  int result = COMM_TX_FAIL;
  
  // Make packet
  tx_packet_[0] = SCS_HEADER;
  tx_packet_[1] = SCS_HEADER;
  tx_packet_[2] = id;
  tx_packet_[3] = 4;  // Length (2 bytes instruction + 1 byte address + 1 byte length + 1 byte checksum)
  tx_packet_[4] = SCS_INST_READ;
  tx_packet_[5] = address;
  tx_packet_[6] = 1;  // Read 1 byte
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 2; i < 7; i++) {
    checksum += tx_packet_[i];
  }
  tx_packet_[7] = ~checksum;  // Invert for checksum
  
  // Transmit packet
  port->setPacketTimeout(static_cast<uint16_t>(8));  // 8 bytes to write
  result = txRxPacket(port);
  
  // Check result
  if (result == COMM_SUCCESS) {
    if (error != nullptr) {
      *error = (rx_packet_[4] & 0x7F);  // Error byte
    }
    
    *data = rx_packet_[5];  // Data is at position 5
  }
  
  return result;
}

int PacketHandler::read2ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint16_t* data, uint8_t* error)
{
  int result = COMM_TX_FAIL;
  
  // Make packet
  tx_packet_[0] = SCS_HEADER;
  tx_packet_[1] = SCS_HEADER;
  tx_packet_[2] = id;
  tx_packet_[3] = 4;  // Length (1 byte instruction + 1 byte address + 1 byte length + 1 byte checksum)
  tx_packet_[4] = SCS_INST_READ;
  tx_packet_[5] = address;
  tx_packet_[6] = 2;  // Read 2 bytes
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 2; i < 7; i++) {
    checksum += tx_packet_[i];
  }
  tx_packet_[7] = ~checksum;  // Invert for checksum
  
  // Transmit packet
  port->setPacketTimeout(static_cast<uint16_t>(8));  // 8 bytes to write
  result = txRxPacket(port);
  
  // Check result
  if (result == COMM_SUCCESS) {
    if (error != nullptr) {
      *error = (rx_packet_[4] & 0x7F);  // Error byte
    }
    
    // Combine two bytes into a 16-bit value
    *data = SCS_MAKEWORD(rx_packet_[5], rx_packet_[6]);
  }
  
  return result;
}

int PacketHandler::read4ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint32_t* data, uint8_t* error)
{
  int result = COMM_TX_FAIL;
  
  // Make packet
  tx_packet_[0] = SCS_HEADER;
  tx_packet_[1] = SCS_HEADER;
  tx_packet_[2] = id;
  tx_packet_[3] = 4;  // Length (1 byte instruction + 1 byte address + 1 byte length + 1 byte checksum)
  tx_packet_[4] = SCS_INST_READ;
  tx_packet_[5] = address;
  tx_packet_[6] = 4;  // Read 4 bytes
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 2; i < 7; i++) {
    checksum += tx_packet_[i];
  }
  tx_packet_[7] = ~checksum;  // Invert for checksum
  
  // Transmit packet
  port->setPacketTimeout(static_cast<uint16_t>(8));  // 8 bytes to write
  result = txRxPacket(port);
  
  // Check result
  if (result == COMM_SUCCESS) {
    if (error != nullptr) {
      *error = (rx_packet_[4] & 0x7F);  // Error byte
    }
    
    // Combine four bytes into a 32-bit value
    *data = ((uint32_t)rx_packet_[5]) | 
            ((uint32_t)rx_packet_[6] << 8) |
            ((uint32_t)rx_packet_[7] << 16) |
            ((uint32_t)rx_packet_[8] << 24);
  }
  
  return result;
}

int PacketHandler::write1ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint8_t data, uint8_t* error)
{
  int result = COMM_TX_FAIL;
  
  // Make packet
  tx_packet_[0] = SCS_HEADER;
  tx_packet_[1] = SCS_HEADER;
  tx_packet_[2] = id;
  tx_packet_[3] = 4;  // Length (1 byte instruction + 1 byte address + 1 byte data + 1 byte checksum)
  tx_packet_[4] = SCS_INST_WRITE;
  tx_packet_[5] = address;
  tx_packet_[6] = data;
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 2; i < 7; i++) {
    checksum += tx_packet_[i];
  }
  tx_packet_[7] = ~checksum;  // Invert for checksum
  
  // Transmit packet
  port->setPacketTimeout(static_cast<uint16_t>(8));  // 8 bytes to write
  result = txRxPacket(port);
  
  // Check result
  if (result == COMM_SUCCESS && error != nullptr) {
    *error = (rx_packet_[4] & 0x7F);  // Error byte
  }
  
  return result;
}

int PacketHandler::write2ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint16_t data, uint8_t* error)
{
  int result = COMM_TX_FAIL;
  
  // Make packet
  tx_packet_[0] = SCS_HEADER;
  tx_packet_[1] = SCS_HEADER;
  tx_packet_[2] = id;
  tx_packet_[3] = 5;  // Length (1 byte instruction + 1 byte address + 2 byte data + 1 byte checksum)
  tx_packet_[4] = SCS_INST_WRITE;
  tx_packet_[5] = address;
  tx_packet_[6] = SCS_LOBYTE(data);
  tx_packet_[7] = SCS_HIBYTE(data);
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 2; i < 8; i++) {
    checksum += tx_packet_[i];
  }
  tx_packet_[8] = ~checksum;  // Invert for checksum
  
  // Transmit packet
  port->setPacketTimeout(static_cast<uint16_t>(9));  // 9 bytes to write
  result = txRxPacket(port);
  
  // Check result
  if (result == COMM_SUCCESS && error != nullptr) {
    *error = (rx_packet_[4] & 0x7F);  // Error byte
  }
  
  return result;
}

int PacketHandler::write4ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint32_t data, uint8_t* error)
{
  int result = COMM_TX_FAIL;
  
  // Make packet
  tx_packet_[0] = SCS_HEADER;
  tx_packet_[1] = SCS_HEADER;
  tx_packet_[2] = id;
  tx_packet_[3] = 7;  // Length (1 byte instruction + 1 byte address + 4 byte data + 1 byte checksum)
  tx_packet_[4] = SCS_INST_WRITE;
  tx_packet_[5] = address;
  tx_packet_[6] = (uint8_t)(data & 0xFF);
  tx_packet_[7] = (uint8_t)((data >> 8) & 0xFF);
  tx_packet_[8] = (uint8_t)((data >> 16) & 0xFF);
  tx_packet_[9] = (uint8_t)((data >> 24) & 0xFF);
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 2; i < 10; i++) {
    checksum += tx_packet_[i];
  }
  tx_packet_[10] = ~checksum;  // Invert for checksum
  
  // Transmit packet
  port->setPacketTimeout(static_cast<uint16_t>(11));  // 11 bytes to write
  result = txRxPacket(port);
  
  // Check result
  if (result == COMM_SUCCESS && error != nullptr) {
    *error = (rx_packet_[4] & 0x7F);  // Error byte
  }
  
  return result;
}

int PacketHandler::syncReadTx(PortHandler* port, uint8_t* param, uint16_t param_length)
{
  int result = COMM_TX_FAIL;
  
  // Make packet
  tx_packet_[0] = SCS_HEADER;
  tx_packet_[1] = SCS_HEADER;
  tx_packet_[2] = 0xFE;  // Broadcast ID
  tx_packet_[3] = param_length + 1;  // Length (1 byte instruction + param_length)
  tx_packet_[4] = SCS_INST_SYNC_READ;
  
  // Copy parameters
  memcpy(&tx_packet_[5], param, param_length);
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 2; i < param_length + 5; i++) {
    checksum += tx_packet_[i];
  }
  tx_packet_[param_length + 5] = ~checksum;  // Invert for checksum
  
  // Transmit packet
  port->setPacketTimeout(static_cast<uint16_t>(param_length + 6));  // 6 bytes + param_length
  result = txPacket(port);
  
  return result;
}

int PacketHandler::readRx(PortHandler* port, uint8_t data_length, uint8_t* data, uint8_t* error)
{
  int result = COMM_RX_FAIL;
  
  // Set packet timeout
  port->setPacketTimeout(static_cast<uint16_t>(data_length + 6));  // 6 bytes + data_length
  
  // Receive packet
  result = rxPacket(port);
  
  // Check result
  if (result == COMM_SUCCESS) {
    if (error != nullptr) {
      *error = (rx_packet_[4] & 0x7F);  // Error byte
    }
    
    // Copy data
    memcpy(data, &rx_packet_[5], data_length);
  }
  
  return result;
}

int PacketHandler::syncWriteTxOnly(PortHandler* port, uint8_t* param, uint16_t param_length)
{
  int result = COMM_TX_FAIL;
  
  // Make packet
  tx_packet_[0] = SCS_HEADER;
  tx_packet_[1] = SCS_HEADER;
  tx_packet_[2] = 0xFE;  // Broadcast ID
  tx_packet_[3] = param_length + 1;  // Length (1 byte instruction + param_length)
  tx_packet_[4] = SCS_INST_SYNC_WRITE;
  
  // Copy parameters
  memcpy(&tx_packet_[5], param, param_length);
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 2; i < param_length + 5; i++) {
    checksum += tx_packet_[i];
  }
  tx_packet_[param_length + 5] = ~checksum;  // Invert for checksum
  
  // Transmit packet
  port->setPacketTimeout(static_cast<uint16_t>(param_length + 6));  // 6 bytes + param_length
  result = txPacket(port);
  
  return result;
}

int PacketHandler::txPacket(PortHandler* port)
{
  uint8_t packet_length = tx_packet_[3] + 4;  // Length field + 4 bytes (2 header + 1 ID + 1 Length)
  
  if (port->isOpen() == false) {
    return COMM_PORT_BUSY;
  }
  
  // Write packet to port
  int written_bytes = port->writePort(tx_packet_, packet_length);
  if (written_bytes != packet_length) {
    return COMM_TX_FAIL;
  }
  
  return COMM_SUCCESS;
}

int PacketHandler::rxPacket(PortHandler* port)
{
  int result = COMM_RX_FAIL;
  uint8_t checksum = 0;
  uint8_t rx_length = 0;
  uint8_t wait_length = 6;  // Minimum packet length (2 header + 1 ID + 1 Length + 1 Error + 1 Checksum)
  
  // Set initial timeout
  port->setPacketTimeout(static_cast<uint16_t>(wait_length));
  
  // Read header (2 bytes)
  uint8_t idx = 0;
  for (idx = 0; idx < 2; idx++) {
    result = port->readPort(&rx_packet_[idx], 1);
    if (result != 1) {
      return COMM_RX_TIMEOUT;
    }
    
    // Check header
    if (rx_packet_[idx] != SCS_HEADER) {
      return COMM_RX_CORRUPT;
    }
  }
  
  // Read ID (1 byte)
  result = port->readPort(&rx_packet_[2], 1);
  if (result != 1) {
    return COMM_RX_TIMEOUT;
  }
  
  // Read Length (1 byte)
  result = port->readPort(&rx_packet_[3], 1);
  if (result != 1) {
    return COMM_RX_TIMEOUT;
  }
  
  rx_length = rx_packet_[3];
  if (rx_length > 254) {  // Max packet size
    return COMM_RX_CORRUPT;
  }
  
  // Update packet timeout based on the length
  port->setPacketTimeout(static_cast<uint16_t>(rx_length));
  
  // Read the rest of the packet
  for (idx = 4; idx < rx_length + 4; idx++) {
    result = port->readPort(&rx_packet_[idx], 1);
    if (result != 1) {
      return COMM_RX_TIMEOUT;
    }
  }
  
  // Calculate checksum
  for (idx = 2; idx < rx_length + 3; idx++) {
    checksum += rx_packet_[idx];
  }
  checksum = ~checksum;  // Invert for checksum
  
  // Verify checksum
  if (checksum != rx_packet_[rx_length + 3]) {
    return COMM_RX_CORRUPT;
  }
  
  return COMM_SUCCESS;
}

int PacketHandler::txRxPacket(PortHandler* port)
{
  int result = COMM_TX_FAIL;
  
  // Transmit packet
  result = txPacket(port);
  if (result != COMM_SUCCESS) {
    return result;
  }
  
  // Wait for response
  port->setPacketTimeout(static_cast<uint16_t>(6));  // Minimum packet length
  result = rxPacket(port);
  
  return result;
}

uint16_t PacketHandler::updateCRC(uint16_t crc_accum, uint8_t* data_blk_ptr, uint8_t data_blk_size)
{
  uint16_t i, j;
  uint16_t crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };
  
  for (j = 0; j < data_blk_size; j++) {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  
  return crc_accum;
}

const char* PacketHandler::getTxRxResult(int result)
{
  switch(result) {
    case COMM_SUCCESS:
      return "COMM_SUCCESS";
    case COMM_PORT_BUSY:
      return "COMM_PORT_BUSY";
    case COMM_TX_FAIL:
      return "COMM_TX_FAIL";
    case COMM_RX_FAIL:
      return "COMM_RX_FAIL";
    case COMM_TX_ERROR:
      return "COMM_TX_ERROR";
    case COMM_RX_WAITING:
      return "COMM_RX_WAITING";
    case COMM_RX_TIMEOUT:
      return "COMM_RX_TIMEOUT";
    case COMM_RX_CORRUPT:
      return "COMM_RX_CORRUPT";
    case COMM_NOT_AVAILABLE:
      return "COMM_NOT_AVAILABLE";
    default:
      return "UNKNOWN_ERROR";
  }
}

const char* PacketHandler::getRxPacketError(uint8_t error)
{
  if (error & SCS_ERROR_INSTRUCTION)
    return "Instruction Error";
  else if (error & SCS_ERROR_OVERLOAD)
    return "Overload Error";
  else if (error & SCS_ERROR_CHECKSUM)
    return "Checksum Error";
  else if (error & SCS_ERROR_RANGE)
    return "Range Error";
  else if (error & SCS_ERROR_OVERHEAT)
    return "Overheat Error";
  else if (error & SCS_ERROR_ANGLE_LIMIT)
    return "Angle Limit Error";
  else if (error & SCS_ERROR_VOLTAGE)
    return "Voltage Error";
  else
    return "Unknown Error";
} 