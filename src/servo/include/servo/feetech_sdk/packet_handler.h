// Copyright (c) 2023
// Author: Servo SDK Team

#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include "port_handler.h"
#include <string>
#include <cstdint>

// Communication Result
#define COMM_SUCCESS        0   // Communication Success
#define COMM_PORT_BUSY      -1  // Port is busy (in use)
#define COMM_TX_FAIL        -2  // Failed transmit instruction packet
#define COMM_RX_FAIL        -3  // Failed get status packet
#define COMM_TX_ERROR       -4  // Incorrect instruction packet
#define COMM_RX_WAITING     -5  // Now receiving status packet
#define COMM_RX_TIMEOUT     -6  // There is no status packet
#define COMM_RX_CORRUPT     -7  // Incorrect status packet
#define COMM_NOT_AVAILABLE  -9  // Not available

// Macro for Control Table Value
#define SCS_LOBYTE(w)       ((uint8_t)((w) & 0xFF))
#define SCS_HIBYTE(w)       ((uint8_t)(((w) >> 8) & 0xFF))
#define SCS_MAKEWORD(l, h)  ((uint16_t)(((uint8_t)(l)) | ((uint16_t)((uint8_t)(h))) << 8))

class PacketHandler {
public:
  PacketHandler();
  virtual ~PacketHandler();

  // Transmission
  int ping(PortHandler* port, uint8_t id, uint8_t* error = nullptr);
  int read1ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint8_t* data, uint8_t* error = nullptr);
  int read2ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint16_t* data, uint8_t* error = nullptr);
  int read4ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint32_t* data, uint8_t* error = nullptr);
  int write1ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint8_t data, uint8_t* error = nullptr);
  int write2ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint16_t data, uint8_t* error = nullptr);
  int write4ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint32_t data, uint8_t* error = nullptr);
  
  // Sync Read/Write
  int syncReadTx(PortHandler* port, uint8_t* param, uint16_t param_length);
  int readRx(PortHandler* port, uint8_t data_length, uint8_t* data, uint8_t* error = nullptr);
  int syncWriteTxOnly(PortHandler* port, uint8_t* param, uint16_t param_length);
  
  // Utility
  const char* getTxRxResult(int result);
  const char* getRxPacketError(uint8_t error);
  
private:
  int protocol_version_;
  uint8_t tx_packet_[256];
  uint8_t rx_packet_[256];
  
  int txPacket(PortHandler* port);
  int rxPacket(PortHandler* port);
  int txRxPacket(PortHandler* port);
  
  uint16_t updateCRC(uint16_t crc_accum, uint8_t* data_blk_ptr, uint8_t data_blk_size);
};

#endif // PACKET_HANDLER_H 