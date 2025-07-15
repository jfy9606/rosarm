// Copyright (c) 2023
// Author: Servo SDK Team

#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include <stdint.h>
#include <vector>
#include <cstddef>  // for size_t

// Forward declaration
class PortHandler;

class PacketHandler {
public:
  PacketHandler();
  ~PacketHandler();

  // Communication result
  enum CommResult {
    COMM_SUCCESS,
    COMM_PORT_BUSY,
    COMM_TX_FAIL,
    COMM_RX_FAIL,
    COMM_RX_TIMEOUT,
    COMM_RX_CORRUPT,
    COMM_UNKNOWN_ERROR
  };

  // FT Servo error
  enum FTServoError {
    SERVO_OK,
    SERVO_ERROR_OVERLOAD,
    SERVO_ERROR_OVERHEAT,
    SERVO_ERROR_VOLTAGE,
    SERVO_ERROR_POSITION
  };

  // Basic command methods
  int write1ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint8_t data, uint8_t* error = nullptr);
  int write2ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint16_t data, uint8_t* error = nullptr);
  int read1ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint8_t* data, uint8_t* error = nullptr);
  int read2ByteTxRx(PortHandler* port, uint8_t id, uint8_t address, uint16_t* data, uint8_t* error = nullptr);

  // Ping method
  int ping(PortHandler* port, uint8_t id, uint8_t* error = nullptr);

  // Utility methods
  uint8_t calculateChecksum(uint8_t* packet, size_t length);
  const char* getTxRxResult(int result);
  const char* getErrorResult(uint8_t error);

private:
  int protocolVersion_;
};

#endif // PACKET_HANDLER_H 