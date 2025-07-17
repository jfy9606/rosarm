// Copyright (c) 2023
// Author: Servo SDK Team

#ifndef GROUP_SYNC_READ_H
#define GROUP_SYNC_READ_H

#include <map>
#include <vector>
#include <cstdint>
#include "port_handler.h"
#include "packet_handler.h"

class GroupSyncRead {
public:
  GroupSyncRead(PortHandler* port, PacketHandler* ph, uint8_t start_address, uint8_t data_length);
  ~GroupSyncRead();

  bool addParam(uint8_t id);
  void removeParam(uint8_t id);
  void clearParam();
  
  int txRxPacket();
  int txPacket();
  int rxPacket();
  
  bool isAvailable(uint8_t id, uint8_t address, uint8_t data_length);
  uint8_t getData(uint8_t id, uint8_t address, uint8_t data_length);
  uint16_t getData16(uint8_t id, uint8_t address);
  uint32_t getData32(uint8_t id, uint8_t address);

private:
  PortHandler* port_;
  PacketHandler* ph_;
  
  uint8_t start_address_;
  uint8_t data_length_;
  
  std::map<uint8_t, uint8_t*> data_list_;  // <id, data>
  std::vector<uint8_t> id_list_;
  
  bool is_param_changed_;
  uint8_t* param_;
  uint16_t param_length_;
};

#endif // GROUP_SYNC_READ_H 