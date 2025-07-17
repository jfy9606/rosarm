// Copyright (c) 2023
// Author: Servo SDK Team

#ifndef GROUP_SYNC_WRITE_H
#define GROUP_SYNC_WRITE_H

#include <map>
#include <vector>
#include <cstdint>
#include "port_handler.h"
#include "packet_handler.h"

class GroupSyncWrite {
public:
  GroupSyncWrite(PortHandler* port, PacketHandler* ph, uint8_t start_address, uint8_t data_length);
  ~GroupSyncWrite();

  bool addParam(uint8_t id, const uint8_t* data);
  void removeParam(uint8_t id);
  void clearParam();
  bool changeParam(uint8_t id, const uint8_t* data);
  
  int txPacket();

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

#endif // GROUP_SYNC_WRITE_H 