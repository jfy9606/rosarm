// Copyright (c) 2023
// Author: Servo SDK Team

#ifndef GROUP_SYNC_READ_H
#define GROUP_SYNC_READ_H

#include <vector>
#include <map>
#include <memory>

// Forward declarations
class PortHandler;
class PacketHandler;

class GroupSyncRead {
public:
  GroupSyncRead(PortHandler* port, PacketHandler* packet, uint8_t start_address, uint8_t data_length);
  ~GroupSyncRead();

  bool addParam(uint8_t id);
  void removeParam(uint8_t id);
  void clearParam();
  
  int txRxPacket();
  bool isAvailable(uint8_t id, uint8_t address, uint8_t data_length);
  
  uint8_t getData(uint8_t id, uint8_t address, uint8_t data_length);
  uint16_t getData16(uint8_t id, uint8_t address);
  uint32_t getData32(uint8_t id, uint8_t address);

private:
  PortHandler* port_;
  PacketHandler* packet_handler_;
  
  uint8_t start_address_;
  uint8_t data_length_;
  
  std::vector<uint8_t> id_list_;
  std::map<uint8_t, std::vector<uint8_t>> data_dict_;
  
  bool is_param_changed_;
};

#endif // GROUP_SYNC_READ_H 