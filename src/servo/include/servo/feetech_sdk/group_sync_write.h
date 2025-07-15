// Copyright (c) 2023
// Author: Servo SDK Team

#ifndef GROUP_SYNC_WRITE_H
#define GROUP_SYNC_WRITE_H

#include <vector>
#include <map>
#include <memory>

// Forward declarations
class PortHandler;
class PacketHandler;

class GroupSyncWrite {
public:
  GroupSyncWrite(PortHandler* port, PacketHandler* packet, uint8_t start_address, uint8_t data_length);
  ~GroupSyncWrite();

  bool addParam(uint8_t id, const uint8_t* data);
  void removeParam(uint8_t id);
  void clearParam();
  
  int txPacket();
  bool changeParam(uint8_t id, const uint8_t* data);

private:
  PortHandler* port_;
  PacketHandler* packet_handler_;
  
  uint8_t start_address_;
  uint8_t data_length_;
  
  std::vector<uint8_t> id_list_;
  std::map<uint8_t, std::vector<uint8_t>> data_dict_;
  
  bool is_param_changed_;
};

#endif // GROUP_SYNC_WRITE_H 