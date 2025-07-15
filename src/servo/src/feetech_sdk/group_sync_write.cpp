#include "servo/feetech_sdk/group_sync_write.h"
#include "servo/feetech_sdk/port_handler.h"
#include "servo/feetech_sdk/packet_handler.h"
#include <algorithm>
#include <iostream>

GroupSyncWrite::GroupSyncWrite(PortHandler* port, PacketHandler* packet, uint8_t start_address, uint8_t data_length)
  : port_(port), packet_handler_(packet), start_address_(start_address), data_length_(data_length), is_param_changed_(false)
{
}

GroupSyncWrite::~GroupSyncWrite()
{
  clearParam();
}

bool GroupSyncWrite::addParam(uint8_t id, const uint8_t* data)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end()) {
    return false; // ID already exists
  }

  id_list_.push_back(id);
  
  // Copy data to the map
  std::vector<uint8_t> data_vec(data_length_);
  for (size_t i = 0; i < data_length_; i++) {
    data_vec[i] = data[i];
  }
  
  data_dict_[id] = data_vec;
  is_param_changed_ = true;

  return true;
}

void GroupSyncWrite::removeParam(uint8_t id)
{
  auto it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it != id_list_.end()) {
    id_list_.erase(it);
    data_dict_.erase(id);
    is_param_changed_ = true;
  }
}

void GroupSyncWrite::clearParam()
{
  id_list_.clear();
  data_dict_.clear();
  is_param_changed_ = true;
}

int GroupSyncWrite::txPacket()
{
  if (id_list_.empty()) {
    return PacketHandler::COMM_SUCCESS;
  }

  if (is_param_changed_ == true || true) {  // This is a simplification
    // Construct packet
    const int TX_MAX_SIZE = 1024;
    uint8_t txpacket[TX_MAX_SIZE] = {0};
    
    txpacket[0] = 0xFF;                                        // Header
    txpacket[1] = 0xFF;                                        // Header
    txpacket[2] = 0xFE;                                        // ID (Broadcast ID)
    txpacket[3] = 4 + (1 + data_length_) * id_list_.size();   // Length
    txpacket[4] = 0x83;                                        // Instruction (SYNC WRITE)
    
    // Add parameters
    int index = 5;
    txpacket[index++] = start_address_;
    txpacket[index++] = data_length_;
    
    for (auto id : id_list_) {
      txpacket[index++] = id;
      
      for (size_t j = 0; j < data_length_; j++) {
        txpacket[index++] = data_dict_[id][j];
      }
    }
    
    // Add checksum
    txpacket[index] = packet_handler_->calculateChecksum(txpacket, index);
    
    // Transmit packet
    port_->clearPort();
    
    // Send packet
    if (port_->writePort(txpacket, index + 1) != index + 1) {
      return PacketHandler::COMM_TX_FAIL;
    }
    
    is_param_changed_ = false;
  }

  return PacketHandler::COMM_SUCCESS;
}

bool GroupSyncWrite::changeParam(uint8_t id, const uint8_t* data)
{
  auto it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end()) {
    return false; // ID doesn't exist
  }
  
  // Update data
  for (size_t i = 0; i < data_length_; i++) {
    data_dict_[id][i] = data[i];
  }
  
  is_param_changed_ = true;
  return true;
} 