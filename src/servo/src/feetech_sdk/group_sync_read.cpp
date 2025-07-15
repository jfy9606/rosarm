#include "servo/feetech_sdk/group_sync_read.h"
#include "servo/feetech_sdk/port_handler.h"
#include "servo/feetech_sdk/packet_handler.h"
#include <algorithm>
#include <iostream>

GroupSyncRead::GroupSyncRead(PortHandler* port, PacketHandler* packet, uint8_t start_address, uint8_t data_length)
  : port_(port), packet_handler_(packet), start_address_(start_address), data_length_(data_length), is_param_changed_(false)
{
}

GroupSyncRead::~GroupSyncRead()
{
  clearParam();
}

bool GroupSyncRead::addParam(uint8_t id)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end()) {
    return false; // ID already exists
  }

  id_list_.push_back(id);
  data_dict_[id] = std::vector<uint8_t>(data_length_, 0);
  is_param_changed_ = true;

  return true;
}

void GroupSyncRead::removeParam(uint8_t id)
{
  auto it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it != id_list_.end()) {
    id_list_.erase(it);
    data_dict_.erase(id);
    is_param_changed_ = true;
  }
}

void GroupSyncRead::clearParam()
{
  id_list_.clear();
  data_dict_.clear();
  is_param_changed_ = true;
}

int GroupSyncRead::txRxPacket()
{
  if (id_list_.empty()) {
    return PacketHandler::COMM_SUCCESS;
  }

  if (is_param_changed_ == true || true) {  // This is a simplification
    // Construct packet
    const int TX_MAX_SIZE = 1024;
    uint8_t txpacket[TX_MAX_SIZE] = {0};
    
    txpacket[0] = 0xFF;                       // Header
    txpacket[1] = 0xFF;                       // Header
    txpacket[2] = 0xFE;                       // ID (Broadcast ID)
    txpacket[3] = 4 + (1 * id_list_.size());  // Length
    txpacket[4] = 0x02;                       // Instruction (READ)
    
    // Add parameters
    int index = 5;
    txpacket[index++] = start_address_;
    txpacket[index++] = data_length_;
    
    for (auto id : id_list_) {
      txpacket[index++] = id;
    }
    
    // Add checksum
    txpacket[index] = packet_handler_->calculateChecksum(txpacket, index);
    
    // Transmit packet
    port_->clearPort();
    
    // Send packet
    if (port_->writePort(txpacket, index + 1) != index + 1) {
      return PacketHandler::COMM_TX_FAIL;
    }
    
    // Receive responses
    for (size_t i = 0; i < id_list_.size(); i++) {
      uint8_t rxpacket[1024];
      
      // A simplified implementation - in a real scenario, this would need to be more sophisticated
      if (port_->readPort(rxpacket, data_length_ + 6) != data_length_ + 6) {
        return PacketHandler::COMM_RX_TIMEOUT;
      }
      
      // Check ID matches
      if (rxpacket[2] != id_list_[i]) {
        return PacketHandler::COMM_RX_CORRUPT;
      }
      
      // Copy data to buffer
      for (int j = 0; j < data_length_; j++) {
        data_dict_[id_list_[i]][j] = rxpacket[5 + j];
      }
    }
    
    is_param_changed_ = false;
  }

  return PacketHandler::COMM_SUCCESS;
}

bool GroupSyncRead::isAvailable(uint8_t id, uint8_t address, uint8_t data_length)
{
  // Check if ID exists
  auto it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end()) {
    return false;
  }
  
  // Check if data is in range
  if (address < start_address_ || address >= start_address_ + data_length_) {
    return false;
  }
  
  if (data_length > data_length_ - (address - start_address_)) {
    return false;
  }
  
  return true;
}

uint8_t GroupSyncRead::getData(uint8_t id, uint8_t address, uint8_t data_length)
{
  if (!isAvailable(id, address, data_length)) {
    return 0;
  }
  
  return data_dict_[id][address - start_address_];
}

uint16_t GroupSyncRead::getData16(uint8_t id, uint8_t address)
{
  if (!isAvailable(id, address, 2)) {
    return 0;
  }
  
  return (uint16_t)data_dict_[id][address - start_address_] | 
         ((uint16_t)data_dict_[id][address - start_address_ + 1] << 8);
}

uint32_t GroupSyncRead::getData32(uint8_t id, uint8_t address)
{
  if (!isAvailable(id, address, 4)) {
    return 0;
  }
  
  return (uint32_t)data_dict_[id][address - start_address_] | 
         ((uint32_t)data_dict_[id][address - start_address_ + 1] << 8) |
         ((uint32_t)data_dict_[id][address - start_address_ + 2] << 16) |
         ((uint32_t)data_dict_[id][address - start_address_ + 3] << 24);
} 