#include "servo/feetech_sdk/group_sync_read.h"
#include <algorithm>
#include <iostream>
#include <cstring>

#define SCS_INST_SYNC_READ      0x82

GroupSyncRead::GroupSyncRead(PortHandler* port, PacketHandler* ph, uint8_t start_address, uint8_t data_length)
  : port_(port), ph_(ph), start_address_(start_address), data_length_(data_length),
    is_param_changed_(false), param_(nullptr), param_length_(0)
{
  clearParam();
}

GroupSyncRead::~GroupSyncRead()
{
  clearParam();
}

bool GroupSyncRead::addParam(uint8_t id)
{
  // If ID already exists, return false
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())
    return false;
  
  // Add ID to the list
  id_list_.push_back(id);
  
  // Create data buffer for this ID
  data_list_[id] = new uint8_t[data_length_];
  
  is_param_changed_ = true;
  return true;
}

void GroupSyncRead::removeParam(uint8_t id)
{
  // Find ID in the list
  auto it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())
    return;
  
  // Remove ID from the list
  id_list_.erase(it);
  
  // Delete data buffer for this ID
  delete[] data_list_[id];
  data_list_.erase(id);
  
  is_param_changed_ = true;
}

void GroupSyncRead::clearParam()
{
  // Delete all data buffers
  for (auto const& id : id_list_) {
    if (data_list_.count(id) > 0) {
      delete[] data_list_[id];
    }
  }
  
  // Clear lists
  id_list_.clear();
  data_list_.clear();
  
  // Delete parameter buffer
  if (param_ != nullptr)
    delete[] param_;
  param_ = nullptr;
  param_length_ = 0;
  
  is_param_changed_ = false;
}

int GroupSyncRead::txPacket()
{
  if (id_list_.empty())
    return COMM_NOT_AVAILABLE;
  
  if (is_param_changed_ == true || param_ == nullptr) {
    if (param_ != nullptr)
      delete[] param_;
    
    // Calculate parameter length
    param_length_ = id_list_.size() + 2;  // ID list length + start address + data length
    
    // Allocate parameter buffer
    param_ = new uint8_t[param_length_];
    
    // Fill parameter buffer
    param_[0] = start_address_;
    param_[1] = data_length_;
    for (size_t i = 0; i < id_list_.size(); i++) {
      param_[i + 2] = id_list_[i];
    }
    
    is_param_changed_ = false;
  }
  
  return ph_->syncReadTx(port_, param_, param_length_);
}

int GroupSyncRead::rxPacket()
{
  int result = COMM_RX_FAIL;
  
  // Clear data buffers
  for (auto const& id : id_list_) {
    if (data_list_.count(id) > 0) {
      memset(data_list_[id], 0, data_length_);
    }
  }
  
  // Receive data for each ID
  for (auto const& id : id_list_) {
    uint8_t error = 0;
    
    // Read data
    result = ph_->readRx(port_, data_length_, data_list_[id], &error);
    if (result != COMM_SUCCESS)
      return result;
  }
  
  return result;
}

int GroupSyncRead::txRxPacket()
{
  int result = COMM_TX_FAIL;
  
  // Transmit packet
  result = txPacket();
  if (result != COMM_SUCCESS)
    return result;
  
  // Receive packet
  return rxPacket();
}

bool GroupSyncRead::isAvailable(uint8_t id, uint8_t address, uint8_t data_length)
{
  // Check if ID exists
  if (data_list_.count(id) == 0)
    return false;
  
  // Check if the requested address is within range
  if (address < start_address_ || start_address_ + data_length_ < address + data_length)
    return false;
  
  return true;
}

uint8_t GroupSyncRead::getData(uint8_t id, uint8_t address, uint8_t data_length)
{
  if (isAvailable(id, address, data_length) == false)
    return 0;
  
  // Calculate offset
  uint8_t offset = address - start_address_;
  
  // Return data
  return data_list_[id][offset];
}

uint16_t GroupSyncRead::getData16(uint8_t id, uint8_t address)
{
  if (isAvailable(id, address, 2) == false)
    return 0;
  
  // Calculate offset
  uint8_t offset = address - start_address_;
  
  // Return data
  return SCS_MAKEWORD(data_list_[id][offset], data_list_[id][offset + 1]);
}

uint32_t GroupSyncRead::getData32(uint8_t id, uint8_t address)
{
  if (isAvailable(id, address, 4) == false)
    return 0;
  
  // Calculate offset
  uint8_t offset = address - start_address_;
  
  // Return data
  return ((uint32_t)data_list_[id][offset + 0]) |
         ((uint32_t)data_list_[id][offset + 1] << 8) |
         ((uint32_t)data_list_[id][offset + 2] << 16) |
         ((uint32_t)data_list_[id][offset + 3] << 24);
} 