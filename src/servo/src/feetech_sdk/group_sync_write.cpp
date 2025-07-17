#include "servo/feetech_sdk/group_sync_write.h"
#include <algorithm>
#include <iostream>
#include <cstring>

#define SCS_INST_SYNC_WRITE     0x83

GroupSyncWrite::GroupSyncWrite(PortHandler* port, PacketHandler* ph, uint8_t start_address, uint8_t data_length)
  : port_(port), ph_(ph), start_address_(start_address), data_length_(data_length),
    is_param_changed_(false), param_(nullptr), param_length_(0)
{
  clearParam();
}

GroupSyncWrite::~GroupSyncWrite()
{
  clearParam();
}

bool GroupSyncWrite::addParam(uint8_t id, const uint8_t* data)
{
  // If ID already exists, return false
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())
    return false;
  
  // Add ID to the list
  id_list_.push_back(id);
  
  // Create data buffer for this ID
  data_list_[id] = new uint8_t[data_length_];
  
  // Copy data
  memcpy(data_list_[id], data, data_length_);
  
  is_param_changed_ = true;
  return true;
}

void GroupSyncWrite::removeParam(uint8_t id)
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

void GroupSyncWrite::clearParam()
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

int GroupSyncWrite::txPacket()
{
  if (id_list_.empty())
    return COMM_NOT_AVAILABLE;
  
  if (is_param_changed_ == true || param_ == nullptr) {
    if (param_ != nullptr)
      delete[] param_;
    
    // Calculate parameter length
    param_length_ = (data_length_ + 1) * id_list_.size() + 2;  // (data_length + ID) * number of IDs + start address + data length
    
    // Allocate parameter buffer
    param_ = new uint8_t[param_length_];
    
    // Fill parameter buffer
    param_[0] = start_address_;
    param_[1] = data_length_;
    
    int idx = 2;
    for (auto const& id : id_list_) {
      param_[idx++] = id;
      memcpy(&param_[idx], data_list_[id], data_length_);
      idx += data_length_;
    }
    
    is_param_changed_ = false;
  }
  
  return ph_->syncWriteTxOnly(port_, param_, param_length_);
}

bool GroupSyncWrite::changeParam(uint8_t id, const uint8_t* data)
{
  auto it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end()) {
    return false; // ID doesn't exist
  }
  
  // Update data
  for (size_t i = 0; i < data_length_; i++) {
    data_list_[id][i] = data[i];
  }
  
  is_param_changed_ = true;
  return true;
} 