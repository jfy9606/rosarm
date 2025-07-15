#include "gui/control_panel.hpp"
#include <QGroupBox>
#include <QGridLayout>
#include <QMessageBox>
#include <functional>

namespace gui {

ControlPanel::ControlPanel(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QWidget(parent)
  , node_(node)
{
  // Initialize joint names
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper"};
  
  // Create mock service clients
  joint_control_client_ = std::make_shared<mock::JointControlClient>();
  vacuum_client_ = std::make_shared<mock::VacuumCmdClient>();
  
  // Setup UI
  setupUI();
  
  // Mock services are always available
  RCLCPP_INFO(node_->get_logger(), "Using mock services for control");
}

void ControlPanel::setupUI()
{
  // Create main layout
  auto main_layout = new QVBoxLayout(this);
  
  // Create joint control group
  auto joint_group = new QGroupBox("Joint Control", this);
  auto joint_layout = new QGridLayout(joint_group);
  
  // Create sliders for each joint
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // Create label for joint name
    auto name_label = new QLabel(QString::fromStdString(joint_names_[i]), this);
    
    // Create slider for joint position
    auto slider = new QSlider(Qt::Horizontal, this);
    slider->setRange(-90, 90);
    slider->setValue(0);
    slider->setTickPosition(QSlider::TicksBelow);
    slider->setTickInterval(10);
    
    // Create label for joint value
    auto value_label = new QLabel("0.0", this);
    
    // Connect slider signal
    connect(slider, &QSlider::valueChanged, [this, i](int value) {
      this->onJointSliderChanged(value);
      this->updateJointLabel(i, value);
    });
    
    // Add to layout
    joint_layout->addWidget(name_label, i, 0);
    joint_layout->addWidget(slider, i, 1);
    joint_layout->addWidget(value_label, i, 2);
    
    // Store sliders and labels
    joint_sliders_.push_back(slider);
    joint_value_labels_.push_back(value_label);
  }
  
  // Add joint group to main layout
  main_layout->addWidget(joint_group);
  
  // Create button layout
  auto button_layout = new QHBoxLayout();
  
  // Create home button
  home_button_ = new QPushButton("Home Position", this);
  connect(home_button_, &QPushButton::clicked, this, &ControlPanel::onHomeButtonClicked);
  button_layout->addWidget(home_button_);
  
  // Create gripper button
  gripper_button_ = new QPushButton("Toggle Gripper", this);
  connect(gripper_button_, &QPushButton::clicked, this, &ControlPanel::onGripperButtonClicked);
  button_layout->addWidget(gripper_button_);
  
  // Add button layout to main layout
  main_layout->addLayout(button_layout);
  
  // Set main layout
  setLayout(main_layout);
}

void ControlPanel::updateJointLabel(int joint_index, int value)
{
  if (joint_index >= 0 && joint_index < static_cast<int>(joint_value_labels_.size())) {
    // Convert slider value to degrees
    double degrees = static_cast<double>(value);
    joint_value_labels_[joint_index]->setText(QString::number(degrees, 'f', 1) + "°");
  }
}

void ControlPanel::onJointSliderChanged(int value)
{
  // Find which slider was changed
  QSlider *slider = qobject_cast<QSlider*>(sender());
  if (!slider) return;
  
  // Find the index of the slider
  auto it = std::find(joint_sliders_.begin(), joint_sliders_.end(), slider);
  if (it == joint_sliders_.end()) return;
  
  int joint_index = std::distance(joint_sliders_.begin(), it);
  
  // Convert slider value to radians and send command
  double position = static_cast<double>(value) * M_PI / 180.0; // Convert degrees to radians
  sendJointCommand(joint_index, position);
}

void ControlPanel::sendJointCommand(int joint_index, double position)
{
  if (joint_index < 0 || joint_index >= static_cast<int>(joint_names_.size())) return;
  
  // Create mock request
  auto request = std::make_shared<mock::JointControlRequest>();
  
  // Set positions for all joints, but only change the selected one
  request->position.resize(joint_names_.size(), 0.0);
  request->position[joint_index] = position;
  
  // Send mock request
  auto result_future = joint_control_client_->async_send_request(
    request,
    [this, joint_index](mock::JointControlClient::SharedFuture future) {
      auto result = future.get();
      if (result->success) {
        RCLCPP_INFO(node_->get_logger(), "Joint %d moved successfully: %s", 
                    joint_index, result->message.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to move joint %d: %s", 
                     joint_index, result->message.c_str());
      }
    });
}

void ControlPanel::onHomeButtonClicked()
{
  RCLCPP_INFO(node_->get_logger(), "Moving to home position");
  
  // Set all sliders to 0
  for (auto slider : joint_sliders_) {
    slider->setValue(0);
  }
  
  // Home position is handled by setting all sliders to 0
}

void ControlPanel::onGripperButtonClicked()
{
  RCLCPP_INFO(node_->get_logger(), "Toggling gripper");
  
  // Create mock request
  auto request = std::make_shared<mock::VacuumCmdRequest>();
  request->enable = true; // Toggle based on current state
  
  // Send mock request
  auto result_future = vacuum_client_->async_send_request(
    request,
    [this](mock::VacuumCmdClient::SharedFuture future) {
      auto result = future.get();
      if (result->success) {
        RCLCPP_INFO(node_->get_logger(), "Gripper toggled successfully");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to toggle gripper: %s", 
                     result->message.c_str());
      }
    });
}

} // namespace gui 