#ifndef GUI_CONTROL_PANEL_HPP
#define GUI_CONTROL_PANEL_HPP

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include "gui/mock_services.hpp"

namespace gui {

class ControlPanel : public QWidget {
  Q_OBJECT

public:
  explicit ControlPanel(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
  ~ControlPanel() = default;

private slots:
  void onJointSliderChanged(int value);
  void onHomeButtonClicked();
  void onGripperButtonClicked();

private:
  rclcpp::Node::SharedPtr node_;
  
  // ROS clients
  mock::JointControlClient::SharedPtr joint_control_client_;
  mock::VacuumCmdClient::SharedPtr vacuum_client_;
  
  // UI elements
  std::vector<QSlider*> joint_sliders_;
  std::vector<QLabel*> joint_value_labels_;
  QPushButton *home_button_;
  QPushButton *gripper_button_;
  
  // Joint names
  std::vector<std::string> joint_names_;
  
  // Methods
  void setupUI();
  void updateJointLabel(int joint_index, int value);
  void sendJointCommand(int joint_index, double position);
};

} // namespace gui

#endif // GUI_CONTROL_PANEL_HPP 