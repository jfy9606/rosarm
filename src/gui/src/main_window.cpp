#include "gui/main_window.hpp"
// #include "ui_main_window.h" // This would be generated from a .ui file

#include <QMessageBox>
#include <QDebug>
#include <functional>

namespace gui {

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QMainWindow(parent)
  , ui(nullptr) // Replace with ui(new Ui::MainWindow) when UI file is available
  , node_(node)
{
  // ui->setupUi(this); // Uncomment when UI file is available
  
  // Setup window properties
  setWindowTitle("ROS 2 Robotic Arm Control");
  resize(800, 600);
  
  // Create a simple UI programmatically (replace with .ui file when available)
  // This is just a placeholder
  
  // Setup ROS publishers and subscribers
  command_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "arm_command", 10);
    
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, 
    std::bind(&MainWindow::jointStateCallback, this, std::placeholders::_1));
  
  // Setup timer for periodic updates
  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateRobotStatus);
  update_timer_->start(100); // 10 Hz
  
  RCLCPP_INFO(node_->get_logger(), "GUI Main Window initialized");
}

MainWindow::~MainWindow()
{
  if (ui) {
    delete ui;
  }
}

void MainWindow::updateRobotStatus()
{
  // Update UI with latest robot status
  // This is a placeholder
}

void MainWindow::onHomeButtonClicked()
{
  RCLCPP_INFO(node_->get_logger(), "Home button clicked");
  
  std_msgs::msg::String msg;
  msg.data = "home";
  command_pub_->publish(msg);
}

void MainWindow::onMoveButtonClicked()
{
  RCLCPP_INFO(node_->get_logger(), "Move button clicked");
  
  std_msgs::msg::String msg;
  msg.data = "move";
  command_pub_->publish(msg);
}

void MainWindow::onGripperButtonClicked()
{
  RCLCPP_INFO(node_->get_logger(), "Gripper button clicked");
  
  std_msgs::msg::String msg;
  msg.data = "gripper";
  command_pub_->publish(msg);
}

void MainWindow::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Process joint state updates
  // This is a placeholder
}

} // namespace gui 