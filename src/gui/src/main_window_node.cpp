#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "gui/main_window.hpp"

int main(int argc, char *argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gui_node");
  
  // Initialize Qt
  QApplication app(argc, argv);
  app.setApplicationName("Robotic Arm GUI");
  
  // Create and show main window
  gui::MainWindow w(node);
  w.show();
  
  // Create a thread for ROS spinning
  std::thread ros_thread([&node]() {
    rclcpp::spin(node);
  });
  
  // Run Qt application
  int result = app.exec();
  
  // Cleanup
  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }
  
  return result;
} 