#ifndef GUI_MAIN_WINDOW_HPP
#define GUI_MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace Ui {
class MainWindow;
}

namespace gui {

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
  ~MainWindow();

private slots:
  void updateRobotStatus();
  void onHomeButtonClicked();
  void onMoveButtonClicked();
  void onGripperButtonClicked();

private:
  Ui::MainWindow *ui;
  rclcpp::Node::SharedPtr node_;
  QTimer *update_timer_;
  
  // ROS publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  
  // Callbacks
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

} // namespace gui

#endif // GUI_MAIN_WINDOW_HPP 