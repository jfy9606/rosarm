#ifndef ENHANCED_ARM_GUI_H
#define ENHANCED_ARM_GUI_H

#include <QMainWindow>
#include <QTimer>
#include <QVector3D>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QGroupBox>
#include <QMessageBox>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <servo_wrist/JointControl.h>
#include <servo_wrist/VacuumControl.h>
#include <servo_wrist/HomePosition.h>
#include <stereo_vision/SetViewMode.h>
#include <stereo_vision/DetectionControl.h>

namespace Ui {
class ArmControlMainWindow;
}

class EnhancedArmGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit EnhancedArmGUI(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~EnhancedArmGUI();

protected:
    void closeEvent(QCloseEvent* event) override;
    void timerEvent(QTimerEvent* event) override;

private slots:
    // 关节控制槽函数
    void onJointSliderChanged(int value);
    void onJointSpinChanged(double value);
    
    // 末端控制槽函数
    void onMoveToPositionClicked();
    
    // 真空吸盘控制槽函数
    void onVacuumOnClicked();
    void onVacuumOffClicked();
    void onVacuumPowerChanged(int value);
    
    // 相机视图控制槽函数
    void onLeftViewClicked();
    void onRightViewClicked();
    
    // 其他控制槽函数
    void onHomeButtonClicked();
    
    // 定时更新UI
    void updateUI();

private:
    // 初始化函数
    void initializeROS();
    void setupConnections();
    void setupJointControls();
    void setupCameraParameters();
    
    // ROS回调函数
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void leftImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void rightImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    
    // 控制函数
    void sendJointCommand();
    void sendJointCommand(const std::vector<double>& joint_values);
    void sendVacuumCommand(bool on, int power);
    void sendHomeCommand();
    void setViewMode(int mode);
    
    // 辅助函数
    QImage cvMatToQImage(const cv::Mat& mat);
    void logMessage(const QString& message);
    double degToRad(double deg);
    double radToDeg(double rad);
    
    // UI相关
    Ui::ArmControlMainWindow* ui;
    
    // ROS相关
    ros::NodeHandle& nh_;
    
    // 发布者
    ros::Publisher joint_command_pub_;
    ros::Publisher vacuum_cmd_pub_;
    ros::Publisher vacuum_power_pub_;
    ros::Publisher view_mode_pub_;
    
    // 订阅者
    ros::Subscriber joint_state_sub_;
    ros::Subscriber left_image_sub_;
    ros::Subscriber right_image_sub_;
    
    // 服务客户端
    ros::ServiceClient joint_control_client_;
    ros::ServiceClient vacuum_control_client_;
    ros::ServiceClient home_position_client_;
    ros::ServiceClient set_view_mode_client_;
    
    // 数据存储
    std::vector<double> current_joint_values_;
    geometry_msgs::Pose current_end_pose_;
    bool vacuum_on_;
    int vacuum_power_;
    int current_view_mode_;  // 0=左, 1=右
    
    // 图像存储
    cv::Mat left_image_;
    cv::Mat right_image_;
    QImage current_display_image_;
    
    // 定时器
    int ui_update_timer_id_;
    
    // 标志
    bool ignore_slider_events_;
    bool ignore_spin_events_;
    bool is_connected_;
};

#endif // ENHANCED_ARM_GUI_H 