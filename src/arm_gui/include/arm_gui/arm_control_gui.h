#ifndef ARM_CONTROL_GUI_H
#define ARM_CONTROL_GUI_H

#include <QMainWindow>
#include <QLabel>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QOpenGLWidget>
#include <QTimer>
#include <QTableWidget>
#include <QImage>
#include <QPixmap>
#include <QDateTime>
#include <QScrollBar>
#include <QTextCursor>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace Ui {
class ArmControlMainWindow;
}

class ArmControlGUI : public QMainWindow {
    Q_OBJECT

public:
    explicit ArmControlGUI(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~ArmControlGUI();

private slots:
    // 关节控制槽
    void onJoint1SliderChanged(int value);
    void onJoint2SliderChanged(int value);
    void onJoint3SliderChanged(int value);
    void onJoint4SliderChanged(int value);
    void onJoint6SliderChanged(int value);
    
    void onJoint1SpinChanged(double value);
    void onJoint2SpinChanged(double value);
    void onJoint3SpinChanged(double value);
    void onJoint4SpinChanged(double value);
    void onJoint6SpinChanged(double value);
    
    // 末端执行器控制槽
    void onMoveToPositionClicked();
    void onHomeButtonClicked();
    
    // 吸附控制槽
    void onVacuumPowerSliderChanged(int value);
    void onVacuumOnButtonClicked();
    void onVacuumOffButtonClicked();
    
    // 任务控制槽
    void onPickButtonClicked();
    void onPlaceButtonClicked();
    void onSequenceButtonClicked();
    
    // 菜单操作槽
    void onOpenTaskSequence();
    void onSaveTaskSequence();
    void onExitApplication();
    void onCameraSettings();
    void onRobotSettings();
    void onAbout();
    
    // 检测表格操作槽
    void onDetectionsTableCellClicked(int row, int column);
    
    // 定时器槽（用于GUI更新）
    void onUpdateGUI();

private:
    // UI相关
    Ui::ArmControlMainWindow* ui;
    QTimer* updateTimer;
    
    // ROS相关
    ros::NodeHandle& nh_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber left_camera_sub_;
    ros::Subscriber right_camera_sub_;
    ros::Subscriber depth_image_sub_;
    ros::Subscriber detection_image_sub_;
    ros::Subscriber detection_poses_sub_;
    ros::Publisher joint_command_pub_;
    ros::Publisher vacuum_command_pub_;
    ros::Publisher arm_command_pub_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 状态数据
    std::vector<double> current_joint_values_;
    geometry_msgs::Pose current_end_effector_pose_;
    bool vacuum_on_;
    int vacuum_power_;
    
    // 图像数据
    QImage left_camera_image_;
    QImage right_camera_image_;
    QImage depth_image_;
    QImage detection_image_;
    
    // 检测到的物体
    struct DetectedObject {
        std::string id;
        std::string type;
        double x;
        double y;
        double z;
        geometry_msgs::Pose pose;
    };
    std::vector<DetectedObject> detected_objects_;
    
    // 初始化函数
    void initializeGUI();
    void initializeJointControlConnections();
    void initializeROS();
    void initializeOpenGL();
    
    // ROS回调函数
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void leftCameraCallback(const sensor_msgs::Image::ConstPtr& msg);
    void rightCameraCallback(const sensor_msgs::Image::ConstPtr& msg);
    void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    
    // GUI更新函数
    void updateJointControlWidgets();
    void updateEndEffectorPose();
    void updateVacuumStatus();
    void updateCameraViews();
    void updateDetectionsTable();
    
    // 操作函数
    void sendJointCommand(const std::vector<double>& joint_values);
    void sendVacuumCommand(bool on, int power);
    void sendPickCommand(const std::string& object_id);
    void sendPlaceCommand(double x, double y, double z);
    void sendHomeCommand();
    
    // OpenGL渲染函数
    void renderRobotArm();
    
    // 日志记录
    void logMessage(const QString& message);
    
    // 辅助函数
    QImage cvMatToQImage(const cv::Mat& mat);
    std::vector<double> poseToJoints(const geometry_msgs::Pose& pose); // 逆运动学
    geometry_msgs::Pose jointsToPos(const std::vector<double>& joint_values); // 正运动学
};

#endif // ARM_CONTROL_GUI_H 