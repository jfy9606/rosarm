#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QWidget>
#include <QTimer>
#include <QGroupBox>
#include <QStatusBar>
#include <QFrame>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QTabWidget>
#include <QScrollArea>
#include <QTextEdit>
#include <QDir>
#include <QScrollBar>
#include <QStandardItemModel>
#include <QSplitter>
#include <QTime>
#include <QColorDialog>
#include <QShortcut>
#include <QInputDialog>
#include <QSettings>
#include <QVector3D>
#include <QQuaternion>
#include <QImage>
#include <QPainter>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// 添加服务头文件
#include <servo_wrist/JointControl.h>
#include <servo_wrist/VacuumCmd.h>
#include <servo_wrist/HomePosition.h>

// 前向声明
namespace Ui {
    class MainWindow;
}

namespace arm_trajectory {
    class KinematicsControl;
}

// 检测到的物体结构
struct DetectedObject {
    std::string id;
    std::string type;
    std::string label;
    float confidence;
    geometry_msgs::Pose pose;
    QVector3D position;
    QVector3D dimensions;
    double z = 0.0;  // 距离，单位厘米
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~MainWindow();

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;
    void closeEvent(QCloseEvent* event) override;

private slots:
    // UI更新槽函数
    void updateUI();
    void updateGUIJointValues();
    void updateCameraView();
    void updateEndEffectorPose();
    void updateVacuumStatus();
    
    // 关节控制槽函数
    void onJoint1SliderChanged(int value);
    void onJoint2SliderChanged(int value);
    void onJoint3SliderChanged(int value);
    void onJoint4SliderChanged(int value);
    void onJoint5SliderChanged(int value);
    void onJoint6SliderChanged(int value);
    
    void onJoint1SpinChanged(double value);
    void onJoint2SpinChanged(double value);
    void onJoint3SpinChanged(double value);
    void onJoint4SpinChanged(double value);
    void onJoint5SpinChanged(double value);
    void onJoint6SpinChanged(double value);
    
    // 真空吸盘控制槽函数
    void onVacuumOnButtonClicked();
    void onVacuumOffButtonClicked();
    void onVacuumPowerSliderChanged(int value);
    
    // 相机视图控制槽函数
    void onLeftViewButtonClicked();
    void onRightViewButtonClicked();
    
    // 位置控制槽函数
    void onMoveToPositionClicked();
    void onHomeButtonClicked();
    
    // 相机重连槽函数
    void attemptCameraReconnect();

private:
    // UI创建函数
    QWidget* createArmControlPanel();
    QWidget* createJointControlPanel();
    QWidget* createPositionControlPanel();
    QWidget* createCameraViewPanel();
    QWidget* createObjectDetectionPanel();
    QWidget* createDepthVisualizationPanel();
    QWidget* createVisualServoBridgePanel();
    
    // 初始化函数
    void initializeROS();
    void initializeMembers();
    void setupUi();
    void connectSignalSlots();
    void setupROSSubscriptions();
    void setupJointLimits();
    
    // ROS回调函数
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void stereoMergedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg, 
                              const geometry_msgs::PoseArray::ConstPtr& poses_msg);
    
    // 辅助函数
    void sendJointCommand();
    void sendJointCommand(const std::vector<double>& joint_values);
    void sendVacuumCommand(bool on, int power);
    void sendHomeCommand();
    void goToHomePosition();
    void logMessage(const QString& message);
    
    // 图像处理函数
    void createPlaceholderImage(const std::string& message = "");
    QImage cvMatToQImage(const cv::Mat& mat);
    cv::Mat qImageToCvMat(const QImage& image);
    cv::Mat createColoredDepthMap(const cv::Mat& depth_map);
    void drawDetectionBoxes(QImage& image, const std::vector<DetectedObject>& objects);
    void showObjectDistanceOverlay(QImage& image, const std::vector<DetectedObject>& objects);
    void estimateObjectDistances(const std::vector<DetectedObject>& objects, const cv::Mat& depth_map);
    
    // 运动学计算函数
    double degToRad(double deg);
    double radToDeg(double rad);
    geometry_msgs::Pose forwardKinematics(const std::vector<double>& joint_values);
    std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                        const std::vector<double>& initial_guess);
    bool checkJointLimits(const std::vector<double>& joint_values);
    bool findAvailableCamera();
    
    // 成员变量
    Ui::MainWindow* ui;
    ros::NodeHandle& nh_;
    QTimer* updateTimer;
    QTimer camera_reconnect_timer_;
    
    // ROS发布者
    ros::Publisher joint_command_pub_;
    ros::Publisher vacuum_cmd_pub_;
    ros::Publisher vacuum_power_pub_;
    ros::Publisher camera_view_mode_pub_;
    
    // ROS订阅者
    ros::Subscriber joint_state_sub_;
    ros::Subscriber stereo_merged_sub_;
    ros::Subscriber depth_image_sub_;
    
    // 同步订阅
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, geometry_msgs::PoseArray> SyncPolicy;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> detection_image_sync_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseArray>> detection_poses_sync_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // ROS服务客户端
    ros::ServiceClient joint_control_client_;
    ros::ServiceClient vacuum_control_client_;
    
    // 关节控制部件
    QSlider* joint1Slider;
    QSlider* joint2Slider;
    QSlider* joint3Slider;
    QSlider* joint4Slider;
    QSlider* joint5Slider;
    QSlider* joint6Slider;
    
    QDoubleSpinBox* joint1Spin;
    QDoubleSpinBox* joint2Spin;
    QDoubleSpinBox* joint3Spin;
    QDoubleSpinBox* joint4Spin;
    QDoubleSpinBox* joint5Spin;
    QDoubleSpinBox* joint6Spin;
    
    // 末端执行器控制部件
    QDoubleSpinBox* posXSpin;
    QDoubleSpinBox* posYSpin;
    QDoubleSpinBox* posZSpin;
    QPushButton* moveToPositionButton;
    QLabel* endEffectorStatusLabel;
    QLabel* jointStatusLabel;
    
    // 真空吸盘控制部件
    QPushButton* vacuumOnButton;
    QPushButton* vacuumOffButton;
    QSlider* vacuumPowerSlider;
    
    // 相机视图部件
    QLabel* cameraView;
    QPushButton* leftViewButton;
    QPushButton* rightViewButton;
    
    // 其他控制部件
    QPushButton* homeButton;
    
    // 状态变量
    std::vector<double> current_joint_values_;
    bool vacuum_on_;
    int vacuum_power_;
    int camera_view_mode_;
    bool is_camera_available_;
    int stereo_camera_error_count_;
    int available_camera_index_;
    bool depth_available_;
    bool yolo_enabled_;
    bool show_detection_boxes_;
    bool show_distance_overlay_;
    std::string current_detection_model_;
    bool arm_ready_;
    int selected_object_index_;
    bool ignore_slider_events_;
    bool ignore_spin_events_;
    bool ui_processing_;
    
    // 图像数据
    QImage current_camera_image_;
    QImage left_camera_image_;
    QImage right_camera_image_;
    QImage current_depth_image_;
    QImage detection_image_;
    cv::Mat current_image_;
    cv::Mat current_depth_map_;
    
    // 位置数据
    QVector3D current_end_position_;
    QQuaternion current_end_orientation_;
    geometry_msgs::Pose current_end_pose_;
    
    // 检测结果
    std::vector<DetectedObject> detected_objects_;
    ros::Time last_detection_time_;
    
    // 运动学工具
    arm_trajectory::KinematicsControl* kinematics_utils_;
};

#endif // MAIN_WINDOW_H 