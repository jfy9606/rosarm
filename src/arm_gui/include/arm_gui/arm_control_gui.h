#ifndef ARM_CONTROL_GUI_H
#define ARM_CONTROL_GUI_H

#include <QMainWindow>
#include <QPushButton>
#include <QImage>
#include <QTimer>
#include <QVector3D>
#include <QMatrix4x4>
#include <QColor>
#include <QPoint>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QDateTime>
#include <QQuaternion>
#include <QHeaderView>
#include <QTextCursor>
#include <QRect>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>
#include <string>
#include <tuple>

// Service client includes
#include <arm_trajectory/ForwardKinematics.h>
#include <arm_trajectory/InverseKinematics.h>
#include <servo_wrist/JointControl.h>
#include <servo_wrist/VacuumControl.h>
#include <servo_wrist/HomePosition.h>
#include <stereo_vision/DetectionControl.h>
#include <stereo_vision/SetViewMode.h>
#include <std_srvs/SetBool.h>

#include "arm_gui/scene_3d_renderer.h"

namespace Ui {
class ArmControlMainWindow;
}

// DH参数结构体
struct DHParam {
    double theta;  // 关节角
    double d;      // 连杆偏移
    double a;      // 连杆长度
    double alpha;  // 连杆扭角
    int joint_type; // 关节类型：0-旋转，1-移动
    
    DHParam(double t, double offset, double length, double twist, int type = 0)
        : theta(t), d(offset), a(length), alpha(twist), joint_type(type) {}
        
    DHParam() : theta(0), d(0), a(0), alpha(0), joint_type(0) {}
};

// 机械臂控制GUI
class ArmControlGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit ArmControlGUI(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~ArmControlGUI();

protected:
    void closeEvent(QCloseEvent* event) override;
    bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
    // Joint control slots
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
    
    // Cartesian control
    void onMoveToPositionClicked();
    void on_moveToPositionButton_clicked();
    
    // Home position
    void onHomeButtonClicked();
    void on_homeButton_clicked();
    
    // Vacuum control slots
    void onVacuumPowerSliderChanged(int value);
    void onVacuumOnButtonClicked();
    void onVacuumOffButtonClicked();
    void on_vacuumOnButton_clicked();
    void on_vacuumOffButton_clicked();
    
    // Task control
    void onPickButtonClicked();
    void onPlaceButtonClicked();
    
    // Menu actions
    void onOpenTaskSequence();
    void onSaveTaskSequence();
    void onExitApplication();
    void onRobotSettings();
    void onAbout();
    
    // Detection table
    void onDetectionsTableCellClicked(int row, int column);
    
    // Camera control slots
    void onCameraSwitchButtonClicked();
    void onLeftViewButtonClicked();  // 切换到左视图
    void onRightViewButtonClicked(); // 切换到右视图
    void onDepthViewButtonClicked(); // 切换到深度视图
    void on_leftViewButton_clicked();
    void on_rightViewButton_clicked();
    void on_depthViewButton_clicked();
    
    // 3D view
    void on3DViewObjectSelected(int index);
    
    // Timer update
    void onUpdateGUI();
    
    // UI update functions
    void updateJointInfo();
    void updateEndEffectorPose();
    void updateVacuumStatus();
    void updateCameraViews();
    void updateCameraView();
    void updateUI();
    void updateGUIJointValues();
    void updateDetectionsTable();
    void updateConnectionStatus();
    void updateJointControlWidgets();
    
    // Navigation slots
    void goToHomePosition();

private:
    // Initialize functions
    void initializeGUI();
    void initializeMembers();
    void createMissingUIElements();  // 添加缺失UI元素的函数
    void initializeROS();
    void initializeOpenGL();
    void initializeJointControlConnections();
    void setupROSSubscriptions();
    void setupUi();
    void createMenus();
    void connectSignalSlots();
    void setupCameraParameters();
    void setupJointLimits();
    void setupDHParameters();
    
    // 辅助数学函数
    double degToRad(double deg);
    double radToDeg(double rad);
    QMatrix4x4 computeDHTransform(double theta, double d, double a, double alpha);
    
    // Kinematics services from arm_trajectory package
    bool checkJointLimits(const std::vector<double>& joint_values);
    
    // Call kinematics service for forward kinematics
    geometry_msgs::Pose forwardKinematics(const std::vector<double>& joint_values);
    
    // Call kinematics service for inverse kinematics
    std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                         const std::vector<double>& initial_guess = {});
    
    // Control functions - now using service calls
    void sendJointCommand(const std::vector<double>& joint_values);
    void sendJointCommand();
    void sendVacuumCommand(bool on, int power);
    void sendHomeCommand();
    void sendRelayOrder(const std::string& command);
    void sendPickCommand(const std::string& object_id);
    void sendPlaceCommand(double x, double y, double z);
    void sendPickObjectCommand(int object_index);
    void enableObjectDetection(bool enable);
    
    // Camera functions
    void updateCameraTransform(const geometry_msgs::Pose& end_effector_pose);
    QVector3D imagePointTo3D(const QPoint& image_point, float depth);
    QPoint point3DToImage(const QVector3D& point_3d);
    float getDepthAtPoint(const QPoint& image_point);
    void onCameraViewClicked(QPoint pos);
    void handleCameraError(const std::string& error_msg);
    void createPlaceholderImage(const std::string& message = "");
    void attemptCameraReconnect();
    bool findAvailableCamera();
    
    // Helper functions
    QImage cvMatToQImage(const cv::Mat& mat);
    // Helper functions that use arm_trajectory services
    std::vector<double> poseToJoints(const geometry_msgs::Pose& pose);
    geometry_msgs::Pose jointsToPos(const std::vector<double>& joint_values);
    void logMessage(const QString& message);
    int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);
    void updateEndEffectorPosition(double x, double y, double z);
    
    // ROS Callbacks
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void stereoMergedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void yoloStatusCallback(const std_msgs::Bool::ConstPtr& msg);
    
    void objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg, 
                               const geometry_msgs::PoseArray::ConstPtr& poses_msg);
    
    // 3D scene functions
    void updateScene3D();
    void updateSceneObjects();
    void onEndEffectorDragged(QVector3D position);
    
    // Data structures
    struct DetectedObject {
        std::string id;
        std::string type;
        std::string label;       // 用于显示的标签
        double confidence;       // 置信度
        QVector3D position;      // 3D位置
        QVector3D dimensions;    // 3D尺寸
        geometry_msgs::Pose pose;
        double x, y, z;  // Position in cm
    };

private:
    Ui::ArmControlMainWindow* ui;
    
    // ROS members
    ros::NodeHandle& nh_;
    
    // Publishers
    ros::Publisher joint_command_pub_;
    ros::Publisher vacuum_cmd_pub_;
    ros::Publisher vacuum_power_pub_;
    ros::Publisher arm_command_pub_;
    ros::Publisher home_command_pub_;
    ros::Publisher relay_order_pub_;
    ros::Publisher camera_view_mode_pub_;
    static ros::Publisher view_mode_pub_;
    
    // Subscribers
    ros::Subscriber joint_state_sub_;
    ros::Subscriber stereo_merged_sub_;
    ros::Subscriber depth_image_sub_;
    ros::Subscriber detection_image_sub_;
    ros::Subscriber detection_poses_sub_;
    ros::Subscriber yolo_status_sub_;
    
    // Service clients
    ros::ServiceClient fk_client_;
    ros::ServiceClient ik_client_;
    ros::ServiceClient joint_control_client_;
    ros::ServiceClient vacuum_control_client_;
    ros::ServiceClient home_position_client_;
    ros::ServiceClient detection_control_client_;
    ros::ServiceClient set_view_mode_client_;
    ros::ServiceClient yolo_control_client_;
    
    // Message filters for synchronized topics
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, geometry_msgs::PoseArray> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image>* detection_image_sub_filter_;
    message_filters::Subscriber<geometry_msgs::PoseArray>* detection_poses_sub_filter_;
    message_filters::Synchronizer<SyncPolicy>* object_detection_sync_;
    
    // 关节限制和DH参数
    std::vector<std::pair<double, double>> joint_limits_;
    std::vector<DHParam> dh_params_;
    
    // Joint state
    std::vector<double> current_joint_values_;
    
    // End effector state
    QVector3D current_end_position_;
    QQuaternion current_end_orientation_;
    geometry_msgs::Pose current_end_pose_;
    
    // Vacuum state
    bool vacuum_on_;
    int vacuum_power_;
    
    // Object detection state
    std::vector<DetectedObject> detected_objects_;
    int selected_object_index_;
    std::string selected_object_id_;
    bool yolo_enabled_;
    
    // Images
    cv::Mat current_image_;
    QImage left_camera_image_;
    QImage right_camera_image_;
    QImage current_camera_image_;
    QImage current_depth_image_;
    QImage detection_image_;
    
    // Camera state
    QMatrix4x4 camera_intrinsic_;
    QMatrix4x4 camera_extrinsic_;
    int camera_view_mode_;
    bool is_camera_available_;
    int stereo_camera_error_count_;
    int available_camera_index_;
    ros::Time last_detection_time_;
    
    // UI state
    bool ignore_slider_events_;
    bool ignore_spin_events_;
    bool ui_processing_;
    bool arm_ready_;
    
    // 3D view
    Scene3DRenderer* scene_3d_renderer_;
    std::vector<std::pair<QVector3D, QColor>> scene_objects_;
    
    // Timer
    QTimer* updateTimer;
    QTimer camera_reconnect_timer_;
};

#endif // ARM_CONTROL_GUI_H 