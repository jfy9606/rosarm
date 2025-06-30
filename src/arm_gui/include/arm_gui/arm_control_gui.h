#ifndef ARM_CONTROL_GUI_H
#define ARM_CONTROL_GUI_H

#include <QMainWindow>
#include <QLabel>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QTimer>
#include <QTableWidget>
#include <QImage>
#include <QPixmap>
#include <QDateTime>
#include <QScrollBar>
#include <QTextCursor>
#include <QCheckBox>
#include <QComboBox>
#include <QMatrix4x4>
#include <QMatrix3x3>
#include <QVector3D>
#include <QQuaternion>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <cv_bridge/cv_bridge.h>
#include <servo_wrist/SerControl.h>
#include <std_srvs/SetBool.h>

// 添加消息过滤器头文件
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace Ui {
class ArmControlMainWindow;
}

// 自定义OpenGL渲染器类
class Scene3DRenderer : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
    
public:
    explicit Scene3DRenderer(QWidget* parent = nullptr);
    ~Scene3DRenderer() override;
    
    // 更新场景中的物体
    void updateObjects(const std::vector<std::pair<QVector3D, QColor>>& objects);
    
    // 设置选中的物体
    void setSelectedObject(int index);
    
    // 获取选中的物体索引
    int getSelectedObject() const;
    
    // 添加机械臂模型
    void setRobotPose(const std::vector<double>& joint_values);
    
signals:
    void objectSelected(int index);
    
protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    
private:
    // 场景中的物体 (位置, 颜色)
    std::vector<std::pair<QVector3D, QColor>> objects_;
    
    // 选中的物体索引
    int selected_object_;
    
    // 相机参数
    QVector3D camera_position_;
    QVector3D camera_target_;
    QVector3D camera_up_;
    
    // 上一次鼠标位置
    QPoint last_mouse_pos_;
    
    // 视角旋转角度
    float yaw_;
    float pitch_;
    
    // 缩放因子
    float zoom_;
    
    // 机械臂关节值
    std::vector<double> robot_joints_;
    
    // 渲染函数
    void renderObjects();
    void renderRobot();
    void renderCoordinateAxes();
    
    // 根据屏幕坐标计算3D空间中的射线
    QVector3D screenToRay(int x, int y);
    
    // 判断射线与物体相交
    bool rayIntersectsSphere(const QVector3D& ray_origin, const QVector3D& ray_dir, 
                            const QVector3D& sphere_center, float sphere_radius);
};

// 机械臂控制模式枚举
enum class ArmControlMode {
    JOINT_CONTROL,    // 关节控制模式
    CARTESIAN_CONTROL, // 笛卡尔空间控制模式
    VISUAL_SERVO      // 视觉伺服控制模式
};

// 检测到的物体信息结构
struct DetectedObject {
    std::string id;
    std::string type;
    double x;
    double y;
    double z;
    geometry_msgs::Pose pose;
};

class ArmControlGUI : public QMainWindow {
    Q_OBJECT

public:
    explicit ArmControlGUI(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~ArmControlGUI();

protected:
    // 添加事件过滤器
    bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
    // 按钮点击处理
    void on_homeButton_clicked();
    void on_moveToPositionButton_clicked();
    void on_vacuumOnButton_clicked();
    void on_vacuumOffButton_clicked();
    
    // 更新UI
    void updateUI();
    
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
    
    // 视觉控制槽
    void onPickButtonClicked();
    void onPlaceButtonClicked();
    
    // 菜单操作槽
    void onOpenTaskSequence();
    void onSaveTaskSequence();
    void onExitApplication();
    void onRobotSettings();
    void onAbout();
    
    // 检测表格操作槽
    void onDetectionsTableCellClicked(int row, int column);
    
    // 定时器槽
    void onUpdateGUI();
    
    // 相机视图更新
    void updateCameraView();
    
    // 3D视图相关槽
    void on3DViewObjectSelected(int index);
    void updateScene3D();
    
    // 摄像头图像鼠标事件处理
    void onCameraViewClicked(QPoint pos);
    
    // 摄像头视图切换
    void onCameraSwitchButtonClicked();
    
    // 将这两个函数移动到槽中
    void updateCameraViews();
    void updateDetectionsTable();
    
    // 添加这些函数到private slots中
    void updateJointControlWidgets();
    void updateEndEffectorPose();

private:
    // UI相关
    Ui::ArmControlMainWindow* ui;
    QTimer* updateTimer;
    bool ui_processing_ = false;
    
    // 控制模式
    ArmControlMode current_control_mode_;
    
    // ROS相关
    ros::NodeHandle& nh_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber stereo_merged_sub_;
    ros::Subscriber depth_image_sub_;     // 添加深度图像订阅
    ros::Subscriber detection_image_sub_;
    ros::Subscriber detection_poses_sub_;
    ros::Subscriber yolo_status_sub_;
    ros::ServiceClient yolo_control_client_;
    
    // 额外ROS相关
    ros::Publisher joint_command_pub_;
    ros::Publisher arm_command_pub_;
    ros::Publisher relay_order_pub_;
    ros::Publisher vacuum_cmd_pub_;       // 吸盘开关控制
    ros::Publisher vacuum_power_pub_;     // 吸盘功率控制
    ros::Publisher camera_view_mode_pub_; // 用于发布摄像头视图模式
    
    // 消息过滤器同步器
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseArray> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image>* detection_image_sub_filter_;
    message_filters::Subscriber<geometry_msgs::PoseArray>* detection_poses_sub_filter_;
    message_filters::Synchronizer<SyncPolicy>* object_detection_sync_;
    
    // 机械臂状态
    std::vector<double> current_joint_values_;
    std::vector<double> target_joint_values_;
    std::vector<double> joint_min_values_;
    std::vector<double> joint_max_values_;
    QVector3D current_end_position_;
    QQuaternion current_end_orientation_;
    
    // 摄像头视图状态
    int camera_view_mode_ = 0; // 0=左图，1=右图，2=深度图
    geometry_msgs::Pose current_end_pose_;
    bool vacuum_on_;
    int vacuum_power_;
    
    // 摄像头相关
    QImage current_camera_image_;       // 当前相机图像
    QImage current_depth_image_;        // 当前深度图像
    QImage left_camera_image_;          // 左摄像头图像
    QImage right_camera_image_;         // 右摄像头图像
    bool is_camera_available_ = false;  // 摄像头是否可用
    int stereo_camera_error_count_ = 0; // 摄像头错误计数
    int available_camera_index_ = 0;    // 可用的摄像头索引
    QTimer camera_reconnect_timer_;     // 摄像头重连定时器
    
    // 视觉相关
    std::vector<DetectedObject> detected_objects_; // 检测到的物体列表
    int selected_object_index_ = -1;    // 选中的物体索引
    bool visual_servo_active_ = false;  // 视觉伺服是否激活
    
    // 相机参数
    QMatrix4x4 camera_intrinsic_;       // 相机内参矩阵
    QMatrix4x4 camera_extrinsic_;       // 相机外参矩阵
    QMatrix4x4 camera_projection_;      // 相机投影矩阵
    QMatrix4x4 camera_transform_;       // 相机变换矩阵
    float camera_focal_length_ = 500.0f;// 相机焦距
    float camera_cx_ = 320.0f;          // 相机光心x坐标
    float camera_cy_ = 240.0f;          // 相机光心y坐标
    
    // 3D视图相关
    Scene3DRenderer* scene_3d_renderer_ = nullptr; // 3D场景渲染器

    // 3D场景中的物体
    std::vector<std::pair<QVector3D, QColor>> scene_objects_;

    // ROS回调函数
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void stereoMergedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void yoloStatusCallback(const std_msgs::Bool::ConstPtr& msg);
    void objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg,
                              const geometry_msgs::PoseArray::ConstPtr& poses_msg);
    void topicCallback_pose(const geometry_msgs::Pose& pose);

    // 机械臂控制函数
    void sendJointCommand(const std::vector<double>& positions);
    void sendVacuumCommand(bool on, int power = 100);
    void sendHomeCommand();
    void sendPickCommand(const std::string& object_id);
    void sendPlaceCommand(double x, double y, double z);
    void sendPickObjectCommand(int object_index);

    // 辅助函数
    void logMessage(const QString& message);
    void updateEndEffectorPosition(double x, double y, double z);
    QImage cvMatToQImage(const cv::Mat& mat);
    
    // 初始化函数
    void initializeGUI();
    void initializeJointControlConnections();
    void initializeROS();
    void initializeOpenGL();
    void initializeMembers();
    void setupROSSubscriptions();
    void setupCameraParameters();
    void setupDHParameters();
    void setupJointLimits();
    void setupUi();
    void createMenus();
    void connectSignalSlots();

    // 更新函数
    void updateVacuumStatus();
    void updateJointInfo();
    void updateConnectionStatus();
    void updateGUIJointValues();
    void updateSceneObjects();

    // 工具函数
    QVector3D imagePointTo3D(const QPoint& image_point, float depth = 1.0);
    QPoint point3DToImage(const QVector3D& point_3d);
    float getDepthAtPoint(const QPoint& image_point);
    void updateCameraTransform(const geometry_msgs::Pose& end_effector_pose);
    int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);

    // 运动学函数
    std::vector<double> poseToJoints(const geometry_msgs::Pose& pose);
    geometry_msgs::Pose jointsToPos(const std::vector<double>& joint_values);
    
    // DH参数和关节限制
    std::vector<std::tuple<int, double, double, double, double>> dh_params_; // [type, d, theta, a, alpha]
    std::vector<std::pair<double, double>> joint_limits_;
    
    // 添加正向和逆向运动学计算函数
    geometry_msgs::Pose forwardKinematics(const std::vector<double>& joint_values);
    std::vector<double> inverseKinematics(const geometry_msgs::Pose& target_pose, const std::vector<double>& initial_guess = {});
    
    // 计算单个关节的DH变换矩阵
    QMatrix4x4 computeDHTransform(double theta, double d, double a, double alpha);
    
    // 检查关节是否在限制范围内
    bool checkJointLimits(const std::vector<double>& joint_values);
    
    // 中继控制命令
    void sendRelayOrder(const std::string& command);

    // 添加摄像头错误处理函数
    void handleCameraError(const std::string& error_msg);
    void createPlaceholderImage();
    void attemptCameraReconnect();
    bool findAvailableCamera();
};

#endif // ARM_CONTROL_GUI_H 