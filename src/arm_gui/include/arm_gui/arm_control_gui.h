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
#include "liancheng_socket/MotorOrder.h"
#include <servo_wrist/SerControl.h>
#include <std_srvs/SetBool.h>

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
    void on_stopButton_clicked();
    void on_moveButton_clicked();
    void on_gripperOpenButton_clicked();
    void on_gripperCloseButton_clicked();
    void on_vacuumOnButton_clicked();
    void on_vacuumOffButton_clicked();
    void on_example1Button_clicked();
    void on_example2Button_clicked();
    void on_example3Button_clicked();
    
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
    
    // 定时器槽
    void onUpdateGUI();
    
    // 示例动作函数
    void onDemoAction1Clicked();
    void onDemoAction2Clicked();
    void onDemoAction3Clicked();
    
    // 相机视图更新
    void updateCameraView();
    
    // ROS更新
    void updateROS();

    // YOLO控制
    void onYoloDetectionToggled(bool checked);

    // 路径规划相关槽
    void onScanObjectsClicked();
    void onPlanPathClicked();
    void onExecutePathClicked();
    void onVisualizeWorkspaceClicked();

    // 更新函数
    void updateJointInfo();
    void updateEndEffectorPose();
    void updateCameraViews();
    void updateConnectionStatus();

    // 添加新的槽函数
    void updateDetectionsTableUI();

    // 3D视图相关槽
    void on3DViewObjectSelected(int index);
    void updateScene3D();
    
    // 摄像头图像鼠标事件处理
    void onCameraViewClicked(QPoint pos);

private:
    // UI相关
    Ui::ArmControlMainWindow* ui;
    QTimer* updateTimer;
    QCheckBox* yolo_checkbox_;
    QComboBox* placement_area_combo_;
    
    // ROS相关
    ros::NodeHandle& nh_;
    ros::Publisher joint_cmd_pub_;
    ros::Publisher gripper_cmd_pub_;
    ros::Publisher vacuum_cmd_pub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber stereo_merged_sub_;
    ros::Subscriber detection_image_sub_;
    ros::Subscriber detection_poses_sub_;
    ros::Subscriber yolo_status_sub_;
    ros::ServiceClient yolo_control_client_;
    
    // 额外ROS相关
    ros::Publisher joint_command_pub_;
    ros::Publisher vacuum_command_pub_;
    ros::Publisher arm_command_pub_;
    ros::Publisher motor_order_pub_;
    ros::Publisher relay_order_pub_;
    ros::Publisher servo_control_pub_;
    
    // 图像显示相关
    QImage left_camera_image_;
    QImage detection_image_;
    QPixmap camera_pixmap_;
    QImage current_camera_image_;
    
    // 机械臂状态
    std::vector<double> current_joint_positions_;
    std::vector<double> current_joint_values_;
    geometry_msgs::Pose current_end_effector_pose_;
    bool gripper_open_;
    bool vacuum_on_;
    int vacuum_power_;
    bool yolo_detection_enabled_;
    
    // 检测对象结构
    struct DetectedObject {
        std::string id;
        std::string type;
        double x;
        double y;
        double z;
        geometry_msgs::Pose pose;
    };
    std::vector<DetectedObject> detected_objects_;
    
    // 回调函数
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void leftCameraCallback(const sensor_msgs::Image::ConstPtr& msg);
    void detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
    void detectionCallback(const sensor_msgs::Image::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void stereoMergedCallback(const sensor_msgs::Image::ConstPtr& msg);
    void yoloStatusCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // 新增统一物体检测回调函数
    void objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg,
                              const geometry_msgs::PoseArray::ConstPtr& poses_msg);
    
    // 辅助函数
    void sendJointCommand(const std::vector<double>& positions);
    void sendGripperCommand(bool open);
    void sendVacuumCommand(bool on, int power = 100);
    void logMessage(const QString& message);
    QImage cvMatToQImage(const cv::Mat& mat);
    void generateTestImages();
    
    // 初始化函数
    void initializeGUI();
    void initializeJointControlConnections();
    void initializeROS();
    void initializeOpenGL();
    
    // 更新函数 - 移除重复声明，因为已在private slots中声明
    void updateJointControlWidgets();
    void updateVacuumStatus();
    
    // 命令函数
    void sendPickCommand(const std::string& object_id);
    void sendPlaceCommand(double x, double y, double z);
    void sendHomeCommand();
    
    // 运动学函数
    std::vector<double> poseToJoints(const geometry_msgs::Pose& pose);
    geometry_msgs::Pose jointsToPos(const std::vector<double>& joint_values);
    
    // 渲染函数
    void renderRobotArm();
    
    // 电机命令
    void sendMotorOrder(uint8_t station_num, uint8_t form, int16_t vel, uint16_t vel_ac, uint16_t vel_de, bool pos_mode, int32_t pos, uint16_t pos_thr);
    void sendRelayOrder(const std::string& command);
    
    // 舵机命令
    void sendServoCommand(int servo_id, int position, int velocity = 1000, int acceleration = 100);
    
    // 辅助函数
    int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);

    // 添加新的辅助函数
    void updateDetectionsTable();

    // 3D渲染器
    Scene3DRenderer* scene_3d_renderer_;
    
    // 3D场景中的物体
    std::vector<std::pair<QVector3D, QColor>> scene_objects_;
    
    // 选中的物体索引
    int selected_object_index_;
    
    // 相机参数
    QMatrix3x3 camera_intrinsic_;
    QMatrix4x4 camera_extrinsic_;
    
    // 3D坐标转换相关
    QVector3D imagePointTo3D(const QPoint& image_point, float depth = 1.0);
    QPoint point3DToImage(const QVector3D& point_3d);
    float getDepthAtPoint(const QPoint& image_point);
    void updateCameraTransform(const geometry_msgs::Pose& end_effector_pose);
    
    // 渲染3D场景
    void render3DScene();
    
    // 更新3D场景中的物体
    void updateSceneObjects();
    
    // 设置摄像头参数
    void setupCameraParameters();
    
    // 发送物体拾取命令
    void sendPickObjectCommand(int object_index);

    bool enable_3d_rendering_;
};

#endif // ARM_CONTROL_GUI_H 