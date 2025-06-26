#include "arm_gui/arm_control_gui.h"
#include "ui_arm_control_main.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QTableWidgetItem>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/SetBool.h>

#include <opencv2/opencv.hpp>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>
#include <QPixmap>
#include <QPainter>

ArmControlGUI::ArmControlGUI(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::ArmControlMainWindow)
    , nh_(nh)
    , tf_listener_(tf_buffer_)
    , vacuum_on_(false)
    , vacuum_power_(0)
    , gripper_open_(false)
    , yolo_detection_enabled_(false)
{
    // 设置UI
    ui->setupUi(this);
    initializeGUI();
    initializeJointControlConnections();
    initializeROS();
    initializeOpenGL();
    
    // 设置定时器用于GUI更新
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &ArmControlGUI::onUpdateGUI);
    updateTimer->start(50); // 20Hz更新率
    
    // 默认关节值初始化
    current_joint_values_ = std::vector<double>{0, 0, 0, 0, M_PI/2, 10};
    
    // 设置窗口标题
    setWindowTitle("机械臂控制面板");
    
    // 添加日志
    logMessage("控制界面初始化完成");
    
    // 生成测试图像
    generateTestImages();
    
    // 设置相机图像显示区域
    ui->cameraView->setScaledContents(false);
    ui->cameraView->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    
    // 只订阅合成的双目摄像头话题 (OV4689, 1280x480)
    stereo_merged_sub_ = nh_.subscribe("/stereo_camera/image_merged", 1, &ArmControlGUI::stereoMergedCallback, this);
    
    // 直接订阅原始摄像头话题
    ros::Subscriber raw_camera_sub = nh_.subscribe("/camera/image_raw", 1, &ArmControlGUI::stereoMergedCallback, this);
    
    // 物体检测相关订阅
    detection_image_sub_ = nh_.subscribe("/stereo_camera/detection_image", 1, &ArmControlGUI::detectionImageCallback, this);
    ros::Subscriber yolo_detection_sub = nh_.subscribe("/yolo_detection/image", 1, &ArmControlGUI::detectionImageCallback, this);
    detection_poses_sub_ = nh_.subscribe("/yolo_detection/poses", 1, &ArmControlGUI::detectionPosesCallback, this);
    
    // YOLO状态订阅
    yolo_status_sub_ = nh_.subscribe("/yolo/status", 1, &ArmControlGUI::yoloStatusCallback, this);
    
    // YOLO控制服务客户端
    yolo_control_client_ = nh_.serviceClient<std_srvs::SetBool>("/yolo/control");
    
    // 设置定时器用于更新ROS
    QTimer *ros_timer = new QTimer(this);
    connect(ros_timer, &QTimer::timeout, this, &ArmControlGUI::updateROS);
    ros_timer->start(10); // 10ms
    
    // 打印订阅的话题信息
    ROS_INFO("GUI已订阅摄像头话题:");
    ROS_INFO(" - /stereo_camera/image_merged (合成双目图像)");
    ROS_INFO(" - /camera/image_raw (原始摄像头图像)");
    ROS_INFO(" - /stereo_camera/detection_image");
    ROS_INFO(" - /yolo_detection/image");
    ROS_INFO(" - /stereo_camera/depth");
    ROS_INFO(" - /yolo_detection/poses");
    
    // 输出调试信息到GUI日志
    logMessage("已订阅图像和检测结果话题，等待数据...");
}

ArmControlGUI::~ArmControlGUI()
{
    if (updateTimer) {
        updateTimer->stop();
        delete updateTimer;
    }
    
    // 关闭ROS订阅
    stereo_merged_sub_.shutdown();
    detection_image_sub_.shutdown();
    yolo_status_sub_.shutdown();
    
    delete ui;
}

// 初始化函数实现
void ArmControlGUI::initializeGUI()
{
    // 连接菜单操作
    connect(ui->actionOpen_Task_Sequence, &QAction::triggered, this, &ArmControlGUI::onOpenTaskSequence);
    connect(ui->actionSave_Task_Sequence, &QAction::triggered, this, &ArmControlGUI::onSaveTaskSequence);
    connect(ui->actionExit, &QAction::triggered, this, &ArmControlGUI::onExitApplication);
    connect(ui->actionCamera_Settings, &QAction::triggered, this, &ArmControlGUI::onCameraSettings);
    connect(ui->actionRobot_Settings, &QAction::triggered, this, &ArmControlGUI::onRobotSettings);
    connect(ui->actionAbout, &QAction::triggered, this, &ArmControlGUI::onAbout);
    
    // 连接末端执行器控制按钮
    connect(ui->moveToPositionButton, &QPushButton::clicked, this, &ArmControlGUI::onMoveToPositionClicked);
    connect(ui->homeButton, &QPushButton::clicked, this, &ArmControlGUI::onHomeButtonClicked);
    
    // 连接吸附控制
    connect(ui->vacuumPowerSlider, &QSlider::valueChanged, this, &ArmControlGUI::onVacuumPowerSliderChanged);
    connect(ui->vacuumOnButton, &QPushButton::clicked, this, &ArmControlGUI::onVacuumOnButtonClicked);
    connect(ui->vacuumOffButton, &QPushButton::clicked, this, &ArmControlGUI::onVacuumOffButtonClicked);
    
    // 路径规划按钮连接在路径规划控制组中
    
    // 创建并连接示例动作按钮
    QGroupBox* demoGroup = new QGroupBox("示例动作");
    QVBoxLayout* demoLayout = new QVBoxLayout(demoGroup);
    
    QPushButton* demoButton1 = new QPushButton("示例1：自由度展示");
    QPushButton* demoButton2 = new QPushButton("示例2：正常姿态");
    QPushButton* demoButton3 = new QPushButton("示例3：大角度姿态");
    
    demoLayout->addWidget(demoButton1);
    demoLayout->addWidget(demoButton2);
    demoLayout->addWidget(demoButton3);
    
    // 创建YOLO检测控制
    QGroupBox* visionGroup = new QGroupBox("视觉检测");
    QVBoxLayout* visionLayout = new QVBoxLayout(visionGroup);
    
    yolo_checkbox_ = new QCheckBox("启用YOLO目标检测");
    connect(yolo_checkbox_, &QCheckBox::toggled, this, &ArmControlGUI::onYoloDetectionToggled);
    visionLayout->addWidget(yolo_checkbox_);
    
    // 添加路径规划控制
    QGroupBox* pathPlanningGroup = new QGroupBox("路径规划");
    QVBoxLayout* pathPlanningLayout = new QVBoxLayout(pathPlanningGroup);
    
    // 添加选择放置区的下拉框
    QHBoxLayout* placementAreaLayout = new QHBoxLayout();
    QLabel* placementAreaLabel = new QLabel("选择放置区:");
    placement_area_combo_ = new QComboBox();
    placement_area_combo_->addItem("区域1", "area_1");
    placement_area_combo_->addItem("区域2", "area_2");
    placement_area_combo_->addItem("区域3", "area_3");
    placementAreaLayout->addWidget(placementAreaLabel);
    placementAreaLayout->addWidget(placement_area_combo_);
    pathPlanningLayout->addLayout(placementAreaLayout);
    
    // 添加路径规划按钮
    QPushButton* scanObjectsButton = new QPushButton("扫描物体");
    QPushButton* planPathButton = new QPushButton("规划路径");
    QPushButton* executePathButton = new QPushButton("执行路径");
    QPushButton* visualizeWorkspaceButton = new QPushButton("可视化工作空间");
    
    connect(scanObjectsButton, &QPushButton::clicked, this, &ArmControlGUI::onScanObjectsClicked);
    connect(planPathButton, &QPushButton::clicked, this, &ArmControlGUI::onPlanPathClicked);
    connect(executePathButton, &QPushButton::clicked, this, &ArmControlGUI::onExecutePathClicked);
    connect(visualizeWorkspaceButton, &QPushButton::clicked, this, &ArmControlGUI::onVisualizeWorkspaceClicked);
    
    pathPlanningLayout->addWidget(scanObjectsButton);
    pathPlanningLayout->addWidget(planPathButton);
    pathPlanningLayout->addWidget(executePathButton);
    pathPlanningLayout->addWidget(visualizeWorkspaceButton);
    
    // 将示例动作组添加到controlWidget的布局中
    QVBoxLayout* controlLayout = qobject_cast<QVBoxLayout*>(ui->controlWidget->layout());
    if (controlLayout) {
        controlLayout->addWidget(demoGroup);
        controlLayout->addWidget(visionGroup);  // 添加视觉控制组
        controlLayout->addWidget(pathPlanningGroup); // 添加路径规划控制组
    } else {
        // 如果没有布局，打印错误信息
        logMessage("错误: 无法获取控制部件布局");
    }
    
    // 连接示例动作按钮信号
    connect(demoButton1, &QPushButton::clicked, this, &ArmControlGUI::onDemoAction1Clicked);
    connect(demoButton2, &QPushButton::clicked, this, &ArmControlGUI::onDemoAction2Clicked);
    connect(demoButton3, &QPushButton::clicked, this, &ArmControlGUI::onDemoAction3Clicked);
    
    // 连接检测表格的单元格点击事件
    if (ui->detectionsTable) {
        connect(ui->detectionsTable, &QTableWidget::cellClicked, 
                this, &ArmControlGUI::onDetectionsTableCellClicked);
    }
}

void ArmControlGUI::initializeJointControlConnections()
{
    // 关节滑块与数值框连接
    connect(ui->joint1_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint1SliderChanged);
    connect(ui->joint2_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint2SliderChanged);
    connect(ui->joint3_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint3SliderChanged);
    connect(ui->joint4_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint4SliderChanged);
    connect(ui->joint6_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint6SliderChanged);
    
    connect(ui->joint1_spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), 
            this, &ArmControlGUI::onJoint1SpinChanged);
    connect(ui->joint2_spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), 
            this, &ArmControlGUI::onJoint2SpinChanged);
    connect(ui->joint3_spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), 
            this, &ArmControlGUI::onJoint3SpinChanged);
    connect(ui->joint4_spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), 
            this, &ArmControlGUI::onJoint4SpinChanged);
    connect(ui->joint6_spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), 
            this, &ArmControlGUI::onJoint6SpinChanged);
}

void ArmControlGUI::initializeROS()
{
    // 订阅关节状态
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &ArmControlGUI::jointStateCallback, this);
    
    // 发布关节命令
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm1/joint_command", 10);
    
    // 发布吸附命令
    vacuum_command_pub_ = nh_.advertise<std_msgs::Bool>("/arm1/vacuum_command", 10);
    
    // 发布机械臂命令
    arm_command_pub_ = nh_.advertise<std_msgs::String>("/arm_command", 10);
    
    // 发布电机控制命令
    motor_order_pub_ = nh_.advertise<liancheng_socket::MotorOrder>("Controller_motor_order", 5);
    
    // 发布继电器控制命令
    relay_order_pub_ = nh_.advertise<std_msgs::String>("RelayOrder", 5);
    
    // 发布夹持器命令
    gripper_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/arm1/gripper_command", 10);
    
    // 发布舵机控制命令
    servo_control_pub_ = nh_.advertise<servo_wrist::SerControl>("/servo_control_topic", 10);
}

void ArmControlGUI::initializeOpenGL()
{
    // 这里可以初始化OpenGL相关的内容
    // 例如：加载3D模型、设置光照等
    // ...
}

// 关节控制槽实现
void ArmControlGUI::onJoint1SliderChanged(int value)
{
    ui->joint1_spin->setValue(value);
    current_joint_values_[0] = value * M_PI / 180.0; // 转换为弧度
    sendJointCommand(current_joint_values_);
    
    // 底座旋转对应舵机ID 1，位置范围映射到舵机范围
    int servo_position = map(value, -180, 180, 500, 2500);
    sendServoCommand(1, servo_position);
}

void ArmControlGUI::onJoint2SliderChanged(int value)
{
    ui->joint2_spin->setValue(value);
    current_joint_values_[1] = value;
    sendJointCommand(current_joint_values_);
}

void ArmControlGUI::onJoint3SliderChanged(int value)
{
    ui->joint3_spin->setValue(value);
    current_joint_values_[2] = value * M_PI / 180.0; // 转换为弧度
    sendJointCommand(current_joint_values_);
    
    // 肩部关节对应舵机ID 2，位置范围映射到舵机范围
    int servo_position = map(value, -90, 90, 500, 2500);
    sendServoCommand(2, servo_position);
}

void ArmControlGUI::onJoint4SliderChanged(int value)
{
    ui->joint4_spin->setValue(value);
    current_joint_values_[3] = value * M_PI / 180.0; // 转换为弧度
    sendJointCommand(current_joint_values_);
    
    // 肘部关节对应舵机ID 3，位置范围映射到舵机范围
    int servo_position = map(value, 0, 180, 500, 2500);
    sendServoCommand(3, servo_position);
}

void ArmControlGUI::onJoint6SliderChanged(int value)
{
    ui->joint6_spin->setValue(value);
    current_joint_values_[5] = value;
    sendJointCommand(current_joint_values_);
}

void ArmControlGUI::onJoint1SpinChanged(double value)
{
    ui->joint1_slider->setValue(static_cast<int>(value));
    current_joint_values_[0] = value * M_PI / 180.0; // 转换为弧度
    sendJointCommand(current_joint_values_);
}

void ArmControlGUI::onJoint2SpinChanged(double value)
{
    ui->joint2_slider->setValue(static_cast<int>(value));
    current_joint_values_[1] = value;
    sendJointCommand(current_joint_values_);
}

void ArmControlGUI::onJoint3SpinChanged(double value)
{
    ui->joint3_slider->setValue(static_cast<int>(value));
    current_joint_values_[2] = value * M_PI / 180.0; // 转换为弧度
    sendJointCommand(current_joint_values_);
}

void ArmControlGUI::onJoint4SpinChanged(double value)
{
    ui->joint4_slider->setValue(static_cast<int>(value));
    current_joint_values_[3] = value * M_PI / 180.0; // 转换为弧度
    sendJointCommand(current_joint_values_);
}

void ArmControlGUI::onJoint6SpinChanged(double value)
{
    ui->joint6_slider->setValue(static_cast<int>(value));
    current_joint_values_[5] = value;
    sendJointCommand(current_joint_values_);
}

// 末端执行器控制槽实现
void ArmControlGUI::onMoveToPositionClicked()
{
    double x = ui->pos_x->value();
    double y = ui->pos_y->value();
    double z = ui->pos_z->value();
    
    // 发送放置命令
    sendPlaceCommand(x, y, z);
    logMessage(QString("移动到位置 (%1, %2, %3)").arg(x).arg(y).arg(z));
}

void ArmControlGUI::onHomeButtonClicked()
{
    // 发送回到初始位置命令
    sendHomeCommand();
    logMessage("移动到初始位置");
}

// 吸附控制槽实现
void ArmControlGUI::onVacuumPowerSliderChanged(int value)
{
    ui->vacuumPowerLabel->setText(QString("%1%").arg(value));
    vacuum_power_ = value;
    
    // 如果吸附已经开启，则更新功率
    if (vacuum_on_) {
        sendVacuumCommand(true, vacuum_power_);
    }
}

void ArmControlGUI::onVacuumOnButtonClicked()
{
    vacuum_on_ = true;
    sendVacuumCommand(true, vacuum_power_);
    logMessage(QString("开启吸附，功率: %1%").arg(vacuum_power_));
}

void ArmControlGUI::onVacuumOffButtonClicked()
{
    vacuum_on_ = false;
    sendVacuumCommand(false, 0);
    logMessage("关闭吸附");
}

// 任务控制槽实现
void ArmControlGUI::onPickButtonClicked()
{
    // 检查是否有检测到的物体
    if (detected_objects_.empty()) {
        logMessage("未检测到物体，无法执行抓取操作");
        return;
    }
    
    // 简单起见，选择第一个检测到的物体
    const DetectedObject& object = detected_objects_[0];
    sendPickCommand(object.id);
    logMessage(QString("抓取物体 %1，位置: (%2, %3, %4)").arg(
        QString::fromStdString(object.id)).arg(object.x).arg(object.y).arg(object.z));
}

void ArmControlGUI::onPlaceButtonClicked()
{
    double x = ui->pos_x->value();
    double y = ui->pos_y->value();
    double z = ui->pos_z->value();
    
    // 发送放置命令
    sendPlaceCommand(x, y, z);
    logMessage(QString("放置物体到位置 (%1, %2, %3)").arg(x).arg(y).arg(z));
}

void ArmControlGUI::onSequenceButtonClicked()
{
    // 这里可以实现预设任务序列执行逻辑
    logMessage("执行预设任务序列");
}

// 菜单操作槽实现
void ArmControlGUI::onOpenTaskSequence()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("打开任务序列文件"), "", tr("任务序列文件 (*.json *.yaml)"));
    if (!fileName.isEmpty()) {
        logMessage(QString("打开任务序列文件: %1").arg(fileName));
    }
}

void ArmControlGUI::onSaveTaskSequence()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("保存任务序列"), "", tr("任务序列文件 (*.json)"));
    if (!fileName.isEmpty()) {
        logMessage(QString("保存任务序列到文件: %1").arg(fileName));
    }
}

void ArmControlGUI::onExitApplication()
{
    QApplication::quit();
}

void ArmControlGUI::onCameraSettings()
{
    // 这里可以实现摄像头设置对话框
    QMessageBox::information(this, "摄像头设置", "摄像头设置功能尚未实现");
}

void ArmControlGUI::onRobotSettings()
{
    // 这里可以实现机械臂设置对话框
    QMessageBox::information(this, "机械臂设置", "机械臂设置功能尚未实现");
}

void ArmControlGUI::onAbout()
{
    QMessageBox::about(this, "关于", "机械臂控制面板\n版本: 0.1.0\n基于ROS和Qt5开发的机械臂控制界面");
}

// 检测表格操作槽实现
void ArmControlGUI::onDetectionsTableCellClicked(int row, int column)
{
    // 由于已经移除了检测表格，此函数现在为空实现
    // 保留函数定义以避免编译错误，但不执行任何操作
    ROS_WARN("onDetectionsTableCellClicked被调用，但检测表格已被移除");
}

// 定时器槽实现
void ArmControlGUI::onUpdateGUI()
{
    // 在这里更新GUI元素，如3D视图、状态等
    updateJointControlWidgets();
    updateEndEffectorPose();
    updateCameraViews();
    updateJointInfo();
    updateConnectionStatus();
}

// ROS回调函数实现
void ArmControlGUI::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // 更新当前关节值
    if (msg->position.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            if (i < msg->position.size()) {
                current_joint_values_[i] = msg->position[i];
            }
        }
    }
}

void ArmControlGUI::leftCameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 将ROS图像转换为OpenCV图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        
        // 记录图像信息
        ROS_INFO("接收到左相机图像: 尺寸=%dx%d, 编码=%s", 
                cv_ptr->image.cols, cv_ptr->image.rows, msg->encoding.c_str());
        logMessage(QString("接收到左相机图像: %1x%2").arg(cv_ptr->image.cols).arg(cv_ptr->image.rows));
        
        // 转换为QImage
        QImage image(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, 
                     cv_ptr->image.step, QImage::Format_RGB888);
        image = image.rgbSwapped(); // BGR to RGB
        
        // 保存图像
        left_camera_image_ = image;
        
        // 将图像同时设置为当前显示的图像
        current_camera_image_ = image;
        
        // 记录成功接收的图像帧
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 30 == 0) {  // 每30帧记录一次
            ROS_INFO("成功接收到摄像头图像帧：%d (尺寸: %dx%d)", 
                    frame_count, image.width(), image.height());
        }
        
        // 更新UI (在主线程中)
        QMetaObject::invokeMethod(this, "updateCameraView", Qt::QueuedConnection);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in left camera callback: %s", e.what());
        logMessage(QString("图像转换错误: %1").arg(e.what()));
    }
}

// rightCameraCallback函数已删除

void ArmControlGUI::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 将ROS图像转换为OpenCV图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        
        // 转换为QImage
        QImage image(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, 
                     cv_ptr->image.step, QImage::Format_RGB888);
        image = image.rgbSwapped(); // BGR to RGB
        
        // 保存图像
        detection_image_ = image;
        
        // 记录成功接收的检测图像帧
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 30 == 0) {  // 每30帧记录一次
            ROS_INFO("成功接收到检测图像帧：%d (尺寸: %dx%d)", 
                   frame_count, image.width(), image.height());
        }
        
        // 更新UI (在主线程中)
        QMetaObject::invokeMethod(this, "updateCameraView", Qt::QueuedConnection);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in detection image callback: %s", e.what());
    }
}

void ArmControlGUI::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // 清空之前的检测结果
    detected_objects_.clear();
    
    // 生成新的检测列表
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        DetectedObject obj;
        obj.id = std::to_string(i+1);  // 从1开始编号
        obj.type = "物体";  // 默认类型
        
        // 转换为图像坐标 (简化处理)
        // 这里将3D世界坐标转为图像像素坐标
        // 实际应用中需要使用相机内参和投影矩阵
        double scale = 100.0; // 缩放因子
        obj.x = static_cast<int>((msg->poses[i].position.x + 0.5) * scale); 
        obj.y = static_cast<int>((0.5 - msg->poses[i].position.y) * scale);
        obj.z = msg->poses[i].position.z * 100; // 转换为厘米
        
        obj.pose = msg->poses[i];
        detected_objects_.push_back(obj);
    }
    
    // 更新检测结果表格
    updateDetectionsTable();
    
    // 检测结果更新后，立即更新视图
    QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
    
    // 记录检测结果
    if (!detected_objects_.empty()) {
        ROS_INFO("收到%zu个物体检测结果", detected_objects_.size());
    }
}

// GUI更新函数实现
void ArmControlGUI::updateJointControlWidgets()
{
    // 防止反馈循环
    bool oldState = ui->joint1_slider->blockSignals(true);
    ui->joint1_slider->setValue(static_cast<int>(current_joint_values_[0] * 180.0 / M_PI)); // 弧度转度
    ui->joint1_slider->blockSignals(oldState);
    
    oldState = ui->joint1_spin->blockSignals(true);
    ui->joint1_spin->setValue(current_joint_values_[0] * 180.0 / M_PI);
    ui->joint1_spin->blockSignals(oldState);
    
    oldState = ui->joint2_slider->blockSignals(true);
    ui->joint2_slider->setValue(static_cast<int>(current_joint_values_[1]));
    ui->joint2_slider->blockSignals(oldState);
    
    oldState = ui->joint2_spin->blockSignals(true);
    ui->joint2_spin->setValue(current_joint_values_[1]);
    ui->joint2_spin->blockSignals(oldState);
    
    oldState = ui->joint3_slider->blockSignals(true);
    ui->joint3_slider->setValue(static_cast<int>(current_joint_values_[2] * 180.0 / M_PI));
    ui->joint3_slider->blockSignals(oldState);
    
    oldState = ui->joint3_spin->blockSignals(true);
    ui->joint3_spin->setValue(current_joint_values_[2] * 180.0 / M_PI);
    ui->joint3_spin->blockSignals(oldState);
    
    oldState = ui->joint4_slider->blockSignals(true);
    ui->joint4_slider->setValue(static_cast<int>(current_joint_values_[3] * 180.0 / M_PI));
    ui->joint4_slider->blockSignals(oldState);
    
    oldState = ui->joint4_spin->blockSignals(true);
    ui->joint4_spin->setValue(current_joint_values_[3] * 180.0 / M_PI);
    ui->joint4_spin->blockSignals(oldState);
    
    oldState = ui->joint6_slider->blockSignals(true);
    ui->joint6_slider->setValue(static_cast<int>(current_joint_values_[5]));
    ui->joint6_slider->blockSignals(oldState);
    
    oldState = ui->joint6_spin->blockSignals(true);
    ui->joint6_spin->setValue(current_joint_values_[5]);
    ui->joint6_spin->blockSignals(oldState);
}

void ArmControlGUI::updateEndEffectorPose()
{
    // 计算末端位姿
    current_end_effector_pose_ = jointsToPos(current_joint_values_);
    
    // 更新位置输入框(转换为厘米)
    ui->pos_x->setValue(current_end_effector_pose_.position.x * 100);
    ui->pos_y->setValue(current_end_effector_pose_.position.y * 100);
    ui->pos_z->setValue(current_end_effector_pose_.position.z * 100);
}

void ArmControlGUI::updateVacuumStatus()
{
    // 更新吸附状态显示
    ui->vacuumPowerSlider->setValue(vacuum_power_);
    ui->vacuumPowerLabel->setText(QString("%1%").arg(vacuum_power_));
    
    // 更新按钮状态
    ui->vacuumOnButton->setEnabled(!vacuum_on_);
    ui->vacuumOffButton->setEnabled(vacuum_on_);
}

void ArmControlGUI::updateCameraViews()
{
    updateCameraView();
}

void ArmControlGUI::updateCameraView()
{
    try {
        // 获取相机视图的大小
        QSize labelSize = ui->cameraView->size();
        
        // 获取基础图像
        QImage base_image;
        if (!current_camera_image_.isNull()) {
            base_image = current_camera_image_;
        } else if (!left_camera_image_.isNull()) {
            base_image = left_camera_image_;
        } else {
            // 如果没有图像，显示空白
            ui->cameraView->setText("等待摄像头图像...");
            return;
        }
        
        // 创建可绘制的图像副本
        QImage drawable_image = base_image.copy();
        
        // 如果启用了YOLO检测且有检测结果，在基础图像上绘制检测框和标签
        if (yolo_detection_enabled_ && !detected_objects_.empty()) {
            QPainter painter(&drawable_image);
            painter.setRenderHint(QPainter::Antialiasing);
            
            // 设置字体
            QFont font = painter.font();
            font.setPointSize(14);
            font.setBold(true);
            painter.setFont(font);
            
            // 遍历所有检测到的物体
            for (const DetectedObject& obj : detected_objects_) {
                // 计算边界框 (这里假设x,y是物体中心坐标)
                int box_width = 80;  // 根据实际情况调整
                int box_height = 80;
                int x1 = obj.x - box_width/2;
                int y1 = obj.y - box_height/2;
                int x2 = obj.x + box_width/2;
                int y2 = obj.y + box_height/2;
                
                // 确保边界框在图像范围内
                x1 = qMax(0, x1);
                y1 = qMax(0, y1);
                x2 = qMin(drawable_image.width() - 1, x2);
                y2 = qMin(drawable_image.height() - 1, y2);
                
                // 设置绘制颜色和宽度
                QPen pen(Qt::red);
                pen.setWidth(3);
                painter.setPen(pen);
                
                // 绘制边界框
                painter.drawRect(x1, y1, x2 - x1, y2 - y1);
                
                // 绘制物体标签背景
                QFontMetrics fm(font);
                QString label = QString("%1 (%.1fcm)").arg(QString::fromStdString(obj.type)).arg(obj.z);
                QRect textRect = fm.boundingRect(label);
                QRect bgRect(x1, y1 - 25, textRect.width() + 10, 25);
                painter.fillRect(bgRect, QColor(0, 0, 0, 180)); // 半透明黑色背景
                
                // 绘制物体标签
                painter.setPen(Qt::white);
                painter.drawText(x1 + 5, y1 - 5, label);
                
                // 绘制距离信息
                QString distInfo = QString("距离: %.1fcm").arg(obj.z);
                painter.drawText(x1 + 5, y2 + 20, distInfo);
            }
            
            // 绘制状态信息
            painter.setPen(Qt::green);
            painter.drawText(10, 30, QString("YOLO: 启用"));
            painter.drawText(10, 60, QString("检测到物体: %1 个").arg(detected_objects_.size()));
        } else {
            // 绘制YOLO状态信息
            QPainter painter(&drawable_image);
            painter.setRenderHint(QPainter::Antialiasing);
            
            QFont font = painter.font();
            font.setPointSize(12);
            painter.setFont(font);
            
            painter.setPen(Qt::red);
            painter.drawText(10, 30, QString("YOLO: %1").arg(yolo_detection_enabled_ ? "启用（等待检测结果）" : "禁用"));
        }
        
        // 缩放图像以适应标签大小，保持纵横比
        QPixmap scaled_pixmap = QPixmap::fromImage(drawable_image).scaled(
            ui->cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        
        // 创建背景
        QPixmap background(ui->cameraView->size());
        background.fill(Qt::black);
        
        // 将图像放在中间
        int x = (ui->cameraView->size().width() - scaled_pixmap.width()) / 2;
        int y = (ui->cameraView->size().height() - scaled_pixmap.height()) / 2;
        
        QPainter painter(&background);
        painter.drawPixmap(x, y, scaled_pixmap);
        
        // 设置图像
        ui->cameraView->setPixmap(background);
        
        // 如果图像为空，显示错误信息
        if (ui->cameraView->pixmap() == nullptr || ui->cameraView->pixmap()->isNull()) {
            QPixmap emptyPix(ui->cameraView->width(), ui->cameraView->height());
            emptyPix.fill(Qt::black);
            
            QPainter painter(&emptyPix);
            painter.setPen(Qt::white);
            painter.setFont(QFont("Arial", 14));
            painter.drawText(emptyPix.rect(), Qt::AlignCenter, "摄像头未连接或未启动");
            
            ui->cameraView->setPixmap(emptyPix);
        }
    }
    catch (std::exception& e) {
        ROS_ERROR("Error in updateCameraView: %s", e.what());
    }
}

// 操作函数实现
void ArmControlGUI::sendJointCommand(const std::vector<double>& joint_values)
{
    // 创建关节命令消息
    sensor_msgs::JointState joint_cmd;
    joint_cmd.header.stamp = ros::Time::now();
    joint_cmd.name = {"arm1_joint1", "arm1_joint2", "arm1_joint3", "arm1_joint4", "arm1_joint5", "arm1_joint6"};
    joint_cmd.position = joint_values;
    
    // 发布关节命令
    joint_command_pub_.publish(joint_cmd);
}

void ArmControlGUI::sendVacuumCommand(bool on, int power)
{
    // 创建吸附命令消息
    std_msgs::Bool vacuum_cmd;
    vacuum_cmd.data = on;
    
    // 创建功率设置消息
    std_msgs::Float64 power_cmd;
    power_cmd.data = power / 100.0; // 转换为0-1范围
    
    // 发布吸附命令
    vacuum_command_pub_.publish(vacuum_cmd);
    
    // 更新状态
    vacuum_on_ = on;
    vacuum_power_ = power;
}

void ArmControlGUI::sendPickCommand(const std::string& object_id)
{
    std_msgs::String cmd_msg;
    cmd_msg.data = "pick arm1 " + object_id;
    arm_command_pub_.publish(cmd_msg);
}

void ArmControlGUI::sendPlaceCommand(double x, double y, double z)
{
    std_msgs::String cmd_msg;
    cmd_msg.data = "place arm1 " + std::to_string(x/100.0) + " " + 
                   std::to_string(y/100.0) + " " + std::to_string(z/100.0);
    arm_command_pub_.publish(cmd_msg);
}

void ArmControlGUI::sendHomeCommand()
{
    std_msgs::String cmd_msg;
    cmd_msg.data = "home arm1";
    arm_command_pub_.publish(cmd_msg);
}

// 辅助函数实现
QImage ArmControlGUI::cvMatToQImage(const cv::Mat& mat)
{
    // 常见的类型转换
    if (mat.type() == CV_8UC1) {
        // 单通道灰度图
        QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
        return image.copy();
    } else if (mat.type() == CV_8UC3) {
        // 三通道彩色图(BGR)
        QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped().copy();
    } else if (mat.type() == CV_8UC4) {
        // 四通道彩色图
        QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    } else {
        // 其他类型，先转换
        cv::Mat converted;
        mat.convertTo(converted, CV_8UC3);
        QImage image(converted.data, converted.cols, converted.rows, converted.step, QImage::Format_RGB888);
        return image.rgbSwapped().copy();
    }
}

std::vector<double> ArmControlGUI::poseToJoints(const geometry_msgs::Pose& pose)
{
    // 这里应该实现逆运动学算法，将末端位姿转换为关节角度
    // 简单起见，先返回当前关节值
    return current_joint_values_;
}

geometry_msgs::Pose ArmControlGUI::jointsToPos(const std::vector<double>& joint_values)
{
    // 这里应该实现正运动学算法，将关节角度转换为末端位姿
    // 简单起见，先返回一个假的位姿
    geometry_msgs::Pose pose;
    pose.position.x = 0.1 + 0.05 * sin(joint_values[0]);
    pose.position.y = 0.2 * sin(joint_values[2]);
    pose.position.z = 0.3 + 0.1 * joint_values[1] / 43.0 + 0.05 * sin(joint_values[3]);
    pose.orientation.w = 1.0;
    return pose;
}

// 日志记录
void ArmControlGUI::logMessage(const QString& message)
{
    // 添加时间戳
    QDateTime now = QDateTime::currentDateTime();
    QString timestamped = now.toString("[hh:mm:ss.zzz] ") + message;
    
    // 添加到日志
    ui->logTextEdit->append(timestamped);
    
    // 滚动到底部 - 使用简化版本避免QScrollBar的问题
    ui->logTextEdit->moveCursor(QTextCursor::End);
}

// OpenGL渲染
void ArmControlGUI::renderRobotArm()
{
    // 这里应该实现OpenGL渲染机械臂的代码
    // 需要使用当前的关节值计算各个连杆的位置和姿态
    // ...
}

// 电机控制函数
void ArmControlGUI::sendMotorOrder(uint8_t station_num, uint8_t form, int16_t vel, uint16_t vel_ac, uint16_t vel_de, bool pos_mode, int32_t pos, uint16_t pos_thr)
{
    liancheng_socket::MotorOrder msg;
    msg.header.stamp = ros::Time::now();
    msg.station_num.push_back(station_num);
    msg.form.push_back(form);
    msg.vel.push_back(vel);
    msg.vel_ac.push_back(vel_ac);
    msg.vel_de.push_back(vel_de);
    msg.pos_mode.push_back(pos_mode);
    msg.pos.push_back(pos);
    msg.pos_thr.push_back(pos_thr);
    
    motor_order_pub_.publish(msg);
    logMessage(QString("发送电机命令: 站点=%1, 位置=%2").arg(station_num).arg(pos));
}

void ArmControlGUI::sendRelayOrder(const std::string& command)
{
    std_msgs::String msg;
    msg.data = command;
    relay_order_pub_.publish(msg);
    logMessage(QString("发送继电器命令: %1").arg(QString::fromStdString(command)));
}

// 示例动作函数
void ArmControlGUI::onDemoAction1Clicked()
{
    logMessage("执行示例动作1: 自由度展示");
    
    // 夹持电机移动
    sendMotorOrder(1, 11, 0, 0, 0, true, -25, 30);
    QApplication::processEvents();
    ros::Duration(5.0).sleep();
    logMessage("夹持电机移动");
    
    // 夹持电机归零
    sendMotorOrder(1, 11, 0, 0, 0, true, 0, 30);
    QApplication::processEvents();
    ros::Duration(5.0).sleep();
    logMessage("夹持电机归零");
    
    // 进给电机移动
    sendMotorOrder(2, 100, 200, 0, 0, false, -2000, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    sendMotorOrder(2, 0, 200, 0, 0, false, -2000, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    sendMotorOrder(2, 99, 200, 0, 0, false, -2000, 10);
    QApplication::processEvents();
    ros::Duration(3.0).sleep();
    logMessage("进给电机移动");
    
    // 进给电机归零
    sendMotorOrder(2, 100, 200, 0, 0, false, 1900, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    sendMotorOrder(2, 0, 200, 0, 0, true, 300, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    sendMotorOrder(2, 99, 200, 0, 0, false, 1900, 10);
    QApplication::processEvents();
    ros::Duration(3.0).sleep();
    sendMotorOrder(2, 100, 100, 0, 0, false, 1900, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    logMessage("进给电机归零");
    
    logMessage("示例动作1完成");
}

void ArmControlGUI::onDemoAction2Clicked()
{
    logMessage("执行示例动作2: 正常姿态展示");
    
    QApplication::processEvents();
    ros::Duration(5.0).sleep();
    
    // 进给电机移动
    sendMotorOrder(2, 100, 100, 0, 0, false, -2000, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    sendMotorOrder(2, 0, 100, 0, 0, false, -2500, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    sendMotorOrder(2, 99, 100, 0, 0, false, -2000, 10);
    QApplication::processEvents();
    ros::Duration(13.0).sleep();
    logMessage("进给电机移动");
    
    // 吸附启动
    sendRelayOrder("11");
    QApplication::processEvents();
    
    // 进给电机归零
    sendMotorOrder(2, 100, 200, 0, 0, false, 2000, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    sendMotorOrder(2, 0, 200, 0, 0, true, 300, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    sendMotorOrder(2, 99, 200, 0, 0, false, 2000, 10);
    QApplication::processEvents();
    ros::Duration(3.0).sleep();
    sendMotorOrder(2, 100, 100, 0, 0, false, 2000, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    logMessage("进给电机归零");
    
    QApplication::processEvents();
    ros::Duration(5.0).sleep();
    
    // 吸附关闭
    sendRelayOrder("00");
    QApplication::processEvents();
    
    logMessage("示例动作2完成");
}

void ArmControlGUI::onDemoAction3Clicked()
{
    logMessage("执行示例动作3: 大角度姿态展示");
    
    // 夹持电机移动
    sendMotorOrder(1, 11, 0, 0, 0, true, -25, 30);
    QApplication::processEvents();
    ros::Duration(5.0).sleep();
    logMessage("夹持电机移动");
    
    logMessage("示例动作3完成");
}

void ArmControlGUI::updateROS()
{
    if (ros::ok()) {
        ros::spinOnce();
    }
}

// 原来的updateCameraView函数已删除，实现已合并到主要的updateCameraView函数中

// 添加缺失的按钮点击处理函数
void ArmControlGUI::on_homeButton_clicked()
{
    onHomeButtonClicked();
}

void ArmControlGUI::on_stopButton_clicked()
{
    // 停止所有运动
    std_msgs::String cmd_msg;
    cmd_msg.data = "stop arm1";
    arm_command_pub_.publish(cmd_msg);
    logMessage("停止所有运动");
}

void ArmControlGUI::on_moveButton_clicked()
{
    onMoveToPositionClicked();
}

void ArmControlGUI::on_gripperOpenButton_clicked()
{
    sendGripperCommand(true);
    logMessage("打开夹持器");
}

void ArmControlGUI::on_gripperCloseButton_clicked()
{
    sendGripperCommand(false);
    logMessage("关闭夹持器");
}

void ArmControlGUI::on_vacuumOnButton_clicked()
{
    onVacuumOnButtonClicked();
}

void ArmControlGUI::on_vacuumOffButton_clicked()
{
    onVacuumOffButtonClicked();
}

void ArmControlGUI::on_example1Button_clicked()
{
    onDemoAction1Clicked();
}

void ArmControlGUI::on_example2Button_clicked()
{
    onDemoAction2Clicked();
}

void ArmControlGUI::on_example3Button_clicked()
{
    onDemoAction3Clicked();
}

void ArmControlGUI::updateUI()
{
    onUpdateGUI();
}

void ArmControlGUI::sendGripperCommand(bool open)
{
    // 创建夹持器命令消息
    std_msgs::Bool gripper_cmd;
    gripper_cmd.data = open;
    
    // 发布夹持器命令
    gripper_cmd_pub_.publish(gripper_cmd);
    
    // 更新状态
    gripper_open_ = open;
}

// 添加舵机控制函数实现
void ArmControlGUI::sendServoCommand(int servo_id, int position, int velocity, int acceleration)
{
    servo_wrist::SerControl msg;
    msg.servo_id = servo_id;
    msg.target_position = position;
    msg.velocity = velocity;
    msg.acceleration = acceleration;
    
    // 发布舵机控制消息
    servo_control_pub_.publish(msg);
    
    logMessage(QString("发送舵机命令: ID=%1, 位置=%2, 速度=%3").arg(servo_id).arg(position).arg(velocity));
}

// 添加辅助函数用于映射范围
int ArmControlGUI::map(int value, int fromLow, int fromHigh, int toLow, int toHigh)
{
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

// 添加新的回调函数用于处理合并的立体图像
void ArmControlGUI::stereoMergedCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 将ROS图像转换为OpenCV图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        
        // 转换为QImage
        QImage image(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, 
                     cv_ptr->image.step, QImage::Format_RGB888);
        image = image.rgbSwapped(); // BGR to RGB
        
        // 保存图像
        left_camera_image_ = image; // 使用合并图像作为主图像
        current_camera_image_ = image; // 同时设置为当前图像
        
        // 记录成功接收的图像帧
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 30 == 0) {  // 每30帧记录一次
            ROS_INFO("成功接收到立体合并图像帧：%d (尺寸: %dx%d)", 
                   frame_count, image.width(), image.height());
        }
        
        // 更新UI (在主线程中)
        QMetaObject::invokeMethod(this, "updateCameraView", Qt::QueuedConnection);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in stereo merged callback: %s", e.what());
    }
}

// 修改YOLO状态回调函数
void ArmControlGUI::yoloStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
    bool previous_state = yolo_detection_enabled_;
    yolo_detection_enabled_ = msg->data;
    
    // 当状态变化时记录
    if (previous_state != yolo_detection_enabled_) {
        ROS_INFO("YOLO检测状态改变: %s", yolo_detection_enabled_ ? "启用" : "禁用");
        logMessage(QString("YOLO检测状态已改变: %1").arg(yolo_detection_enabled_ ? "启用" : "禁用"));
    }
    
    // 更新复选框状态，但不触发信号
    if (yolo_checkbox_) {
        yolo_checkbox_->blockSignals(true);
        yolo_checkbox_->setChecked(yolo_detection_enabled_);
        yolo_checkbox_->blockSignals(false);
    }
    
    // 立即更新视图，以便正确显示检测结果或普通图像
    updateCameraViews();
}

// 修改YOLO切换处理函数
void ArmControlGUI::onYoloDetectionToggled(bool checked)
{
    // 记录操作到日志
    logMessage(QString("尝试%1 YOLO检测...").arg(checked ? "启用" : "禁用"));
    
    // 无论服务是否可用，先将状态保存起来
    yolo_detection_enabled_ = checked;
    
    // 检查服务是否可用
    if (!yolo_control_client_.exists()) {
        logMessage("警告: YOLO控制服务不可用，但已记录您的选择");
        return;
    }
    
    // 创建服务请求
    std_srvs::SetBool srv;
    srv.request.data = checked;
    
    // 调用服务
    if (yolo_control_client_.call(srv)) {
        if (srv.response.success) {
            logMessage(QString("YOLO检测已%1").arg(checked ? "启用" : "禁用"));
        } else {
            logMessage(QString("无法%1 YOLO检测: %2").arg(
                checked ? "启用" : "禁用").arg(QString::fromStdString(srv.response.message)));
            
            // 恢复复选框状态
            if (yolo_checkbox_) {
                yolo_checkbox_->blockSignals(true);
                yolo_checkbox_->setChecked(!checked); // 恢复之前的状态
                yolo_checkbox_->blockSignals(false);
            }
        }
    } else {
        logMessage("YOLO控制服务调用失败，但已记录您的选择");
    }
    
    // 即使服务调用失败，也立即更新视图，以便在本地显示检测结果
    updateCameraViews();
}

// 添加路径规划相关的槽函数
void ArmControlGUI::onScanObjectsClicked()
{
    logMessage("开始扫描物体...");
    
    // 确保YOLO检测已启用
    if (!yolo_detection_enabled_) {
        logMessage("启用YOLO检测以扫描物体");
        if (yolo_checkbox_) {
            yolo_checkbox_->setChecked(true);
        }
    }
    
    // 发送扫描命令
    std_msgs::String cmd_msg;
    cmd_msg.data = "scan";
    arm_command_pub_.publish(cmd_msg);
}

void ArmControlGUI::onPlanPathClicked()
{
    logMessage("开始规划路径...");
    
    // 由于检测表格已被移除，我们直接使用第一个检测到的物体（如果有的话）
    if (detected_objects_.empty()) {
        logMessage("没有检测到物体，无法规划路径");
        return;
    }
    
    // 使用第一个检测到的物体
    const DetectedObject& obj = detected_objects_[0];
    QString object_id = QString::fromStdString(obj.id);
    
    // 获取选中的放置区
    QString placement_area = placement_area_combo_->currentData().toString();
    
    // 发送规划命令
    std_msgs::String cmd_msg;
    cmd_msg.data = QString("plan %1 %2").arg(object_id).arg(placement_area).toStdString();
    arm_command_pub_.publish(cmd_msg);
    
    logMessage(QString("已规划从物体 %1 到放置区 %2 的路径").arg(object_id).arg(placement_area));
}

void ArmControlGUI::onExecutePathClicked()
{
    logMessage("执行规划的路径...");
    
    // 发送执行命令
    std_msgs::String cmd_msg;
    cmd_msg.data = "execute";
    arm_command_pub_.publish(cmd_msg);
}

void ArmControlGUI::onVisualizeWorkspaceClicked()
{
    logMessage("可视化工作空间...");
    
    // 发送可视化命令
    std_msgs::String cmd_msg;
    cmd_msg.data = "visualize_workspace";
    arm_command_pub_.publish(cmd_msg);
}

// 新增统一的物体检测回调函数，处理YOLO格式的检测结果
void ArmControlGUI::objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg, 
                                         const geometry_msgs::PoseArray::ConstPtr& poses_msg)
{
    try {
        // 处理检测图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        
        // 转换为QImage并保存
        QImage image(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, 
                     cv_ptr->image.step, QImage::Format_RGB888);
        image = image.rgbSwapped(); // BGR to RGB
        
        // 更新基础图像和检测图像
        left_camera_image_ = image;
        current_camera_image_ = image;
        
        // 处理物体位置数据
        detected_objects_.clear();
        for (size_t i = 0; i < poses_msg->poses.size(); ++i) {
            DetectedObject obj;
            obj.id = "object_" + std::to_string(i);
            obj.type = "物体"; // 默认类型
            
            // 转换为图像坐标 (简化处理)
            double scale = 100.0; // 缩放因子
            obj.x = static_cast<int>((poses_msg->poses[i].position.x + 0.5) * scale);
            obj.y = static_cast<int>((0.5 - poses_msg->poses[i].position.y) * scale);
            obj.z = poses_msg->poses[i].position.z * 100;
            
            obj.pose = poses_msg->poses[i];
            detected_objects_.push_back(obj);
        }
        
        // 更新UI
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("统一物体检测回调中的cv_bridge异常: %s", e.what());
    }
}

// 修改updateJointInfo函数实现，使用正确的UI元素
void ArmControlGUI::updateJointInfo()
{
    // 更新关节信息显示
    if (current_joint_values_.size() >= 6) {
        // 更新关节角度标签 - 使用关节控制组中的标签
        // 由于UI中没有专门的joint*_value标签，我们可以更新关节控制组中的标签文本
        if (ui->label_joint1) {
            ui->label_joint1->setText(QString("底座旋转 (θ1)：%1°").arg(
                QString::number(current_joint_values_[0] * 180.0 / M_PI, 'f', 1)));
        }
        
        if (ui->label_joint2) {
            ui->label_joint2->setText(QString("伸缩关节 (d2)：%1cm").arg(
                QString::number(current_joint_values_[1], 'f', 1)));
        }
        
        if (ui->label_joint3) {
            ui->label_joint3->setText(QString("肩部关节 (θ3)：%1°").arg(
                QString::number(current_joint_values_[2] * 180.0 / M_PI, 'f', 1)));
        }
        
        if (ui->label_joint4) {
            ui->label_joint4->setText(QString("肘部关节 (θ4)：%1°").arg(
                QString::number(current_joint_values_[3] * 180.0 / M_PI, 'f', 1)));
        }
        
        if (ui->label_joint6) {
            ui->label_joint6->setText(QString("末端伸缩 (d6)：%1cm").arg(
                QString::number(current_joint_values_[5], 'f', 1)));
        }
    }
}

// 添加updateConnectionStatus函数实现
void ArmControlGUI::updateConnectionStatus()
{
    // 检查ROS连接状态
    bool ros_ok = ros::ok();
    
    // 检查相机连接状态
    bool camera_ok = !left_camera_image_.isNull() || !current_camera_image_.isNull();
    
    // 检查深度相机连接状态
    // 深度视图已删除
    bool depth_ok = false;
    
    // 更新状态栏
    QString status;
    if (ros_ok) {
        status += "ROS: 已连接 | ";
    } else {
        status += "ROS: 未连接 | ";
    }
    
    if (camera_ok) {
        status += "相机: 已连接 | ";
    } else {
        status += "相机: 未连接 | ";
    }
    
    if (depth_ok) {
        status += "深度相机: 已连接";
    } else {
        status += "深度相机: 未连接";
    }
    
    // 设置状态栏文本
    statusBar()->showMessage(status);
}

// 添加测试图像生成函数
void ArmControlGUI::generateTestImages()
{
    // 生成相机测试图像
    QImage testCamera(640, 480, QImage::Format_RGB888);
    testCamera.fill(Qt::black);
    
    QPainter painterCamera(&testCamera);
    painterCamera.setPen(QPen(Qt::white, 2));
    painterCamera.setFont(QFont("Arial", 14, QFont::Bold));
    
    painterCamera.drawText(50, 100, "测试相机图像");
    painterCamera.drawText(50, 150, "等待摄像头连接...");
    
    // 绘制一些图形以便识别
    painterCamera.setPen(QPen(Qt::red, 3));
    painterCamera.drawEllipse(320, 240, 100, 100);
    painterCamera.setPen(QPen(Qt::green, 3));
    painterCamera.drawRect(220, 140, 200, 200);
    
    // 保存测试图像
    left_camera_image_ = testCamera;
    
    // 删除深度图测试图像生成
    // remove: // 生成深度图测试图像
    // remove: QImage testDepth(640, 480, QImage::Format_RGB888);
    // remove: testDepth.fill(Qt::black);
    // remove: 
    // remove: QPainter painterDepth(&testDepth);
    // remove: painterDepth.setPen(QPen(Qt::white, 2));
    // remove: painterDepth.setFont(QFont("Arial", 14, QFont::Bold));
    // remove: 
    // remove: painterDepth.drawText(50, 100, "测试深度图像");
    // remove: painterDepth.drawText(50, 150, "等待深度图数据...");
    // remove: 
    // remove: // 创建彩色梯度作为深度图示例
    // remove: for (int y = 0; y < 480; y += 2) {
    // remove:     for (int x = 0; x < 640; x += 2) {
    // remove:         int r = qMin(255, x * 255 / 640);
    // remove:         int g = qMin(255, y * 255 / 480);
    // remove:         int b = qMin(255, (x + y) * 255 / (640 + 480));
    // remove:         testDepth.setPixelColor(x, y, QColor(r, g, b));
    // remove:         testDepth.setPixelColor(x+1, y, QColor(r, g, b));
    // remove:         testDepth.setPixelColor(x, y+1, QColor(r, g, b));
    // remove:         testDepth.setPixelColor(x+1, y+1, QColor(r, g, b));
    // remove:     }
    // remove: }
    // remove: 
    // remove: // 保存测试图像
    // remove: depth_image_ = testDepth;
}

// 添加更新检测结果表格的函数
void ArmControlGUI::updateDetectionsTable()
{
    // 确保在主线程中更新UI
    QMetaObject::invokeMethod(this, "updateDetectionsTableUI", Qt::QueuedConnection);
}

// 在主线程中更新检测结果表格
void ArmControlGUI::updateDetectionsTableUI()
{
    if (!ui->detectionsTable) {
        return;  // 表格不存在，可能是旧版UI
    }
    
    // 阻止表格信号以避免触发不必要的回调
    ui->detectionsTable->blockSignals(true);
    
    // 清空表格
    ui->detectionsTable->setRowCount(0);
    
    // 设置表格列数
    ui->detectionsTable->setColumnCount(6);
    
    // 设置表头
    QStringList headers;
    headers << "ID" << "类型" << "X" << "Y" << "Z" << "操作";
    ui->detectionsTable->setHorizontalHeaderLabels(headers);
    
    // 填充检测结果
    for (size_t i = 0; i < detected_objects_.size(); ++i) {
        const DetectedObject& obj = detected_objects_[i];
        
        // 添加新行
        int row = ui->detectionsTable->rowCount();
        ui->detectionsTable->insertRow(row);
        
        // 设置ID
        QTableWidgetItem* idItem = new QTableWidgetItem(QString::fromStdString(obj.id));
        ui->detectionsTable->setItem(row, 0, idItem);
        
        // 设置类型
        QTableWidgetItem* typeItem = new QTableWidgetItem(QString::fromStdString(obj.type));
        ui->detectionsTable->setItem(row, 1, typeItem);
        
        // 设置X坐标
        QTableWidgetItem* xItem = new QTableWidgetItem(QString::number(obj.pose.position.x * 100, 'f', 1));
        ui->detectionsTable->setItem(row, 2, xItem);
        
        // 设置Y坐标
        QTableWidgetItem* yItem = new QTableWidgetItem(QString::number(obj.pose.position.y * 100, 'f', 1));
        ui->detectionsTable->setItem(row, 3, yItem);
        
        // 设置Z坐标
        QTableWidgetItem* zItem = new QTableWidgetItem(QString::number(obj.pose.position.z * 100, 'f', 1));
        ui->detectionsTable->setItem(row, 4, zItem);
        
        // 添加操作按钮
        QPushButton* pickButton = new QPushButton("抓取");
        pickButton->setProperty("object_id", QString::fromStdString(obj.id));
        connect(pickButton, &QPushButton::clicked, this, [this, id=obj.id]() {
            this->sendPickCommand(id);
        });
        ui->detectionsTable->setCellWidget(row, 5, pickButton);
    }
    
    // 调整列宽以适应内容
    ui->detectionsTable->resizeColumnsToContents();
    
    // 恢复表格信号
    ui->detectionsTable->blockSignals(false);
    
    // 如果没有检测结果，显示提示信息
    if (detected_objects_.empty()) {
        ui->detectionsTable->setRowCount(1);
        QTableWidgetItem* noDataItem = new QTableWidgetItem("无检测结果");
        noDataItem->setTextAlignment(Qt::AlignCenter);
        ui->detectionsTable->setSpan(0, 0, 1, 6);
        ui->detectionsTable->setItem(0, 0, noDataItem);
    }
} 