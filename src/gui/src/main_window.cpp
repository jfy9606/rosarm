#include "main_window.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QCloseEvent>
#include <QCheckBox>
#include <QTableWidget>
#include <QComboBox>
#include <QHeaderView>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/master.h>
#include "trajectory/kinematics_control.h"
#include <std_msgs/Float64MultiArray.h>

// 前向声明UI命名空间
namespace Ui {
    class MainWindow {};
}

MainWindow::MainWindow(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent), ui(nullptr), nh_(nh),
    vacuum_on_(false), vacuum_power_(50), selected_object_index_(-1),
    yolo_enabled_(false), camera_view_mode_(0), is_camera_available_(false),
    stereo_camera_error_count_(0), available_camera_index_(-1),
    ignore_slider_events_(false), ignore_spin_events_(false), ui_processing_(false),
    updateTimer(new QTimer(this))
{
    // 设置窗口标题和大小
    setWindowTitle("Robotic Arm Control System");
    resize(1024, 768);
    
    // 设置状态栏
    statusBar()->showMessage("Initializing robotic arm control interface...");
    
    // 创建占位图像 - 即使没有相机也先创建
    createPlaceholderImage("Waiting for camera connection...");
    
    // 初始化ROS
    initializeROS();
    
    // 初始化成员变量
    initializeMembers();
    
    // 初始化UI配置
    setupUi();
    
    // 初始化控件连接
    connectSignalSlots();
    
    // 设置关节限制
    setupJointLimits();
    
    // 初始化运动学工具
    kinematics_utils_ = new trajectory::KinematicsControl();
    
    // 创建主标签页控件
    QTabWidget* mainTabWidget = new QTabWidget(this);
    setCentralWidget(mainTabWidget);

    // 创建机械臂控制标签页
    QWidget* armControlTab = new QWidget();
    QVBoxLayout* armLayout = new QVBoxLayout(armControlTab);

    // 添加机械臂控制相关组件
    armLayout->addWidget(createArmControlPanel());
    armLayout->addWidget(createJointControlPanel());
    armLayout->addWidget(createPositionControlPanel());

    // 创建视觉系统标签页
    QWidget* visionTab = new QWidget();
    QVBoxLayout* visionLayout = new QVBoxLayout(visionTab);

    // 添加视觉系统相关组件
    visionLayout->addWidget(createCameraViewPanel());
    visionLayout->addWidget(createObjectDetectionPanel());
    visionLayout->addWidget(createDepthVisualizationPanel());

    // 创建桥接控制标签页
    QWidget* bridgeTab = new QWidget();
    QVBoxLayout* bridgeLayout = new QVBoxLayout(bridgeTab);

    // 添加桥接控制相关组件
    bridgeLayout->addWidget(createVisualServoBridgePanel());

    // 将标签页添加到主标签页控件
    mainTabWidget->addTab(armControlTab, tr("机械臂控制"));
    mainTabWidget->addTab(visionTab, tr("视觉系统"));
    mainTabWidget->addTab(bridgeTab, tr("系统桥接"));
    
    // 启动定时器
    updateTimer->start(33);  // 30FPS
    
    // 设置相机重连定时器
    connect(&camera_reconnect_timer_, &QTimer::timeout, this, &MainWindow::attemptCameraReconnect);
    camera_reconnect_timer_.start(5000);  // 每5秒尝试重连相机
    
    // 将界面设置为初始状态
    updateGUIJointValues();
    
    // 更新UI
    updateUI();
    
    // 更新相机视图（使用占位图像）
    updateCameraView();
    
    statusBar()->showMessage("机械臂控制界面已启动");
}

MainWindow::~MainWindow()
{
    if (kinematics_utils_) {
        delete kinematics_utils_;
    }
    if (ui) {
        delete ui;
    }
}

void MainWindow::initializeROS()
{
    try {
        // 检查ROS是否正在运行并已连接
        if (!ros::master::check()) {
            ROS_ERROR("Cannot connect to ROS master. Is roscore running?");
            QMessageBox::critical(this, "ROS连接错误", 
                                 "无法连接到ROS主节点。请确保roscore正在运行。");
            return;
        }

        // 初始化发布者
        joint_command_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_command", 1);
        vacuum_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/vacuum_command", 1);
        vacuum_power_pub_ = nh_.advertise<std_msgs::Int32>("/vacuum_power", 1);
        camera_view_mode_pub_ = nh_.advertise<std_msgs::Int32>("/stereo_camera/view_mode", 1);
        
        // 初始化服务客户端
        joint_control_client_ = nh_.serviceClient<servo::JointControl>("/joint_control");
        vacuum_control_client_ = nh_.serviceClient<servo::VacuumCmd>("/vacuum_control");
        
        // 设置ROS订阅
        setupROSSubscriptions();
        
        ROS_INFO("ROS initialization complete");
    }
    catch (const std::exception& e) {
        ROS_ERROR("ROS initialization failed: %s", e.what());
        QMessageBox::critical(this, "ROS初始化错误", 
                             QString("ROS初始化失败: %1").arg(e.what()));
    }
}

void MainWindow::initializeMembers()
{
    // 初始化相机参数
    is_camera_available_ = false;
    stereo_camera_error_count_ = 0;
    camera_view_mode_ = 0;
    available_camera_index_ = -1;
    depth_available_ = false;
    
    // 初始化YOLO检测相关变量
    yolo_enabled_ = false;
    show_detection_boxes_ = true;
    show_distance_overlay_ = true;
    current_detection_model_ = "yolo11n";
    
    // 初始化末端执行器状态
    current_end_position_ = QVector3D(30.0f, 0.0f, 30.0f); // 初始位置（厘米）
    current_end_orientation_ = QQuaternion(1.0f, 0.0f, 0.0f, 0.0f); // 初始方向
    
    // 初始化机械臂关节状态
    current_joint_values_ = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // 初始化真空吸盘状态
    vacuum_on_ = false;
    vacuum_power_ = 50;
    
    // 初始化选中对象
    selected_object_index_ = -1;
    
    // 初始化UI标志
    ignore_slider_events_ = false;
    ignore_spin_events_ = false;
    ui_processing_ = false;
    
    // 创建占位图像
    createPlaceholderImage("等待相机连接...");
    
    // 连接相机重连定时器
    connect(&camera_reconnect_timer_, &QTimer::timeout, this, &MainWindow::attemptCameraReconnect);
    camera_reconnect_timer_.start(5000); // 每5秒尝试重连一次
}

void MainWindow::setupUi()
{
    // 创建关节控制部件
    joint1Slider = new QSlider(Qt::Horizontal);
    joint2Slider = new QSlider(Qt::Horizontal);
    joint3Slider = new QSlider(Qt::Horizontal);
    joint4Slider = new QSlider(Qt::Horizontal);
    joint5Slider = new QSlider(Qt::Horizontal);
    joint6Slider = new QSlider(Qt::Horizontal);
    
    joint1Spin = new QDoubleSpinBox();
    joint2Spin = new QDoubleSpinBox();
    joint3Spin = new QDoubleSpinBox();
    joint4Spin = new QDoubleSpinBox();
    joint5Spin = new QDoubleSpinBox();
    joint6Spin = new QDoubleSpinBox();
    
    // 设置关节控件范围
    joint1Slider->setRange(-180, 180);
    joint2Slider->setRange(0, 50);
    joint3Slider->setRange(-90, 90);
    joint4Slider->setRange(0, 180);
    joint5Slider->setRange(-90, 90);
    joint6Slider->setRange(5, 15);
    
    joint1Spin->setRange(-180.0, 180.0);
    joint2Spin->setRange(0.0, 50.0);
    joint3Spin->setRange(-90.0, 90.0);
    joint4Spin->setRange(0.0, 180.0);
    joint5Spin->setRange(-90.0, 90.0);
    joint6Spin->setRange(5.0, 15.0);
    
    // 创建末端执行器控制部件
    posXSpin = new QDoubleSpinBox();
    posYSpin = new QDoubleSpinBox();
    posZSpin = new QDoubleSpinBox();
    moveToPositionButton = new QPushButton("移动到位置");
    endEffectorStatusLabel = new QLabel("末端位置: 等待更新...");
    jointStatusLabel = new QLabel("关节状态: 等待更新...");
    
    // 设置末端执行器控件范围
    posXSpin->setRange(-50, 50);
    posYSpin->setRange(-50, 50);
    posZSpin->setRange(0, 50);
    posXSpin->setValue(30);
    posYSpin->setValue(0);
    posZSpin->setValue(20);
    
    // 创建真空吸盘控制部件
    vacuumOnButton = new QPushButton("开启");
    vacuumOffButton = new QPushButton("关闭");
    vacuumPowerSlider = new QSlider(Qt::Horizontal);
    vacuumPowerSlider->setRange(0, 100);
    vacuumPowerSlider->setValue(50);
    
    // 创建相机视图部件
    cameraView = new QLabel("相机视图");
    cameraView->setMinimumSize(640, 480);
    cameraView->setAlignment(Qt::AlignCenter);
    cameraView->setStyleSheet("background-color: #333333; color: white;");
    
    leftViewButton = new QPushButton("左视图");
    rightViewButton = new QPushButton("右视图");
    
    // 创建其他控制部件
    homeButton = new QPushButton("回到初始位置");
}

void MainWindow::connectSignalSlots()
{
    // 连接定时器
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::updateUI);
    
    // 连接关节滑块和微调框
    connect(joint1Slider, &QSlider::valueChanged, this, &MainWindow::onJoint1SliderChanged);
    connect(joint2Slider, &QSlider::valueChanged, this, &MainWindow::onJoint2SliderChanged);
    connect(joint3Slider, &QSlider::valueChanged, this, &MainWindow::onJoint3SliderChanged);
    connect(joint4Slider, &QSlider::valueChanged, this, &MainWindow::onJoint4SliderChanged);
    connect(joint5Slider, &QSlider::valueChanged, this, &MainWindow::onJoint5SliderChanged);
    connect(joint6Slider, &QSlider::valueChanged, this, &MainWindow::onJoint6SliderChanged);
    
    connect(joint1Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onJoint1SpinChanged);
    connect(joint2Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onJoint2SpinChanged);
    connect(joint3Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onJoint3SpinChanged);
    connect(joint4Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onJoint4SpinChanged);
    connect(joint5Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onJoint5SpinChanged);
    connect(joint6Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onJoint6SpinChanged);
    
    // 连接真空吸盘控制
    connect(vacuumOnButton, &QPushButton::clicked, this, &MainWindow::onVacuumOnButtonClicked);
    connect(vacuumOffButton, &QPushButton::clicked, this, &MainWindow::onVacuumOffButtonClicked);
    connect(vacuumPowerSlider, &QSlider::valueChanged, this, &MainWindow::onVacuumPowerSliderChanged);
    
    // 连接相机视图控制
    connect(leftViewButton, &QPushButton::clicked, this, &MainWindow::onLeftViewButtonClicked);
    connect(rightViewButton, &QPushButton::clicked, this, &MainWindow::onRightViewButtonClicked);
    
    // 添加缺少的立体相机和深度视图按钮连接
    if (stereoViewButton) {
        connect(stereoViewButton, &QPushButton::clicked, this, &MainWindow::onStereoViewButtonClicked);
    }
    if (depthViewButton) {
        connect(depthViewButton, &QPushButton::clicked, this, &MainWindow::onDepthViewButtonClicked);
    }
    
    // 连接位置控制
    connect(moveToPositionButton, &QPushButton::clicked, this, &MainWindow::onMoveToPositionClicked);
    connect(homeButton, &QPushButton::clicked, this, &MainWindow::onHomeButtonClicked);
}

void MainWindow::setupROSSubscriptions()
{
    try {
        // 关节状态订阅
        joint_state_sub_ = nh_.subscribe("/joint_states", 10, 
                                       &MainWindow::jointStateCallback, this);
        
        // 相机图像订阅
        stereo_merged_sub_ = nh_.subscribe("/stereo_camera/merged_image", 1, 
                                         &MainWindow::stereoMergedCallback, this);
        depth_image_sub_ = nh_.subscribe("/stereo_camera/depth", 1, 
                                       &MainWindow::depthImageCallback, this);
        
        // 同步订阅
        detection_image_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(
            nh_, "/yolo_detector/detection_image", 1);
        detection_poses_sync_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::PoseArray>>(
            nh_, "/yolo_detector/detection_poses", 1);
        
        // 设置同步策略
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *detection_image_sync_sub_, *detection_poses_sync_sub_);
        
        // 注册同步回调
        sync_->registerCallback(boost::bind(&MainWindow::objectDetectionCallback, 
                                         this, _1, _2));
        
        // 发布器
        camera_view_mode_pub_ = nh_.advertise<std_msgs::Int32>("/stereo_camera/view_mode", 1, true);
        
        ROS_INFO("ROS subscriptions setup complete");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to setup ROS subscriptions: %s", e.what());
        // 显示错误对话框但不阻止程序继续运行
        QMessageBox::warning(this, "ROS订阅警告", 
                            QString("设置ROS订阅时出现问题: %1\n部分功能可能不可用.").arg(e.what()));
    }
}

// UI面板创建方法
QWidget* MainWindow::createArmControlPanel()
{
    QGroupBox* armControlGroup = new QGroupBox("机械臂控制");
    QVBoxLayout* layout = new QVBoxLayout(armControlGroup);
    
    // 添加回到初始位置按钮
    layout->addWidget(homeButton);
    
    // 添加关节状态标签
    layout->addWidget(jointStatusLabel);
    
    return armControlGroup;
}

QWidget* MainWindow::createJointControlPanel()
{
    QGroupBox* jointControlGroup = new QGroupBox("关节控制");
    QVBoxLayout* layout = new QVBoxLayout(jointControlGroup);
    
    // 创建滚动区域以容纳所有关节控制
    QScrollArea* scrollArea = new QScrollArea();
    scrollArea->setWidgetResizable(true);
    QWidget* scrollContent = new QWidget();
    QVBoxLayout* scrollLayout = new QVBoxLayout(scrollContent);
    
    // 添加关节1控制
    QGroupBox* joint1Group = new QGroupBox("关节1");
    QVBoxLayout* joint1Layout = new QVBoxLayout(joint1Group);
    joint1Layout->addWidget(joint1Slider);
    QHBoxLayout* j1SpinLayout = new QHBoxLayout();
    j1SpinLayout->addWidget(joint1Spin);
    j1SpinLayout->addWidget(new QLabel("度"));
    joint1Layout->addLayout(j1SpinLayout);
    scrollLayout->addWidget(joint1Group);
    
    // 添加关节2控制
    QGroupBox* joint2Group = new QGroupBox("关节2");
    QVBoxLayout* joint2Layout = new QVBoxLayout(joint2Group);
    joint2Layout->addWidget(joint2Slider);
    QHBoxLayout* j2SpinLayout = new QHBoxLayout();
    j2SpinLayout->addWidget(joint2Spin);
    j2SpinLayout->addWidget(new QLabel("度"));
    joint2Layout->addLayout(j2SpinLayout);
    scrollLayout->addWidget(joint2Group);
    
    // 添加关节3控制
    QGroupBox* joint3Group = new QGroupBox("关节3");
    QVBoxLayout* joint3Layout = new QVBoxLayout(joint3Group);
    joint3Layout->addWidget(joint3Slider);
    QHBoxLayout* j3SpinLayout = new QHBoxLayout();
    j3SpinLayout->addWidget(joint3Spin);
    j3SpinLayout->addWidget(new QLabel("度"));
    joint3Layout->addLayout(j3SpinLayout);
    scrollLayout->addWidget(joint3Group);
    
    // 添加关节4控制
    QGroupBox* joint4Group = new QGroupBox("关节4");
    QVBoxLayout* joint4Layout = new QVBoxLayout(joint4Group);
    joint4Layout->addWidget(joint4Slider);
    QHBoxLayout* j4SpinLayout = new QHBoxLayout();
    j4SpinLayout->addWidget(joint4Spin);
    j4SpinLayout->addWidget(new QLabel("度"));
    joint4Layout->addLayout(j4SpinLayout);
    scrollLayout->addWidget(joint4Group);
    
    // 添加关节5控制
    QGroupBox* joint5Group = new QGroupBox("关节5");
    QVBoxLayout* joint5Layout = new QVBoxLayout(joint5Group);
    joint5Layout->addWidget(joint5Slider);
    QHBoxLayout* j5SpinLayout = new QHBoxLayout();
    j5SpinLayout->addWidget(joint5Spin);
    j5SpinLayout->addWidget(new QLabel("度"));
    joint5Layout->addLayout(j5SpinLayout);
    scrollLayout->addWidget(joint5Group);
    
    // 添加关节6控制
    QGroupBox* joint6Group = new QGroupBox("关节6");
    QVBoxLayout* joint6Layout = new QVBoxLayout(joint6Group);
    joint6Layout->addWidget(joint6Slider);
    QHBoxLayout* j6SpinLayout = new QHBoxLayout();
    j6SpinLayout->addWidget(joint6Spin);
    j6SpinLayout->addWidget(new QLabel("度"));
    joint6Layout->addLayout(j6SpinLayout);
    scrollLayout->addWidget(joint6Group);
    
    scrollArea->setWidget(scrollContent);
    layout->addWidget(scrollArea);
    
    return jointControlGroup;
}

QWidget* MainWindow::createPositionControlPanel()
{
    QGroupBox* positionGroup = new QGroupBox("位置控制");
    QVBoxLayout* layout = new QVBoxLayout(positionGroup);
    
    // 添加末端执行器位置控制
    QGridLayout* posGrid = new QGridLayout();
    posGrid->addWidget(new QLabel("X 位置 (cm):"), 0, 0);
    posGrid->addWidget(posXSpin, 0, 1);
    posGrid->addWidget(new QLabel("Y 位置 (cm):"), 1, 0);
    posGrid->addWidget(posYSpin, 1, 1);
    posGrid->addWidget(new QLabel("Z 位置 (cm):"), 2, 0);
    posGrid->addWidget(posZSpin, 2, 1);
    layout->addLayout(posGrid);
    
    // 添加移动按钮
    layout->addWidget(moveToPositionButton);
    
    // 添加末端执行器状态标签
    layout->addWidget(endEffectorStatusLabel);
    
    // 添加真空吸盘控制
    QGroupBox* vacuumGroup = new QGroupBox("真空吸盘控制");
    QVBoxLayout* vacuumLayout = new QVBoxLayout(vacuumGroup);
    
    QHBoxLayout* vacuumBtnLayout = new QHBoxLayout();
    vacuumBtnLayout->addWidget(vacuumOnButton);
    vacuumBtnLayout->addWidget(vacuumOffButton);
    vacuumLayout->addLayout(vacuumBtnLayout);
    
    QHBoxLayout* vacuumPowerLayout = new QHBoxLayout();
    vacuumPowerLayout->addWidget(new QLabel("功率:"));
    vacuumPowerLayout->addWidget(vacuumPowerSlider);
    vacuumLayout->addLayout(vacuumPowerLayout);
    
    layout->addWidget(vacuumGroup);
    
    return positionGroup;
}

QWidget* MainWindow::createCameraViewPanel()
{
    QGroupBox* cameraGroup = new QGroupBox("相机视图");
    QVBoxLayout* layout = new QVBoxLayout(cameraGroup);
    
    // 添加相机视图
    layout->addWidget(cameraView);
    
    // 添加相机控制按钮
    QHBoxLayout* cameraBtnLayout = new QHBoxLayout();
    cameraBtnLayout->addWidget(leftViewButton);
    cameraBtnLayout->addWidget(rightViewButton);
    
    // 创建并添加缺少的立体视图和深度视图按钮
    stereoViewButton = new QPushButton("立体视图");
    depthViewButton = new QPushButton("深度视图");
    cameraBtnLayout->addWidget(stereoViewButton);
    cameraBtnLayout->addWidget(depthViewButton);
    
    layout->addLayout(cameraBtnLayout);
    
    return cameraGroup;
}

QWidget* MainWindow::createObjectDetectionPanel()
{
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    
    QCheckBox* enableDetectionCheck = new QCheckBox("启用物体检测");
    layout->addWidget(enableDetectionCheck);
    
    // 创建检测表格
    QTableWidget* detectionTable = new QTableWidget(0, 3);
    detectionTable->setHorizontalHeaderLabels(QStringList() << "物体" << "置信度" << "距离(cm)");
    detectionTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    detectionTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    detectionTable->horizontalHeader()->setStretchLastSection(true);
    layout->addWidget(detectionTable);
    
    return panel;
}

QWidget* MainWindow::createDepthVisualizationPanel()
{
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    
    QCheckBox* showDistanceCheck = new QCheckBox("显示距离信息");
    showDistanceCheck->setChecked(true);
    layout->addWidget(showDistanceCheck);
    
    return panel;
}

QWidget* MainWindow::createVisualServoBridgePanel()
{
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    
    QCheckBox* enableBridgeCheck = new QCheckBox("启用视觉伺服桥接");
    layout->addWidget(enableBridgeCheck);
    
    // 桥接模式选择
    QComboBox* bridgeModeCombo = new QComboBox();
    bridgeModeCombo->addItem("位置跟踪模式");
    bridgeModeCombo->addItem("抓取模式");
    bridgeModeCombo->addItem("放置模式");
    layout->addWidget(bridgeModeCombo);
    
    // 目标对象选择
    QComboBox* targetObjectCombo = new QComboBox();
    targetObjectCombo->addItem("最近对象");
    targetObjectCombo->addItem("指定物体 1");
    targetObjectCombo->addItem("指定物体 2");
    layout->addWidget(targetObjectCombo);
    
    return panel;
}

// 更新UI
void MainWindow::updateUI()
{
    // 更新末端执行器位置显示
    updateEndEffectorPose();
    
    // 更新吸附状态
    updateVacuumStatus();
}

void MainWindow::updateVacuumStatus()
{
    // 更新真空吸盘状态指示
    if (vacuum_on_) {
        vacuumOnButton->setStyleSheet("QPushButton { background-color: green; }");
        vacuumOffButton->setStyleSheet("");
    } else {
        vacuumOnButton->setStyleSheet("");
        vacuumOffButton->setStyleSheet("QPushButton { background-color: red; }");
    }
    
    // 更新吸盘功率滑块
    vacuumPowerSlider->setValue(vacuum_power_);
}

void MainWindow::updateGUIJointValues()
{
    if (ignore_slider_events_)
        return;
    
    // 阻止信号触发，避免循环调用
    ignore_slider_events_ = true;
    ignore_spin_events_ = true;
    
    // 将弧度转换为角度
    double joint1_deg = radToDeg(current_joint_values_[0]);
    double joint3_deg = radToDeg(current_joint_values_[2]);
    double joint4_deg = radToDeg(current_joint_values_[3]);
    double joint5_deg = radToDeg(current_joint_values_[4]);
    
    // 更新滑块值
    joint1Slider->setValue(static_cast<int>(joint1_deg));
    joint2Slider->setValue(static_cast<int>(current_joint_values_[1]));
    joint3Slider->setValue(static_cast<int>(joint3_deg));
    joint4Slider->setValue(static_cast<int>(joint4_deg));
    joint5Slider->setValue(static_cast<int>(joint5_deg));
    joint6Slider->setValue(static_cast<int>(current_joint_values_[5]));
    
    // 更新微调框值
    joint1Spin->setValue(joint1_deg);
    joint2Spin->setValue(current_joint_values_[1]);
    joint3Spin->setValue(joint3_deg);
    joint4Spin->setValue(joint4_deg);
    joint5Spin->setValue(joint5_deg);
    joint6Spin->setValue(current_joint_values_[5]);
    
    // 更新关节状态标签
    QString statusText = QString("关节状态: J1:%1° J2:%2 J3:%3° J4:%4° J5:%5° J6:%6")
                       .arg(joint1_deg, 0, 'f', 1)
                       .arg(current_joint_values_[1], 0, 'f', 1)
                       .arg(joint3_deg, 0, 'f', 1)
                       .arg(joint4_deg, 0, 'f', 1)
                       .arg(joint5_deg, 0, 'f', 1)
                       .arg(current_joint_values_[5], 0, 'f', 1);
    jointStatusLabel->setText(statusText);
    
    ignore_slider_events_ = false;
    ignore_spin_events_ = false;
}

// 关节控制槽函数
void MainWindow::onJoint1SliderChanged(int value)
{
    if (ignore_slider_events_)
        return;
    
    // 将滑块值转换为关节值
    double joint_value = value; // 这里直接使用滑块值作为角度
    
    // 更新微调框（不触发微调框的事件）
    ignore_spin_events_ = true;
    joint1Spin->setValue(joint_value);
    ignore_spin_events_ = false;
    
    // 更新关节值并立即发送命令
    current_joint_values_[0] = degToRad(joint_value);
    sendJointCommand(current_joint_values_);
}

void MainWindow::onJoint2SliderChanged(int value)
{
    if (ignore_slider_events_)
        return;
    
    // 将滑块值直接用作关节值（毫米）
    double joint_value = value;
    
    // 更新微调框（不触发微调框的事件）
    ignore_spin_events_ = true;
    joint2Spin->setValue(joint_value);
    ignore_spin_events_ = false;
    
    // 更新关节值并立即发送命令
    current_joint_values_[1] = joint_value;
    sendJointCommand(current_joint_values_);
}

void MainWindow::onJoint3SliderChanged(int value)
{
    if (ignore_slider_events_)
        return;
    
    // 将滑块值转换为关节值
    double joint_value = value; // 角度
    
    // 更新微调框（不触发微调框的事件）
    ignore_spin_events_ = true;
    joint3Spin->setValue(joint_value);
    ignore_spin_events_ = false;
    
    // 更新关节值并立即发送命令
    current_joint_values_[2] = degToRad(joint_value);
    sendJointCommand(current_joint_values_);
}

void MainWindow::onJoint4SliderChanged(int value)
{
    if (ignore_slider_events_)
        return;
    
    // 将滑块值转换为关节值
    double joint_value = value; // 角度
    
    // 更新微调框（不触发微调框的事件）
    ignore_spin_events_ = true;
    joint4Spin->setValue(joint_value);
    ignore_spin_events_ = false;
    
    // 更新关节值并立即发送命令
    current_joint_values_[3] = degToRad(joint_value);
    sendJointCommand(current_joint_values_);
}

void MainWindow::onJoint5SliderChanged(int value)
{
    if (ignore_slider_events_)
        return;
    
    // 将滑块值转换为关节值
    double joint_value = value; // 角度
    
    // 更新微调框（不触发微调框的事件）
    ignore_spin_events_ = true;
    joint5Spin->setValue(joint_value);
    ignore_spin_events_ = false;
    
    // 更新关节值并立即发送命令
    current_joint_values_[4] = degToRad(joint_value);
    sendJointCommand(current_joint_values_);
}

void MainWindow::onJoint6SliderChanged(int value)
{
    if (ignore_slider_events_)
        return;
    
    // 将滑块值直接用作关节值（毫米）
    double joint_value = value;
    
    // 更新微调框（不触发微调框的事件）
    ignore_spin_events_ = true;
    joint6Spin->setValue(joint_value);
    ignore_spin_events_ = false;
    
    // 更新关节值并立即发送命令
    current_joint_values_[5] = joint_value;
    sendJointCommand(current_joint_values_);
}

// 关节微调框槽函数
void MainWindow::onJoint1SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        joint1Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        double radians = degToRad(value);
        std::vector<double> new_joints = current_joint_values_;
        new_joints[0] = radians;
        
        sendJointCommand(new_joints);
    }
}

void MainWindow::onJoint2SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        joint2Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        std::vector<double> new_joints = current_joint_values_;
        new_joints[1] = value;
        
        sendJointCommand(new_joints);
    }
}

void MainWindow::onJoint3SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        joint3Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        double radians = degToRad(value);
        std::vector<double> new_joints = current_joint_values_;
        new_joints[2] = radians;
        
        sendJointCommand(new_joints);
    }
}

void MainWindow::onJoint4SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        joint4Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        double radians = degToRad(value);
        std::vector<double> new_joints = current_joint_values_;
        new_joints[3] = radians;
        
        sendJointCommand(new_joints);
    }
}

void MainWindow::onJoint5SpinChanged(double value)
{
    if (ignore_spin_events_)
        return;
    
    ignore_slider_events_ = true;
    joint5Slider->setValue(static_cast<int>(value));
    ignore_slider_events_ = false;
    
    // 将角度转换为弧度
    double angle_rad = degToRad(value);
    
    // 更新关节值
    current_joint_values_[4] = angle_rad;
    
    // 发送关节命令
    sendJointCommand();
}

void MainWindow::onJoint6SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        joint6Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        std::vector<double> new_joints = current_joint_values_;
        new_joints[5] = value;
        
        sendJointCommand(new_joints);
    }
}

// 真空吸盘控制槽函数
void MainWindow::onVacuumPowerSliderChanged(int value)
{
    if (vacuum_on_) {
        sendVacuumCommand(true, value);
    }
    vacuum_power_ = value;
}

void MainWindow::onVacuumOnButtonClicked()
{
    sendVacuumCommand(true, vacuum_power_);
}

void MainWindow::onVacuumOffButtonClicked()
{
    sendVacuumCommand(false, vacuum_power_);
}

// 相机视图控制槽函数
void MainWindow::onLeftViewButtonClicked()
{
    camera_view_mode_ = 0;  // 设置为左视图模式
    ROS_INFO("切换到左视图");
    
    // 更新按钮状态
    leftViewButton->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
    rightViewButton->setStyleSheet("");
    
    // 发布相机视图模式
    std_msgs::Int32 mode_msg;
    mode_msg.data = camera_view_mode_;
    if (camera_view_mode_pub_) {
        camera_view_mode_pub_.publish(mode_msg);
    }
    
    // 更新显示
    updateCameraView();
}

void MainWindow::onRightViewButtonClicked()
{
    camera_view_mode_ = 1;  // 设置为右视图模式
    ROS_INFO("切换到右视图");
    
    // 更新按钮状态
    rightViewButton->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
    leftViewButton->setStyleSheet("");
    
    // 发布相机视图模式
    std_msgs::Int32 mode_msg;
    mode_msg.data = camera_view_mode_;
    if (camera_view_mode_pub_) {
        camera_view_mode_pub_.publish(mode_msg);
    }
    
    // 更新显示
    updateCameraView();
}

// 位置控制槽函数
void MainWindow::onMoveToPositionClicked()
{
    try {
        // 获取输入坐标（单位：cm）
        double x = posXSpin->value();
        double y = posYSpin->value();
        double z = posZSpin->value();
        
        // 记录到日志
        logMessage(QString("移动到位置 X:%1 Y:%2 Z:%3 cm").arg(x).arg(y).arg(z));
        
        // 转换为米（ROS单位）
        x /= 100.0;
        y /= 100.0;
        z /= 100.0;
        
        // 创建目标位姿
        geometry_msgs::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.w = 1.0;  // 默认方向
        
        // 使用逆运动学计算关节值
        std::vector<double> target_joints = inverseKinematics(target_pose, current_joint_values_);
        
        // 检查是否得到有效解
        if (target_joints.empty()) {
            logMessage("无法计算到指定位置的逆运动学解");
            return;
        }
        
        // 检查关节限制
        if (!checkJointLimits(target_joints)) {
            logMessage("目标位置超出机械臂关节限制范围");
            return;
        }
        
        // 保存新关节值
        current_joint_values_ = target_joints;
        
        // 更新GUI
        updateGUIJointValues();
        
        // 发送关节命令
        sendJointCommand(target_joints);
        
        logMessage("移动命令已发送");
    } catch (const std::exception& e) {
        logMessage(QString("移动失败: %1").arg(e.what()));
    }
}

void MainWindow::onHomeButtonClicked()
{
    sendHomeCommand();
}

// 相机重连槽函数
void MainWindow::attemptCameraReconnect()
{
    // 查找可用相机
    if (!is_camera_available_) {
    if (findAvailableCamera()) {
        is_camera_available_ = true;
            stereo_camera_error_count_ = 0;
            camera_reconnect_timer_.setInterval(5000);  // 恢复正常检查间隔
            ROS_INFO("Camera reconnected successfully.");
            QString message = "Camera reconnected successfully";
            createPlaceholderImage(message);
        } else {
            stereo_camera_error_count_++;
            // 如果长时间无法连接相机，降低重连频率
            if (stereo_camera_error_count_ > 12) {  // 1分钟后
                camera_reconnect_timer_.setInterval(30000);  // 30秒检查一次
            }
            QString message = "Waiting for camera connection... (" + QString::number(stereo_camera_error_count_) + ")";
            createPlaceholderImage(message);
        }
    }
}

// ROS回调函数
void MainWindow::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // 检查消息是否有效
    if (msg->position.size() < 6) {
        return;
    }
    
    // 更新当前关节值
    for (size_t i = 0; i < std::min(msg->position.size(), current_joint_values_.size()); ++i) {
        current_joint_values_[i] = msg->position[i];
    }
    
    // 标记机械臂准备就绪
    arm_ready_ = true;
    
    // 更新GUI（在主线程中）
    QMetaObject::invokeMethod(this, "updateGUIJointValues", Qt::QueuedConnection);
}

// 辅助函数
void MainWindow::sendJointCommand()
{
    sendJointCommand(current_joint_values_);
}

void MainWindow::sendJointCommand(const std::vector<double>& joint_values)
{
    if (joint_values.size() < 6) {
        ROS_ERROR("Invalid joint values size");
        return;
    }
    
    // 创建关节命令消息
    std_msgs::Float64MultiArray joint_cmd;
    joint_cmd.data = joint_values;
    
    // 发布关节命令
    joint_command_pub_.publish(joint_cmd);
    
    ROS_INFO("Joint command sent");
}

void MainWindow::sendVacuumCommand(bool on, int power)
{
    // 发布消息（保持向后兼容）
    std_msgs::Bool on_msg;
    on_msg.data = on;
    vacuum_cmd_pub_.publish(on_msg);
    
    std_msgs::Int32 power_msg;
    power_msg.data = power;
    vacuum_power_pub_.publish(power_msg);
    
    // 使用服务调用
    if (vacuum_control_client_.exists()) {
        servo::VacuumCmd srv;
        srv.request.enable = on;
        srv.request.power = power;
        
        if (vacuum_control_client_.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("真空吸盘控制成功: %s, 功率: %d%%", on ? "开启" : "关闭", power);
            } else {
                ROS_WARN("真空吸盘控制失败: %s", srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("调用真空吸盘控制服务失败");
        }
    } else {
        ROS_WARN("真空吸盘控制服务不可用，仅使用消息发布");
    }
    
    vacuum_on_ = on;
    vacuum_power_ = power;
}

void MainWindow::sendHomeCommand()
{
    // 通过发布消息方式实现回到初始位置
    std_msgs::Bool home_msg;
    home_msg.data = true;
    nh_.advertise<std_msgs::Bool>("/home_position", 1).publish(home_msg);
    
    // 使用服务调用（如果可用）
    ros::ServiceClient homeClient = nh_.serviceClient<servo::HomePosition>("/home_position");
    if (homeClient.exists()) {
        servo::HomePosition srv;
        if (homeClient.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("回到初始位置成功");
            } else {
                ROS_WARN("回到初始位置失败: %s", srv.response.message.c_str());
            }
        } else {
            ROS_ERROR("调用回到初始位置服务失败");
        }
    } else {
        ROS_INFO("回到初始位置服务不可用，仅使用消息发布");
        
        // 使用预定义的初始位置
        std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0, 0.0, 10.0};
        sendJointCommand(home_position);
    }
}

void MainWindow::goToHomePosition()
{
    logMessage("正在将机械臂移动到初始位置...");
    sendHomeCommand();
}

void MainWindow::logMessage(const QString& message)
{
    // 记录日志到状态栏
    statusBar()->showMessage(message, 3000);  // 显示3秒
    
    // 同时输出到ROS日志
    ROS_INFO("%s", message.toStdString().c_str());
}

// 运动学计算函数
double MainWindow::degToRad(double deg)
{
    // 直接实现角度到弧度的转换
    return deg * M_PI / 180.0;
}

double MainWindow::radToDeg(double rad)
{
    // 直接实现弧度到角度的转换
    return rad * 180.0 / M_PI;
}

geometry_msgs::Pose MainWindow::forwardKinematics(const std::vector<double>& joint_values)
{
    if (!kinematics_utils_) {
        // Fallback if kinematics_utils_ is not initialized
        geometry_msgs::Pose default_pose;
        default_pose.position.x = 0.3; // 默认30cm前方
        default_pose.position.y = 0.0;
        default_pose.position.z = 0.3; // 默认30cm高度
        default_pose.orientation.w = 1.0;
        default_pose.orientation.x = 0.0;
        default_pose.orientation.y = 0.0;
        default_pose.orientation.z = 0.0;
        
        // 记录警告
        ROS_WARN("运动学工具未初始化，使用默认位姿");
        
        return default_pose;
    }
    
    try {
        return kinematics_utils_->forwardKinematics(joint_values);
    } catch (const std::exception& e) {
        // 捕获可能的异常并提供默认值
        ROS_ERROR("正向运动学计算失败: %s", e.what());
        
        geometry_msgs::Pose default_pose;
        default_pose.position.x = 0.3;
        default_pose.position.y = 0.0;
        default_pose.position.z = 0.3;
        default_pose.orientation.w = 1.0;
        default_pose.orientation.x = 0.0;
        default_pose.orientation.y = 0.0;
        default_pose.orientation.z = 0.0;
        
        return default_pose;
    }
}

std::vector<double> MainWindow::inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                               const std::vector<double>& initial_guess)
{
    if (!kinematics_utils_) {
        // Fallback if kinematics_utils_ is not initialized
        ROS_ERROR("Kinematics utilities not initialized, cannot perform inverse kinematics");
        return std::vector<double>();
    }
    return kinematics_utils_->inverseKinematics(target_pose, initial_guess);
}

bool MainWindow::checkJointLimits(const std::vector<double>& joint_values)
{
    if (!kinematics_utils_) {
        // Fallback if kinematics_utils_ is not initialized
        ROS_ERROR("Kinematics utilities not initialized, cannot check joint limits");
        return false;
    }
    return kinematics_utils_->checkJointLimits(joint_values);
}

void MainWindow::setupJointLimits()
{
    if (kinematics_utils_) {
        kinematics_utils_->setupJointLimits("");
    }
}

bool MainWindow::findAvailableCamera()
{
    // 这个方法会检查相机的可用性
    // 在ROS系统中，我们依赖于相机节点，所以这里只是检查是否收到了图像消息
    
    // 如果最近收到了图像，则认为相机可用
    if (!left_camera_image_.isNull() || !right_camera_image_.isNull() || !current_depth_image_.isNull()) {
        return true;
    }
    
    // 否则，检查是否有相机设备存在
    // 注意：这里只是简单检查设备文件，实际上并不保证相机可用
    bool device_exists = false;
    
    // 检查常见的相机设备
    std::vector<std::string> devices = {"/dev/video0", "/dev/video1", "/dev/video2"};
    for (const auto& device : devices) {
        if (access(device.c_str(), F_OK) != -1) {
            ROS_INFO_THROTTLE(10, "相机设备存在: %s", device.c_str());
            device_exists = true;
            break;
        }
    }
    
    if (!device_exists) {
        ROS_INFO_THROTTLE(10, "未找到相机设备，GUI将使用占位图像");
    }
    
    // 即使设备不存在，我们也允许GUI继续工作，只是显示占位图像
    return device_exists;
}

// 事件处理
bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
    // 简单实现，不做任何特殊处理
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    // 关闭窗口时的处理
    QMainWindow::closeEvent(event);
}

void MainWindow::estimateObjectDistances(const std::vector<DetectedObject>& objects, 
                                      const cv::Mat& depth_map)
{
    if (depth_map.empty() || objects.empty()) {
        return;
    }
    
    // 深度图类型检查
    bool is_float_depth = (depth_map.type() == CV_32FC1);
    bool is_uint16_depth = (depth_map.type() == CV_16UC1);
    
    if (!is_float_depth && !is_uint16_depth) {
        ROS_WARN("不支持的深度图类型: %d", depth_map.type());
        return;
    }
    
    // 深度转换比例
    double depth_scale = 1.0; // 对于32FC1格式，单位为米
    if (is_uint16_depth) {
        depth_scale = 0.001; // 对于16UC1格式，通常单位为毫米，转换为米
    }
    
    try {
        // 创建检测结果的可修改副本
        std::vector<DetectedObject> updated_objects = objects;
        
        for (auto& obj : updated_objects) {
            // 计算物体在图像中的矩形区域
            // 假设obj.pose中包含图像坐标系的位置信息
            double center_x = obj.pose.position.x;
            double center_y = obj.pose.position.y;
            double width = 0.0, height = 0.0;
            
            // 尝试从obj.dimensions获取尺寸
            if (obj.dimensions.x() > 0 && obj.dimensions.y() > 0) {
                width = obj.dimensions.x();
                height = obj.dimensions.y();
            } else {
                // 如果没有尺寸信息，使用默认值
                width = 50;
                height = 50;
            }
            
            // 确保区域在图像范围内
            int left = std::max(0, static_cast<int>(center_x - width/2));
            int top = std::max(0, static_cast<int>(center_y - height/2));
            int right = std::min(depth_map.cols - 1, static_cast<int>(center_x + width/2));
            int bottom = std::min(depth_map.rows - 1, static_cast<int>(center_y + height/2));
            
            // 确保区域有效
            if (right <= left || bottom <= top) {
                ROS_WARN("物体 %s 的区域无效: [%d,%d,%d,%d]", 
                         obj.id.c_str(), left, top, right, bottom);
                continue;
            }
            
            // 提取深度区域
            cv::Rect roi(left, top, right - left, bottom - top);
            cv::Mat depth_roi;
            
            // 检查ROI有效性
            if (roi.x >= 0 && roi.y >= 0 && 
                roi.x + roi.width <= depth_map.cols &&
                roi.y + roi.height <= depth_map.rows) {
                depth_roi = depth_map(roi);
            } else {
                ROS_WARN("物体 %s 的ROI超出图像范围", obj.id.c_str());
                continue;
            }
            
            // 创建有效深度掩码
            cv::Mat mask;
            double min_val = 0.1; // 最小有效深度（米）
            double max_val = 10.0; // 最大有效深度（米）
            
            if (is_float_depth) {
                mask = (depth_roi > min_val) & (depth_roi < max_val);
            } else {
                mask = (depth_roi > min_val/depth_scale) & (depth_roi < max_val/depth_scale);
            }
            
            // 检查是否有有效深度值
            if (cv::countNonZero(mask) == 0) {
                ROS_WARN("物体 %s 的深度区域中没有有效值", obj.id.c_str());
                continue;
            }
            
            // 计算平均深度
            double avg_depth = 0.0;
            int valid_count = 0;
            
            for (int y = 0; y < depth_roi.rows; y++) {
                for (int x = 0; x < depth_roi.cols; x++) {
                    if (mask.at<uchar>(y, x)) {
                        double depth_value = 0.0;
                        if (is_float_depth) {
                            depth_value = depth_roi.at<float>(y, x);
                        } else {
                            depth_value = depth_roi.at<uint16_t>(y, x) * depth_scale;
                        }
                        
                        avg_depth += depth_value;
                        valid_count++;
                    }
                }
            }
            
            // 计算最终平均值
            if (valid_count > 0) {
                avg_depth /= valid_count;
                
                // 更新物体的3D位置
                obj.position = QVector3D(
                    static_cast<float>(center_x), 
                    static_cast<float>(center_y), 
                    static_cast<float>(avg_depth)
                );
                
                // 将距离转换为厘米并保存
                obj.z = avg_depth * 100.0;
                
                // 添加详细位置信息到对象标签
                obj.label = QString("%1 (%.1f cm)").arg(
                    QString::fromStdString(obj.id)).arg(obj.z).toStdString();
                
                ROS_DEBUG("物体 %s 的深度估计: %.2f 米", obj.id.c_str(), avg_depth);
            }
        }
        
        // 更新物体列表
        detected_objects_ = updated_objects;
        
    } catch (const cv::Exception& e) {
        ROS_ERROR("深度估计出错: %s", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("深度估计出错: %s", e.what());
    } catch (...) {
        ROS_ERROR("深度估计出现未知错误");
    }
}

void MainWindow::drawDetectionBoxes(QImage& image, const std::vector<DetectedObject>& objects)
{
    if (image.isNull() || objects.empty()) {
        return;
    }
    
    try {
        // 创建可绘制的副本
        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing);
        
        // 设置字体
        QFont font = painter.font();
        font.setPointSize(10);
        font.setBold(true);
        painter.setFont(font);
        
        // 设置文本度量对象，用于测量文本尺寸
        QFontMetrics fm(font);
        
        // 为不同类别设置颜色
        QMap<QString, QColor> classColors;
        classColors["person"] = QColor(255, 0, 0);    // 红色
        classColors["cup"] = QColor(0, 255, 0);       // 绿色
        classColors["bottle"] = QColor(0, 0, 255);    // 蓝色
        classColors["bowl"] = QColor(255, 255, 0);    // 黄色
        classColors["book"] = QColor(255, 0, 255);    // 洋红色
        classColors["cell phone"] = QColor(0, 255, 255); // 青色
        
        // 遍历检测对象绘制边界框
        for (const auto& obj : objects) {
            // 提取物体位置和尺寸
            double center_x = obj.pose.position.x;
            double center_y = obj.pose.position.y;
            double width = 0.0, height = 0.0;
            
            // 尝试从obj.dimensions获取尺寸
            if (obj.dimensions.x() > 0 && obj.dimensions.y() > 0) {
                width = obj.dimensions.x();
                height = obj.dimensions.y();
            } else {
                // 如果没有尺寸信息，使用默认值
                width = 50;
                height = 50;
            }
            
            // 计算矩形区域
            int left = static_cast<int>(center_x - width/2);
            int top = static_cast<int>(center_y - height/2);
            int right = static_cast<int>(center_x + width/2);
            int bottom = static_cast<int>(center_y + height/2);
            
            // 确保区域在图像范围内
            left = std::max(0, std::min(image.width()-1, left));
            top = std::max(0, std::min(image.height()-1, top));
            right = std::max(0, std::min(image.width()-1, right));
            bottom = std::max(0, std::min(image.height()-1, bottom));
            
            // 边界框区域
            QRect rect(left, top, right-left, bottom-top);
            
            // 选择绘制颜色
            QString className = QString::fromStdString(obj.type);
            QColor color = classColors.contains(className) ? 
                         classColors[className] : QColor(255, 165, 0); // 默认橙色
            
            // 计算置信度文本
            QString confidenceText = QString::number(obj.confidence * 100.0, 'f', 1) + "%";
            
            // 计算类别和置信度标签
            QString labelText = QString::fromStdString(obj.type) + " " + confidenceText;
            
            // 如果有距离信息，添加到标签
            if (obj.z > 0) {
                labelText += QString(" (%1 cm)").arg(obj.z, 0, 'f', 1);
            }
            
            // 计算文本尺寸
            QRect textRect = fm.boundingRect(labelText);
            
            // 绘制边界框
            QPen pen(color, 3);
            painter.setPen(pen);
            painter.drawRect(rect);
            
            // 绘制标签背景
            QRect backgroundRect(left, top - textRect.height() - 4, 
                               textRect.width() + 10, textRect.height() + 4);
            painter.fillRect(backgroundRect, QColor(0, 0, 0, 180));
            
            // 绘制标签文本
            painter.setPen(QPen(Qt::white));
            painter.drawText(left + 5, top - 4, labelText);
        }
    } catch (const std::exception& e) {
        ROS_ERROR("绘制检测框出错: %s", e.what());
    } catch (...) {
        ROS_ERROR("绘制检测框时出现未知错误");
    }
}

void MainWindow::showObjectDistanceOverlay(QImage& image, const std::vector<DetectedObject>& objects)
{
    if (image.isNull() || objects.empty()) {
        return;
    }
    
    try {
        // 创建可绘制的副本
        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing);
        
        // 设置字体
        QFont font = painter.font();
        font.setPointSize(12);
        font.setBold(true);
        painter.setFont(font);
        
        // 设置文本度量对象，用于测量文本尺寸
        QFontMetrics fm(font);
        
        // 在图像顶部创建信息面板
        int panelHeight = 30;
        int padding = 10;
        
        // 绘制信息面板背景
        painter.fillRect(QRect(0, 0, image.width(), panelHeight), 
                       QColor(0, 0, 0, 180));
        
        // 计算要显示的物体数量（最多显示前3个）
        int numObjects = std::min(3, static_cast<int>(objects.size()));
        
        // 如果有物体且有距离信息
        if (numObjects > 0) {
            // 排序物体按距离从近到远
            auto sortedObjects = objects;
            std::sort(sortedObjects.begin(), sortedObjects.end(), 
                    [](const DetectedObject& a, const DetectedObject& b) {
                        return a.z < b.z;
                    });
            
            // 显示物体距离信息
            QString infoText = "检测到 " + QString::number(objects.size()) + " 个物体 | ";
            
            for (int i = 0; i < numObjects; ++i) {
                const auto& obj = sortedObjects[i];
                
                // 检查是否有有效距离
                if (obj.z <= 0) {
                    continue;
                }
                
                // 添加物体信息
                if (i > 0) {
                    infoText += " | ";
                }
                
                infoText += QString::fromStdString(obj.type) + 
                          ": " + QString::number(obj.z, 'f', 1) + " cm";
            }
            
            // 绘制信息文本
            painter.setPen(QPen(Qt::white));
            painter.drawText(padding, panelHeight - padding, infoText);
        } else {
            // 没有物体或无距离信息
            QString infoText = "未检测到物体";
            painter.setPen(QPen(Qt::white));
            painter.drawText(padding, panelHeight - padding, infoText);
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("绘制距离信息叠加层出错: %s", e.what());
    } catch (...) {
        ROS_ERROR("绘制距离信息叠加层时出现未知错误");
    }
}

void MainWindow::createPlaceholderImage(const QString& message)
{
    // 创建占位图像
    int width = 640;
    int height = 480;
    QImage placeholder(width, height, QImage::Format_RGB888);
    placeholder.fill(Qt::black);
    
    QPainter painter(&placeholder);
    
    // 绘制提示文本
    if (!message.isEmpty()) {
    painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 14, QFont::Bold));
        QRect textRect = painter.fontMetrics().boundingRect(0, 0, width, height, Qt::AlignCenter, message);
        painter.drawText((width - textRect.width()) / 2, (height + textRect.height()) / 2, message);
    }
    
    // 存储占位图像
    current_camera_image_ = placeholder;
    left_camera_image_ = placeholder;
    right_camera_image_ = placeholder;
    current_depth_image_ = placeholder;
    detection_image_ = placeholder;
    
    // 更新界面
    updateCameraView();
}

QImage MainWindow::cvMatToQImage(const cv::Mat& mat)
{
    // 增加额外的检查，避免处理空图像或非法图像
    if(mat.empty() || mat.cols <= 0 || mat.rows <= 0) {
        ROS_WARN("尝试转换空的或无效的图像");
        return QImage();
    }

    try {
        // 将OpenCV的Mat转换为Qt的QImage
        if(mat.type() == CV_8UC1) {
            // 灰度图像
            QImage image(mat.cols, mat.rows, QImage::Format_Grayscale8);
            for(int i = 0; i < mat.rows; i++) {
                if (mat.ptr(i) != nullptr) {
                    memcpy(image.scanLine(i), mat.ptr(i), static_cast<size_t>(mat.cols));
                } else {
                    ROS_ERROR("空图像数据指针");
                    return QImage();
                }
            }
            return image;
        } else if(mat.type() == CV_8UC3) {
            // BGR颜色图像转换为RGB - 使用更安全的深拷贝方式
            cv::Mat rgbMat;
            try {
                cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
                if (rgbMat.empty() || rgbMat.data == nullptr) {
                    ROS_ERROR("颜色转换后图像为空");
                    return QImage();
                }
                
                // 创建一个完整的拷贝，避免共享内存
                return QImage(rgbMat.data, rgbMat.cols, rgbMat.rows, 
                            static_cast<int>(rgbMat.step), QImage::Format_RGB888).copy();
            } catch (const cv::Exception& e) {
                ROS_ERROR("OpenCV转换错误: %s", e.what());
                return QImage();
            }
        } else if(mat.type() == CV_8UC4) {
            // BGRA颜色图像
            if (mat.data == nullptr) {
                ROS_ERROR("BGRA图像数据为空");
                return QImage();
            }
            
            // 创建一个完整的拷贝，避免共享内存
            return QImage(mat.data, mat.cols, mat.rows, 
                        static_cast<int>(mat.step), QImage::Format_ARGB32).copy();
        }
        
        // 不支持的格式，返回空图像
        ROS_WARN("不支持的图像格式: %d", mat.type());
        return QImage();
    } catch (const std::exception& e) {
        ROS_ERROR("图像转换过程中出现异常: %s", e.what());
        return QImage();
    } catch (...) {
        ROS_ERROR("图像转换过程中出现未知异常");
        return QImage();
    }
}

cv::Mat MainWindow::qImageToCvMat(const QImage& image)
{
    if (image.isNull()) {
        ROS_ERROR("尝试转换空的QImage");
        return cv::Mat();
    }
    
    try {
        switch (image.format()) {
            case QImage::Format_RGB888: {
                // RGB图像转换为BGR
                cv::Mat mat(image.height(), image.width(), CV_8UC3, 
                         const_cast<uchar*>(image.bits()), image.bytesPerLine());
                cv::Mat result;
                cv::cvtColor(mat, result, cv::COLOR_RGB2BGR);
                return result.clone();  // 返回深拷贝
            }
            
            case QImage::Format_ARGB32:
            case QImage::Format_ARGB32_Premultiplied: {
                // ARGB图像转换为BGRA
                cv::Mat mat(image.height(), image.width(), CV_8UC4, 
                         const_cast<uchar*>(image.bits()), image.bytesPerLine());
                cv::Mat result;
                cv::cvtColor(mat, result, cv::COLOR_RGBA2BGRA);
                return result.clone();  // 返回深拷贝
            }
            
            case QImage::Format_Grayscale8: {
                // 灰度图像
                cv::Mat mat(image.height(), image.width(), CV_8UC1, 
                         const_cast<uchar*>(image.bits()), image.bytesPerLine());
                return mat.clone();  // 返回深拷贝
            }
            
            default: {
                // 其他格式，先转换为RGB888
                QImage converted = image.convertToFormat(QImage::Format_RGB888);
                cv::Mat mat(converted.height(), converted.width(), CV_8UC3, 
                         const_cast<uchar*>(converted.bits()), converted.bytesPerLine());
                cv::Mat result;
                cv::cvtColor(mat, result, cv::COLOR_RGB2BGR);
                return result.clone();  // 返回深拷贝
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("QImage转换为cv::Mat出错: %s", e.what());
        return cv::Mat();
    } catch (...) {
        ROS_ERROR("QImage转换为cv::Mat出现未知错误");
        return cv::Mat();
    }
} 

void MainWindow::stereoMergedCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // 检查消息是否有效
    if (!msg) {
        ROS_WARN("接收到空的图像消息指针");
        return;
    }

    try {
        // 检查消息大小
        if (msg->width == 0 || msg->height == 0 || msg->data.empty()) {
            ROS_WARN("接收到无效尺寸的图像消息 (%dx%d)", msg->width, msg->height);
            return;
        }

        // 检查编码格式
        std::string encoding = msg->encoding;
        if (encoding.empty()) {
            encoding = sensor_msgs::image_encodings::BGR8;  // 默认假设BGR8格式
            ROS_WARN("图像消息没有指定编码格式，假设为BGR8");
        }

        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("CV Bridge转换错误: %s", e.what());
            stereo_camera_error_count_++;
            
            // 如果错误次数过多，标记相机不可用
            if (stereo_camera_error_count_ > 10) {
                is_camera_available_ = false;
                createPlaceholderImage("图像格式转换错误"); 
            }
            return;
        }

        // 安全检查：确保cv_ptr和cv_ptr->image有效
        if (!cv_ptr || cv_ptr->image.empty()) {
            ROS_WARN("CV桥接转换后图像为空");
            stereo_camera_error_count_++;
            if (stereo_camera_error_count_ > 10) {
                is_camera_available_ = false;
                createPlaceholderImage("图像转换后为空");
            }
            return;
        }

        cv::Mat cv_image = cv_ptr->image;
        
        if (!cv_image.empty() && cv_image.cols > 0 && cv_image.rows > 0) {
            // 保存当前图像
            current_image_ = cv_image.clone();
            
            // 获取图像来源标识
            std::string frame_id = msg->header.frame_id;
            
            // 根据图像帧ID确定图像类型
            bool is_detection_image = (frame_id == "detection");
            bool is_left_image = (frame_id == "stereo_left" || frame_id.empty());
            bool is_right_image = (frame_id == "stereo_right");
            
            // 处理普通相机图像
            if (!is_detection_image) {
                QImage qimg = cvMatToQImage(cv_image);
                if (!qimg.isNull()) {
                    // 根据帧ID处理不同视图
                    if (is_left_image) {
                        left_camera_image_ = qimg;
                        if (camera_view_mode_ == 0) {
                            // 如果启用了YOLO检测并要显示检测框
                            if (yolo_enabled_ && show_detection_boxes_ && !detected_objects_.empty()) {
                                // 创建可绘制的副本
                                QImage processed_image = qimg.copy();
                                // 绘制检测框
                                drawDetectionBoxes(processed_image, detected_objects_);
                                // 如果需要显示距离信息
                                if (show_distance_overlay_ && depth_available_) {
                                    showObjectDistanceOverlay(processed_image, detected_objects_);
                                }
                                current_camera_image_ = processed_image;
                            } else {
                                current_camera_image_ = qimg;
                            }
                        }
                    } else if (is_right_image) {
                        right_camera_image_ = qimg;
                        if (camera_view_mode_ == 1) {
                            current_camera_image_ = qimg;
                        }
                    } else {
                        // 通用处理，直接显示收到的图像
                        current_camera_image_ = qimg;
                    }
                } else {
                    ROS_WARN("相机图像转换为QImage失败");
                }
            } 
            // 处理YOLO检测图像
            else {
                QImage qimg = cvMatToQImage(cv_image);
                if (!qimg.isNull()) {
                    detection_image_ = qimg;
                    // 如果当前是在"检测视图"模式下，直接显示检测结果图像
                    if (camera_view_mode_ == 3) {  // 假设3是检测视图模式
                        current_camera_image_ = detection_image_;
                    }
                    // 否则，更新当前视图模式下的图像
                    else if (camera_view_mode_ == 0 && !left_camera_image_.isNull()) {
                        // 创建可绘制的副本
                        QImage processed_image = left_camera_image_.copy();
                        // 绘制检测框
                        drawDetectionBoxes(processed_image, detected_objects_);
                        // 如果需要显示距离信息
                        if (show_distance_overlay_ && depth_available_) {
                            showObjectDistanceOverlay(processed_image, detected_objects_);
                        }
                        current_camera_image_ = processed_image;
                    }
                } else {
                    ROS_WARN("检测图像转换为QImage失败");
                }
            }
            
            // 标记相机为可用
            is_camera_available_ = true;
            stereo_camera_error_count_ = 0;
            
            // 如果有深度图可用，并且有检测对象，则估计物体距离
            if (depth_available_ && !detected_objects_.empty() && !current_depth_map_.empty()) {
                estimateObjectDistances(detected_objects_, current_depth_map_);
            }
            
        } else {
            ROS_WARN("接收到空图像");
            stereo_camera_error_count_++;
            
            // 如果错误次数过多，标记相机不可用
            if (stereo_camera_error_count_ > 10) {
                is_camera_available_ = false;
                createPlaceholderImage("接收到空图像"); 
            }
        }
    } catch (std::exception& e) {
        ROS_ERROR("处理图像时发生未知错误: %s", e.what());
        stereo_camera_error_count_++;
        
        // 如果错误次数过多，标记相机不可用
        if (stereo_camera_error_count_ > 10) {
            is_camera_available_ = false;
            createPlaceholderImage("图像处理错误"); 
        }
    } catch (...) {
        ROS_ERROR("处理图像时发生未知异常");
        stereo_camera_error_count_++;
        
        // 如果错误次数过多，标记相机不可用
        if (stereo_camera_error_count_ > 10) {
            is_camera_available_ = false;
            createPlaceholderImage("未知图像错误");
        }
    }
}

void MainWindow::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // 检查消息是否有效
    if (!msg) {
        ROS_WARN("接收到空的深度图像消息指针");
        return;
    }

    try {
        // 检查消息大小
        if (msg->width == 0 || msg->height == 0 || msg->data.empty()) {
            ROS_WARN("接收到无效尺寸的深度图像消息 (%dx%d)", msg->width, msg->height);
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        
        // 根据深度图编码格式进行不同处理
        if (msg->encoding == "32FC1") {
            try {
                // 原始深度图
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
                
                // 安全检查：确保cv_ptr和cv_ptr->image有效
                if (!cv_ptr || cv_ptr->image.empty()) {
                    ROS_WARN("32FC1深度图转换后为空");
                    return;
                }
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("CV Bridge 32FC1转换错误: %s", e.what());
                return;
            }
            
            cv::Mat depth_float = cv_ptr->image;
            if (depth_float.empty() || depth_float.cols <= 0 || depth_float.rows <= 0) {
                ROS_WARN("32FC1深度图为空或尺寸无效");
                return;
            }
            
            // 保存原始深度图
            current_depth_map_ = depth_float.clone();
            depth_available_ = true;
            
            // 如果有检测对象，则估计物体距离
            if (!detected_objects_.empty()) {
                estimateObjectDistances(detected_objects_, current_depth_map_);
            }
        }
        else if (msg->encoding == "16UC1") {
            try {
                // 16位无符号整数深度图
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                
                // 安全检查：确保cv_ptr和cv_ptr->image有效
                if (!cv_ptr || cv_ptr->image.empty()) {
                    ROS_WARN("16UC1深度图转换后为空");
                    return;
                }
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("CV Bridge 16UC1转换错误: %s", e.what());
                return;
            }
            
            cv::Mat depth_16u = cv_ptr->image;
            if (depth_16u.empty() || depth_16u.cols <= 0 || depth_16u.rows <= 0) {
                ROS_WARN("16UC1深度图为空或尺寸无效");
                return;
            }
            
            // 保存原始深度图
            // 转换为32FC1格式，这样就可以使用统一的处理方式
            cv::Mat depth_float;
            depth_16u.convertTo(depth_float, CV_32FC1, 0.001); // 通常16位深度单位为毫米，转换为米
            current_depth_map_ = depth_float.clone();
            depth_available_ = true;
            
            // 如果有检测对象，则估计物体距离
            if (!detected_objects_.empty()) {
                estimateObjectDistances(detected_objects_, current_depth_map_);
            }
        }
        else {
            ROS_WARN("不支持的深度图编码格式: %s", msg->encoding.c_str());
            return;
        }
        
        // 标记为相机可用
        is_camera_available_ = true;
        stereo_camera_error_count_ = 0;
        
    } catch (std::exception& e) {
        ROS_ERROR("处理深度图像时发生错误: %s", e.what());
    } catch (...) {
        ROS_ERROR("处理深度图像时发生未知异常");
    }
}

void MainWindow::objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg, 
                                      const geometry_msgs::PoseArray::ConstPtr& poses_msg)
{
    // 检查消息是否有效
    if (!img_msg || !poses_msg) {
        ROS_WARN("接收到空的检测消息指针");
        return;
    }
    
    try {
        // 处理检测图像
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("CV Bridge转换错误: %s", e.what());
            return;
        }
        
        if (!cv_ptr || cv_ptr->image.empty()) {
            ROS_WARN("检测图像为空或转换失败");
            return;
        }
        
        // 保存检测图像
        cv::Mat detection_image = cv_ptr->image.clone();
        detection_image_ = cvMatToQImage(detection_image);
        
        // 处理位姿数据
        std::vector<DetectedObject> detected_objects;
        std::vector<geometry_msgs::Pose> poses = poses_msg->poses;
        
        // 设置检测时间
        last_detection_time_ = ros::Time::now();
        
        // 解析位姿数组中的信息
        for (size_t i = 0; i < poses.size(); ++i) {
            const auto& pose = poses[i];
            
            DetectedObject obj;
            obj.id = "object_" + std::to_string(i);
            
            // 从pose中提取类别信息和置信度（通常编码在orientation字段）
            // 假设类别索引存储在orientation.x，置信度存储在orientation.y
            int class_id = static_cast<int>(pose.orientation.x);
            obj.confidence = pose.orientation.y;
            
            // 将类别ID转换为类别名称
            switch (class_id) {
                case 0: obj.type = "person"; break;
                case 1: obj.type = "bottle"; break;
                case 2: obj.type = "cup"; break;
                case 3: obj.type = "bowl"; break;
                case 4: obj.type = "book"; break;
                case 5: obj.type = "cell phone"; break;
                default: obj.type = "unknown"; break;
            }
            
            // 提取2D位置和尺寸（存储在position字段）
            // 假设中心x,y在position.x和position.y，宽高在position.z和orientation.z
            obj.pose = pose;
            
            // 提取尺寸信息（如果有）
            double width = pose.position.z;  // 宽度
            double height = pose.orientation.z;  // 高度
            
            // 设置对象尺寸
            obj.dimensions = QVector3D(width, height, 0.0f);
            
            // 初始化位置（稍后在深度估计中会更新z坐标）
            obj.position = QVector3D(pose.position.x, pose.position.y, 0.0f);
            
            // 基于类别添加初始标签
            obj.label = obj.type + " (" + std::to_string(static_cast<int>(obj.confidence * 100)) + "%)";
            
            // 添加到对象列表
            detected_objects.push_back(obj);
        }
        
        // 更新检测结果
        detected_objects_ = detected_objects;
        
        // 如果有深度图可用，则估计物体距离
        if (depth_available_ && !current_depth_map_.empty()) {
            estimateObjectDistances(detected_objects_, current_depth_map_);
        }
        
        // 触发UI更新
        QMetaObject::invokeMethod(this, "updateCameraView", Qt::QueuedConnection);
        
    } catch (std::exception& e) {
        ROS_ERROR("处理检测结果时发生错误: %s", e.what());
    } catch (...) {
        ROS_ERROR("处理检测结果时发生未知异常");
    }
}

void MainWindow::updateCameraView()
{
    if (!cameraView) {
        return;
    }
    
    // 根据相机模式选择要显示的图像
    QImage displayImage;
    
    if (!is_camera_available_) {
        // 如果相机不可用，显示占位图像
        createPlaceholderImage("相机连接中...");
        displayImage = current_camera_image_;
    } else {
        // 根据相机视图模式显示不同的图像
        switch(camera_view_mode_) {
        case 0:  // 左相机
            displayImage = left_camera_image_;
            break;
        case 1:  // 右相机
            displayImage = right_camera_image_;
            break;
        default:  // 默认混合视图
            displayImage = current_camera_image_;
            break;
        }
    }
    
    // 在图像上绘制检测框
    if (yolo_enabled_ && show_detection_boxes_ && !detected_objects_.empty()) {
        drawDetectionBoxes(displayImage, detected_objects_);
    }
    
    // 显示图像
    cameraView->setPixmap(QPixmap::fromImage(displayImage.scaled(
        cameraView->width(), 
        cameraView->height(),
        Qt::KeepAspectRatio
    )));
}

void MainWindow::updateEndEffectorPose()
{
    if (!kinematics_utils_ || current_joint_values_.empty()) {
        return;
    }

    try {
        // 使用前向运动学计算末端位置
        current_end_pose_ = forwardKinematics(current_joint_values_);
        
        // 更新显示
        if (endEffectorStatusLabel) {
            QString positionText = QString("位置: X:%1 Y:%2 Z:%3 cm")
                .arg(current_end_pose_.position.x * 100, 0, 'f', 2)
                .arg(current_end_pose_.position.y * 100, 0, 'f', 2)
                .arg(current_end_pose_.position.z * 100, 0, 'f', 2);
            endEffectorStatusLabel->setText(positionText);
        }
    } catch (const std::exception& e) {
        // 错误处理
        if (endEffectorStatusLabel) {
            endEffectorStatusLabel->setText("末端位置计算错误");
        }
        ROS_ERROR("末端位置计算错误: %s", e.what());
    }
}

void MainWindow::onShowDetectionBoxesToggled(bool checked) {
    show_detection_boxes_ = checked;
    updateCameraView();
}

void MainWindow::onShowDistanceOverlayToggled(bool checked) {
    show_distance_overlay_ = checked;
    updateCameraView();
}

void MainWindow::onDetectionModelChanged(const QString& model) {
    current_detection_model_ = model.toStdString();
    // 发送模型变更消息
    ROS_INFO("Detection model changed to: %s", current_detection_model_.c_str());
}

void MainWindow::onDetectObjectsClicked() {
    // 触发物体检测
    ROS_INFO("Object detection triggered");
}

void MainWindow::onObjectTableItemClicked(int row, int column) {
    selected_object_index_ = row;
    ROS_INFO("Selected object at index: %d", selected_object_index_);
}

void MainWindow::onGraspSelectedObjectClicked() {
    if (selected_object_index_ >= 0 && selected_object_index_ < static_cast<int>(detected_objects_.size())) {
        ROS_INFO("Attempting to grasp selected object");
    } else {
        ROS_WARN("No valid object selected for grasping");
    }
}

void MainWindow::onStereoViewButtonClicked() {
    camera_view_mode_ = 2;
    std_msgs::Int32 mode_msg;
    mode_msg.data = camera_view_mode_;
    camera_view_mode_pub_.publish(mode_msg);
    ROS_INFO("Camera view mode set to: Stereo");
}

void MainWindow::onDepthViewButtonClicked() {
    camera_view_mode_ = 3;
    std_msgs::Int32 mode_msg;
    mode_msg.data = camera_view_mode_;
    camera_view_mode_pub_.publish(mode_msg);
    ROS_INFO("Camera view mode set to: Depth");
}
