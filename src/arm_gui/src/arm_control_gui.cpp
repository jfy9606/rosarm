#include "arm_gui/arm_control_gui.h"
#include "ui_arm_control_main.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QTimer>
#include <QCloseEvent>
#include <cmath>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <unistd.h> // 用于access函数

ArmControlGUI::ArmControlGUI(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent), ui(new Ui::ArmControlMainWindow), nh_(nh),
    vacuum_on_(false), vacuum_power_(50), selected_object_index_(-1),
    yolo_enabled_(false), camera_view_mode_(0), is_camera_available_(false),
    stereo_camera_error_count_(0), available_camera_index_(-1),
    ignore_slider_events_(false), ignore_spin_events_(false), ui_processing_(false),
    scene_3d_renderer_(nullptr), updateTimer(new QTimer(this)), camera_reconnect_timer_(this)
{
    // 初始化UI
    ui->setupUi(this);
    
    // 设置状态栏
    statusBar()->showMessage("机械臂控制界面初始化中...");
    
    // 创建占位图像 - 即使没有相机也先创建
    createPlaceholderImage("等待摄像头连接...");
    
    // 初始化ROS
    initializeROS();
    
    // 初始化成员变量
    initializeMembers();
    
    // 初始化控件连接
    connectSignalSlots();
    
    // 初始化UI配置
    setupUi();
    
    // 初始化GUI组件
    initializeGUI();
    
    // 设置相机参数
    setupCameraParameters();
    
    // 设置关节限制
    setupJointLimits();
    
    // 初始化运动学工具
    kinematics_utils_ = new arm_trajectory::KinematicsUtils(nh_);
    
    // 启动定时器
    updateTimer->start(33);  // 30FPS
    
    // 设置相机重连定时器
    connect(&camera_reconnect_timer_, &QTimer::timeout, this, &ArmControlGUI::attemptCameraReconnect);
    camera_reconnect_timer_.start(5000);  // 每5秒尝试重连相机
    
    // 设置标志
    ignore_slider_events_ = false;
    ignore_spin_events_ = false;
    ui_processing_ = false;
    is_camera_available_ = false;
    stereo_camera_error_count_ = 0;
    camera_view_mode_ = 0;  // 默认左视图
    
    // 将界面设置为初始状态
    updateGUIJointValues();
    
    // 更新UI
    updateUI();
    
    // 更新相机视图（使用占位图像）
    updateCameraView();
    
    statusBar()->showMessage("机械臂控制界面已启动");
}

ArmControlGUI::~ArmControlGUI()
{
    if (scene_3d_renderer_) {
        delete scene_3d_renderer_;
    }
    if (kinematics_utils_) {
        delete kinematics_utils_;
    }
    delete ui;
}

void ArmControlGUI::initializeROS()
{
    try {
        // 初始化ROS节点
        ros::NodeHandle nh;
        
        // 初始化发布者
        joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_command", 1);
        vacuum_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/vacuum_command", 1);
        vacuum_power_pub_ = nh_.advertise<std_msgs::Int32>("/vacuum_power", 1);
        camera_view_mode_pub_ = nh_.advertise<std_msgs::Int32>("/stereo_camera/view_mode", 1);
        
        // 初始化服务客户端 - 不再需要直接使用运动学服务，通过KinematicsUtils访问
        joint_control_client_ = nh_.serviceClient<servo_wrist::JointControl>("/joint_control");
        vacuum_control_client_ = nh_.serviceClient<servo_wrist::VacuumControl>("/vacuum_control");
        home_position_client_ = nh_.serviceClient<servo_wrist::HomePosition>("/home_position");
        
        // 设置ROS订阅
        setupROSSubscriptions();
        
        ROS_INFO("ROS初始化完成");
    }
    catch (const std::exception& e) {
        ROS_ERROR("ROS初始化失败: %s", e.what());
    }
}

void ArmControlGUI::initializeMembers()
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
    current_detection_model_ = "yolov8n";
    
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
    connect(&camera_reconnect_timer_, &QTimer::timeout, this, &ArmControlGUI::attemptCameraReconnect);
    camera_reconnect_timer_.start(5000); // 每5秒尝试重连一次
}

void ArmControlGUI::setupUi()
{
    // 设置关节控件范围
    ui->joint1Slider->setRange(-180, 180);
    ui->joint2Slider->setRange(0, 50);
    ui->joint3Slider->setRange(-90, 90);
    ui->joint4Slider->setRange(0, 180);
    ui->joint5Slider->setRange(-90, 90);
    ui->joint6Slider->setRange(5, 15);
    
    ui->joint1Spin->setRange(-180.0, 180.0);
    ui->joint2Spin->setRange(0.0, 50.0);
    ui->joint3Spin->setRange(-90.0, 90.0);
    ui->joint4Spin->setRange(0.0, 180.0);
    ui->joint5Spin->setRange(-90.0, 90.0);
    ui->joint6Spin->setRange(5.0, 15.0);
    
    // 检查并创建可能缺失的UI元素
    createMissingUIElements();
}

// 创建缺失的UI元素
void ArmControlGUI::createMissingUIElements()
{
    // 查找cameraView，如果不存在则创建
    QLabel* cameraView = findChild<QLabel*>("cameraView");
    if (!cameraView) {
        cameraView = new QLabel(this);
        cameraView->setObjectName("cameraView");
        cameraView->setText("相机视图");
        cameraView->setMinimumSize(640, 480);
        cameraView->setAlignment(Qt::AlignCenter);
        cameraView->setStyleSheet("background-color: #333333; color: white;");
        
        // 添加到布局
        QVBoxLayout* cameraLayout = new QVBoxLayout();
        cameraLayout->setObjectName("cameraLayout");
        cameraLayout->addWidget(cameraView);
        
        // 查找右侧面板
        QWidget* rightPanel = findChild<QWidget*>("rightPanel");
        if (rightPanel) {
            QBoxLayout* rightLayout = qobject_cast<QBoxLayout*>(rightPanel->layout());
            if (rightLayout) {
                rightLayout->insertLayout(0, cameraLayout);
            }
        } else {
            // 如果没有找到右侧面板，尝试添加到中央部件
            QVBoxLayout* centralLayout = new QVBoxLayout(ui->centralwidget);
            centralLayout->addLayout(cameraLayout);
        }
        
        // 使其可接收事件
        cameraView->installEventFilter(this);
    }
    
    // 创建视图切换按钮面板
    QWidget* cameraControls = findChild<QWidget*>("cameraControls");
    if (!cameraControls) {
        cameraControls = new QWidget(this);
        cameraControls->setObjectName("cameraControls");
        
        QHBoxLayout* controlLayout = new QHBoxLayout(cameraControls);
        controlLayout->setObjectName("cameraControlsLayout");
        controlLayout->setContentsMargins(0, 0, 0, 0);
        
        // 创建左视图按钮
        QPushButton* leftViewButton = findChild<QPushButton*>("leftViewButton");
        if (!leftViewButton) {
            leftViewButton = new QPushButton("左视图", cameraControls);
            leftViewButton->setObjectName("leftViewButton");
            leftViewButton->setToolTip("显示左相机视图");
            connect(leftViewButton, &QPushButton::clicked, this, &ArmControlGUI::onLeftViewButtonClicked);
            controlLayout->addWidget(leftViewButton);
        }
        
        // 创建右视图按钮
        QPushButton* rightViewButton = findChild<QPushButton*>("rightViewButton");
        if (!rightViewButton) {
            rightViewButton = new QPushButton("右视图", cameraControls);
            rightViewButton->setObjectName("rightViewButton");
            rightViewButton->setToolTip("显示右相机视图");
            connect(rightViewButton, &QPushButton::clicked, this, &ArmControlGUI::onRightViewButtonClicked);
            controlLayout->addWidget(rightViewButton);
        }
        
        // 创建深度视图按钮
        QPushButton* depthViewButton = findChild<QPushButton*>("depthViewButton");
        if (!depthViewButton) {
            depthViewButton = new QPushButton("深度视图", cameraControls);
            depthViewButton->setObjectName("depthViewButton");
            depthViewButton->setToolTip("显示深度视图");
            depthViewButton->setEnabled(false); // 初始禁用
            connect(depthViewButton, &QPushButton::clicked, this, &ArmControlGUI::onDepthViewButtonClicked);
            controlLayout->addWidget(depthViewButton);
        }
        
        // 查找cameraLayout并添加控制面板
        QLayout* cameraLayout = findChild<QLayout*>("cameraLayout");
        if (cameraLayout) {
            cameraLayout->addWidget(cameraControls);
        } else {
            // 如果没有找到cameraLayout，添加到现有的布局
            QLabel* cameraView = findChild<QLabel*>("cameraView");
            if (cameraView) {
                QLayout* parentLayout = cameraView->parentWidget()->layout();
                if (parentLayout) {
                    parentLayout->addWidget(cameraControls);
                }
            }
        }
    }
}

void ArmControlGUI::connectSignalSlots()
{
    // 连接定时器
    connect(updateTimer, &QTimer::timeout, this, &ArmControlGUI::updateUI);
    
    // 连接关节滑块和微调框
    connect(ui->joint1Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint1SliderChanged);
    connect(ui->joint2Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint2SliderChanged);
    connect(ui->joint3Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint3SliderChanged);
    connect(ui->joint4Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint4SliderChanged);
    connect(ui->joint5Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint5SliderChanged);
    connect(ui->joint6Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint6SliderChanged);
    
    connect(ui->joint1Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint1SpinChanged);
    connect(ui->joint2Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint2SpinChanged);
    connect(ui->joint3Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint3SpinChanged);
    connect(ui->joint4Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint4SpinChanged);
    connect(ui->joint5Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint5SpinChanged);
    connect(ui->joint6Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint6SpinChanged);
    
    // 连接真空吸盘控制
    connect(ui->vacuumOnButton, &QPushButton::clicked, this, [this]() { sendVacuumCommand(true, ui->vacuumPowerSlider->value()); });
    connect(ui->vacuumOffButton, &QPushButton::clicked, this, [this]() { sendVacuumCommand(false, 0); });
    connect(ui->vacuumPowerSlider, &QSlider::valueChanged, this, &ArmControlGUI::onVacuumPowerSliderChanged);
    
    // 连接Home按钮
    connect(ui->homeButton, &QPushButton::clicked, this, &ArmControlGUI::goToHomePosition);
}

void ArmControlGUI::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
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

void ArmControlGUI::onUpdateGUI()
{
    // 避免在正在处理UI事件时执行更新
    if (ui_processing_) {
        return;
    }
    
    // 更新关节信息显示
    updateJointInfo();
    
    // 更新末端执行器姿态显示
    updateEndEffectorPose();
    
    // 更新真空吸盘状态显示
    updateVacuumStatus();
    
    // 更新相机视图
    updateCameraView();
    
    // 更新检测表格
    updateDetectionsTable();
    
    // 更新连接状态
    updateConnectionStatus();
    
    // 更新3D场景
    updateScene3D();
}

void ArmControlGUI::updateUI()
{
    // 更新末端执行器位置显示
    updateEndEffectorPose();
    
    // 更新吸附状态
    updateVacuumStatus();
}

void ArmControlGUI::updateEndEffectorPose()
{
    if (current_joint_values_.empty() || current_joint_values_.size() < 6) {
        return;
    }
    
    try {
        // 使用正向运动学计算当前末端位姿
        geometry_msgs::Pose current_pose = forwardKinematics(current_joint_values_);
        
        // 更新当前末端位置
        current_end_position_ = QVector3D(
            current_pose.position.x * 100.0f,  // 转换为厘米
            current_pose.position.y * 100.0f, 
            current_pose.position.z * 100.0f
        );
        
        // 更新当前末端方向
        current_end_orientation_ = QQuaternion(
            current_pose.orientation.w,
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z
        );
        
        // 更新当前末端位姿
        current_end_pose_ = current_pose;
        
        // 更新UI显示
        if (ui->endEffectorStatusLabel) {
            // 使用Qt的数字占位符格式，而不是printf风格的占位符
            QString posText = QString("末端位置: X=%1, Y=%2, Z=%3 cm")
                .arg(current_end_position_.x(), 0, 'f', 2)
                .arg(current_end_position_.y(), 0, 'f', 2)
                .arg(current_end_position_.z(), 0, 'f', 2);
            ui->endEffectorStatusLabel->setText(posText);
        }
        
        // 更新坐标输入框的值，但不触发事件
        ui->posXSpin->blockSignals(true);
        ui->posYSpin->blockSignals(true);
        ui->posZSpin->blockSignals(true);
        
        ui->posXSpin->setValue(current_end_position_.x());
        ui->posYSpin->setValue(current_end_position_.y());
        ui->posZSpin->setValue(current_end_position_.z());
        
        ui->posXSpin->blockSignals(false);
        ui->posYSpin->blockSignals(false);
        ui->posZSpin->blockSignals(false);
        
    } catch (const std::exception& e) {
        ROS_ERROR("更新末端位姿失败: %s", e.what());
    }
}

void ArmControlGUI::updateVacuumStatus()
{
    // 更新吸附按钮状态
    ui->vacuumOnButton->setEnabled(!vacuum_on_);
    ui->vacuumOffButton->setEnabled(vacuum_on_);
}

void ArmControlGUI::updateGUIJointValues()
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
    ui->joint1Slider->setValue(static_cast<int>(joint1_deg));
    ui->joint2Slider->setValue(static_cast<int>(current_joint_values_[1]));
    ui->joint3Slider->setValue(static_cast<int>(joint3_deg));
    ui->joint4Slider->setValue(static_cast<int>(joint4_deg));
    ui->joint5Slider->setValue(static_cast<int>(joint5_deg));
    ui->joint6Slider->setValue(static_cast<int>(current_joint_values_[5]));
    
    // 更新微调框值
    ui->joint1Spin->setValue(joint1_deg);
    ui->joint2Spin->setValue(current_joint_values_[1]);
    ui->joint3Spin->setValue(joint3_deg);
    ui->joint4Spin->setValue(joint4_deg);
    ui->joint5Spin->setValue(joint5_deg);
    ui->joint6Spin->setValue(current_joint_values_[5]);
    
    // 更新关节状态标签
    QString statusText = QString("关节状态: J1=%1° J2=%2 J3=%3° J4=%4° J5=%5° J6=%6")
                        .arg(joint1_deg, 0, 'f', 1)
                        .arg(current_joint_values_[1], 0, 'f', 1)
                        .arg(joint3_deg, 0, 'f', 1)
                        .arg(joint4_deg, 0, 'f', 1)
                        .arg(joint5_deg, 0, 'f', 1)
                        .arg(current_joint_values_[5], 0, 'f', 1);
    ui->jointStatusLabel->setText(statusText);
    
    ignore_slider_events_ = false;
    ignore_spin_events_ = false;
}

// Helper math functions
double ArmControlGUI::degToRad(double deg)
{
    if (!kinematics_utils_) {
        // Fallback if kinematics_utils_ is not initialized
        return deg * M_PI / 180.0;
    }
    return kinematics_utils_->degToRad(deg);
}

double ArmControlGUI::radToDeg(double rad)
{
    if (!kinematics_utils_) {
        // Fallback if kinematics_utils_ is not initialized
        return rad * 180.0 / M_PI;
    }
    return kinematics_utils_->radToDeg(rad);
}

// 使用KinematicsUtils实现正向运动学计算
geometry_msgs::Pose ArmControlGUI::forwardKinematics(const std::vector<double>& joint_values)
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

// 使用KinematicsUtils实现逆运动学计算
std::vector<double> ArmControlGUI::inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                                   const std::vector<double>& initial_guess)
{
    if (!kinematics_utils_) {
        // Fallback if kinematics_utils_ is not initialized
        ROS_ERROR("Kinematics utilities not initialized, cannot perform inverse kinematics");
        return std::vector<double>();
    }
    return kinematics_utils_->inverseKinematics(target_pose, initial_guess);
}

// 发送关节命令
void ArmControlGUI::sendJointCommand(const std::vector<double>& joint_values)
{
    // 创建关节状态消息
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = ros::Time::now();
    joint_msg.name = {"arm1_joint1", "arm1_joint2", "arm1_joint3", "arm1_joint4", "arm1_joint5", "arm1_joint6"};
    joint_msg.position = joint_values;
    
    // 发布关节命令
    joint_command_pub_.publish(joint_msg);
}

// 重载的无参数版本，使用当前关节值
void ArmControlGUI::sendJointCommand()
{
    sendJointCommand(current_joint_values_);
}

void ArmControlGUI::sendVacuumCommand(bool on, int power)
{
    std_msgs::Bool on_msg;
    on_msg.data = on;
    vacuum_cmd_pub_.publish(on_msg);
    
    std_msgs::Int32 power_msg;
    power_msg.data = power;
    vacuum_power_pub_.publish(power_msg);
    
    vacuum_on_ = on;
    vacuum_power_ = power;
}

void ArmControlGUI::sendHomeCommand()
{
    // 设定一组初始关节角度作为回原点位置
    std::vector<double> home_position = {0.0, 10.0, 0.0, 1.57, 1.57, 10.0};  // 根据实际机械臂调整
    sendJointCommand(home_position);
}

// 槽函数
void ArmControlGUI::onJoint1SliderChanged(int value)
{
    if (!ignore_slider_events_) {
        ignore_spin_events_ = true;
        ui->joint1Spin->setValue(value);
        ignore_spin_events_ = false;
        
        double radians = degToRad(value);
        std::vector<double> new_joints = current_joint_values_;
        new_joints[0] = radians;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onJoint2SliderChanged(int value)
{
    if (!ignore_slider_events_) {
        ignore_spin_events_ = true;
        ui->joint2Spin->setValue(value);
        ignore_spin_events_ = false;
        
        std::vector<double> new_joints = current_joint_values_;
        new_joints[1] = value;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onJoint3SliderChanged(int value)
{
    if (!ignore_slider_events_) {
        ignore_spin_events_ = true;
        ui->joint3Spin->setValue(value);
        ignore_spin_events_ = false;
        
        double radians = degToRad(value);
        std::vector<double> new_joints = current_joint_values_;
        new_joints[2] = radians;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onJoint4SliderChanged(int value)
{
    if (!ignore_slider_events_) {
        ignore_spin_events_ = true;
        ui->joint4Spin->setValue(value);
        ignore_spin_events_ = false;
        
        double radians = degToRad(value);
        std::vector<double> new_joints = current_joint_values_;
        new_joints[3] = radians;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onJoint5SliderChanged(int value)
{
    if (ignore_slider_events_)
        return;
    
    ignore_spin_events_ = true;
    ui->joint5Spin->setValue(value);
    ignore_spin_events_ = false;
    
    // 将角度转换为弧度
    double angle_rad = degToRad(value);
    
    // 更新关节值
    current_joint_values_[4] = angle_rad;
    
    // 发送关节命令
    sendJointCommand();
}

void ArmControlGUI::onJoint6SliderChanged(int value)
{
    if (!ignore_slider_events_) {
        ignore_spin_events_ = true;
        ui->joint6Spin->setValue(value);
        ignore_spin_events_ = false;
        
        std::vector<double> new_joints = current_joint_values_;
        new_joints[5] = value;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onJoint1SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        ui->joint1Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        double radians = degToRad(value);
        std::vector<double> new_joints = current_joint_values_;
        new_joints[0] = radians;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onJoint2SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        ui->joint2Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        std::vector<double> new_joints = current_joint_values_;
        new_joints[1] = value;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onJoint3SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        ui->joint3Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        double radians = degToRad(value);
        std::vector<double> new_joints = current_joint_values_;
        new_joints[2] = radians;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onJoint4SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        ui->joint4Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        double radians = degToRad(value);
        std::vector<double> new_joints = current_joint_values_;
        new_joints[3] = radians;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onJoint5SpinChanged(double value)
{
    if (ignore_spin_events_)
        return;
    
    ignore_slider_events_ = true;
    ui->joint5Slider->setValue(static_cast<int>(value));
    ignore_slider_events_ = false;
    
    // 将角度转换为弧度
    double angle_rad = degToRad(value);
    
    // 更新关节值
    current_joint_values_[4] = angle_rad;
    
    // 发送关节命令
    sendJointCommand();
}

void ArmControlGUI::onJoint6SpinChanged(double value)
{
    if (!ignore_spin_events_) {
        ignore_slider_events_ = true;
        ui->joint6Slider->setValue(static_cast<int>(value));
        ignore_slider_events_ = false;
        
        std::vector<double> new_joints = current_joint_values_;
        new_joints[5] = value;
        
        sendJointCommand(new_joints);
    }
}

void ArmControlGUI::onVacuumPowerSliderChanged(int value)
{
    if (vacuum_on_) {
        sendVacuumCommand(true, value);
    }
    vacuum_power_ = value;
}

void ArmControlGUI::onVacuumOnButtonClicked()
{
    sendVacuumCommand(true, vacuum_power_);
}

void ArmControlGUI::onVacuumOffButtonClicked()
{
    sendVacuumCommand(false, vacuum_power_);
}

void ArmControlGUI::on_vacuumOnButton_clicked()
{
    onVacuumOnButtonClicked();
}

void ArmControlGUI::on_vacuumOffButton_clicked()
{
    onVacuumOffButtonClicked();
}

void ArmControlGUI::onHomeButtonClicked()
{
    sendHomeCommand();
}

void ArmControlGUI::on_homeButton_clicked()
{
    onHomeButtonClicked();
}

bool ArmControlGUI::eventFilter(QObject* obj, QEvent* event)
{
    // 简单实现，不做任何特殊处理
    return QMainWindow::eventFilter(obj, event);
}

void ArmControlGUI::closeEvent(QCloseEvent* event)
{
    // 关闭窗口时的处理
    QMainWindow::closeEvent(event);
}

// Helper functions for kinematics
std::vector<double> ArmControlGUI::poseToJoints(const geometry_msgs::Pose& pose)
{
    return inverseKinematics(pose, current_joint_values_);
}

geometry_msgs::Pose ArmControlGUI::jointsToPos(const std::vector<double>& joint_values)
{
    return forwardKinematics(joint_values);
}

// 实现移动到指定位置的槽函数
void ArmControlGUI::onMoveToPositionClicked()
{
    // 获取用户输入的目标位置
    double x = ui->posXSpin->value();
    double y = ui->posYSpin->value();
    double z = ui->posZSpin->value();
    
    // 创建目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x / 100.0;  // 转换为米
    target_pose.position.y = y / 100.0;
    target_pose.position.z = z / 100.0;
    
    // 设置方向（使末端执行器朝下）
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.7071;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.7071;
    
    // 记录日志
    logMessage(QString("正在移动到位置: X=%1, Y=%2, Z=%3 cm").arg(x).arg(y).arg(z));
    
    try {
        // 使用逆运动学计算关节角度
        std::vector<double> target_joints = inverseKinematics(target_pose, current_joint_values_);
        
        // 检查是否找到有效解
        if (target_joints.empty()) {
            logMessage("无法找到有效的逆运动学解决方案");
            return;
        }
        
        // 更新关节值
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

// Qt Designer自动生成的槽函数
void ArmControlGUI::on_moveToPositionButton_clicked()
{
    onMoveToPositionClicked();
}





// 未实现的方法存根
void ArmControlGUI::onPickButtonClicked() {}
void ArmControlGUI::onPlaceButtonClicked() {}
void ArmControlGUI::onOpenTaskSequence() {}
void ArmControlGUI::onSaveTaskSequence() {}
void ArmControlGUI::onExitApplication() {}
void ArmControlGUI::onRobotSettings() {}
void ArmControlGUI::onAbout() {}
void ArmControlGUI::onDetectionsTableCellClicked(int row, int column) {}
void ArmControlGUI::onCameraSwitchButtonClicked() {}
void ArmControlGUI::on3DViewObjectSelected(int index) {}
void ArmControlGUI::updateJointInfo() {}
void ArmControlGUI::updateCameraViews() {}
void ArmControlGUI::updateCameraView()
{
    ui_processing_ = true;
    
    try {
        // 根据当前视图模式显示相应的图像
        QLabel* cameraView = findChild<QLabel*>("cameraView");
        if (!cameraView) {
            ROS_WARN("无法找到相机视图标签组件");
            ui_processing_ = false;
            return;
        }
        
        // 如果相机不可用，直接显示占位图像
        if (!is_camera_available_) {
            if (!current_camera_image_.isNull()) {
                try {
                    QPixmap pixmap = QPixmap::fromImage(current_camera_image_);
                    if (!pixmap.isNull() && cameraView->width() > 0 && cameraView->height() > 0) {
                        pixmap = pixmap.scaled(cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                        cameraView->setPixmap(pixmap);
                    } else {
                        ROS_WARN("无效的图像或视图大小");
                    }
                } catch (const std::exception& e) {
                    ROS_ERROR("显示相机占位图像时出错: %s", e.what());
                }
            }
            ui_processing_ = false;
            return;
        }
        
        QPixmap pixmap;
        bool pixmapSet = false;
        
        // 根据当前视图模式选择要显示的图像
        switch (camera_view_mode_) {
            case 0: // 左视图
                if (!left_camera_image_.isNull()) {
                    try {
                        QImage displayImage = left_camera_image_;
                        
                        // 如果启用了YOLO检测并要显示检测框
                        if (yolo_enabled_ && show_detection_boxes_ && !detected_objects_.empty()) {
                            // 创建可绘制的副本
                            displayImage = left_camera_image_.copy();
                            // 绘制检测框
                            drawDetectionBoxes(displayImage, detected_objects_);
                            // 如果需要显示距离信息
                            if (show_distance_overlay_ && depth_available_) {
                                showObjectDistanceOverlay(displayImage, detected_objects_);
                            }
                        }
                        
                        pixmap = QPixmap::fromImage(displayImage);
                        if (!pixmap.isNull()) {
                            // 确保窗口有效大小
                            if (cameraView->width() > 0 && cameraView->height() > 0) {
                                pixmap = pixmap.scaled(cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                                cameraView->setPixmap(pixmap);
                                pixmapSet = true;
                                
                                // 更新标题
                                this->setWindowTitle("机械臂控制 - 左视图");
                            } else {
                                ROS_WARN("相机视图窗口大小无效 [%d x %d]", 
                                    cameraView->width(), cameraView->height());
                            }
                        }
                    } catch (const std::exception& e) {
                        ROS_ERROR("显示左视图时出错: %s", e.what());
                    }
                }
                break;
                
            case 1: // 右视图
                if (!right_camera_image_.isNull()) {
                    try {
                        QImage displayImage = right_camera_image_;
                        
                        // 如果启用了YOLO检测并要显示检测框
                        if (yolo_enabled_ && show_detection_boxes_ && !detected_objects_.empty()) {
                            // 创建可绘制的副本
                            displayImage = right_camera_image_.copy();
                            // 绘制检测框
                            drawDetectionBoxes(displayImage, detected_objects_);
                            // 如果需要显示距离信息
                            if (show_distance_overlay_ && depth_available_) {
                                showObjectDistanceOverlay(displayImage, detected_objects_);
                            }
                        }
                        
                        pixmap = QPixmap::fromImage(displayImage);
                        if (!pixmap.isNull()) {
                            // 确保窗口有效大小
                            if (cameraView->width() > 0 && cameraView->height() > 0) {
                                pixmap = pixmap.scaled(cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                                cameraView->setPixmap(pixmap);
                                pixmapSet = true;
                                
                                // 更新标题
                                this->setWindowTitle("机械臂控制 - 右视图");
                            }
                        }
                    } catch (const std::exception& e) {
                        ROS_ERROR("显示右视图时出错: %s", e.what());
                    }
                }
                break;
                
            case 2: // 深度视图
                if (!current_depth_image_.isNull()) {
                    try {
                        pixmap = QPixmap::fromImage(current_depth_image_);
                        if (!pixmap.isNull()) {
                            // 确保窗口有效大小
                            if (cameraView->width() > 0 && cameraView->height() > 0) {
                                pixmap = pixmap.scaled(cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                                cameraView->setPixmap(pixmap);
                                pixmapSet = true;
                                
                                // 更新标题
                                this->setWindowTitle("机械臂控制 - 深度视图");
                            }
                        }
                    } catch (const std::exception& e) {
                        ROS_ERROR("显示深度视图时出错: %s", e.what());
                    }
                }
                break;
                
            case 3: // YOLO检测视图
                if (!detection_image_.isNull()) {
                    try {
                        pixmap = QPixmap::fromImage(detection_image_);
                        if (!pixmap.isNull()) {
                            // 确保窗口有效大小
                            if (cameraView->width() > 0 && cameraView->height() > 0) {
                                pixmap = pixmap.scaled(cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                                cameraView->setPixmap(pixmap);
                                pixmapSet = true;
                                
                                // 更新标题
                                this->setWindowTitle("机械臂控制 - 目标检测视图");
                            }
                        }
                    } catch (const std::exception& e) {
                        ROS_ERROR("显示检测视图时出错: %s", e.what());
                    }
                }
                break;
        }
        
        // 如果没有设置任何图像，显示默认占位图像
        if (!pixmapSet) {
            if (camera_view_mode_ == 0 && left_camera_image_.isNull()) {
                createPlaceholderImage("左视图暂不可用");
            } else if (camera_view_mode_ == 1 && right_camera_image_.isNull()) {
                createPlaceholderImage("右视图暂不可用");
            } else if (camera_view_mode_ == 2 && current_depth_image_.isNull()) {
                createPlaceholderImage("深度视图暂不可用");
            } else if (camera_view_mode_ == 3 && detection_image_.isNull()) {
                createPlaceholderImage("检测视图暂不可用");
            } else {
                createPlaceholderImage("相机视图不可用");
            }
            
            pixmap = QPixmap::fromImage(current_camera_image_);
            if (!pixmap.isNull() && cameraView->width() > 0 && cameraView->height() > 0) {
                pixmap = pixmap.scaled(cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
                cameraView->setPixmap(pixmap);
            }
        }
        
        // 更新检测结果表格
        updateDetectionsTable();
        
    } catch (const std::exception& e) {
        ROS_ERROR("更新相机视图时出错: %s", e.what());
    } catch (...) {
        ROS_ERROR("更新相机视图时出现未知错误");
    }
    
    ui_processing_ = false;
}

void ArmControlGUI::updateDetectionsTable() {}
void ArmControlGUI::updateConnectionStatus() {}
void ArmControlGUI::updateJointControlWidgets() {}
void ArmControlGUI::initializeGUI()
{
    // 初始化相机视图相关UI（先查找是否存在这些按钮）
    QPushButton* leftViewButton = findChild<QPushButton*>("leftViewButton");
    if (leftViewButton) {
        leftViewButton->setToolTip("显示左相机视图");
        connect(leftViewButton, &QPushButton::clicked, this, &ArmControlGUI::onLeftViewButtonClicked);
    } else {
        // 如果按钮不存在，可以考虑动态创建
        ROS_WARN("leftViewButton not found in UI");
    }
    
    QPushButton* rightViewButton = findChild<QPushButton*>("rightViewButton");
    if (rightViewButton) {
        rightViewButton->setToolTip("显示右相机视图");
        connect(rightViewButton, &QPushButton::clicked, this, &ArmControlGUI::onRightViewButtonClicked);
    } else {
        ROS_WARN("rightViewButton not found in UI");
    }
    
    QPushButton* depthViewButton = findChild<QPushButton*>("depthViewButton");
    if (depthViewButton) {
        depthViewButton->setToolTip("显示深度视图");
        depthViewButton->setEnabled(false); // 初始禁用，等待深度图可用时启用
        connect(depthViewButton, &QPushButton::clicked, this, &ArmControlGUI::onDepthViewButtonClicked);
    } else {
        ROS_WARN("depthViewButton not found in UI");
    }
    
    // 初始化图像
    createPlaceholderImage();
    
    // 默认视图模式为左视图
    camera_view_mode_ = 0;
}

void ArmControlGUI::initializeOpenGL() {}
void ArmControlGUI::initializeJointControlConnections() {}
void ArmControlGUI::setupROSSubscriptions()
{
    // 关节状态订阅
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, 
                                   &ArmControlGUI::jointStateCallback, this);
    
    // 相机图像订阅
    stereo_merged_sub_ = nh_.subscribe("/stereo_camera/merged_image", 1, 
                                     &ArmControlGUI::stereoMergedCallback, this);
    depth_image_sub_ = nh_.subscribe("/stereo_camera/depth", 1, 
                                   &ArmControlGUI::depthImageCallback, this);
    
    // YOLO检测结果订阅
    detection_image_sub_ = nh_.subscribe("/yolo_detector/detection_image", 1, 
                                      &ArmControlGUI::detectionImageCallback, this);
    detection_poses_sub_ = nh_.subscribe("/yolo_detector/detection_poses", 1, 
                                      &ArmControlGUI::detectionPosesCallback, this);
    yolo_status_sub_ = nh_.subscribe("/yolo_detector/status", 1, 
                                   &ArmControlGUI::yoloStatusCallback, this);
    
    // 同步订阅
    detection_image_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(
        nh_, "/yolo_detector/detection_image", 1);
    detection_poses_sync_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::PoseArray>>(
        nh_, "/yolo_detector/detection_poses", 1);
    
    // 设置同步策略
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *detection_image_sync_sub_, *detection_poses_sync_sub_);
    
    // 注册同步回调
    sync_->registerCallback(boost::bind(&ArmControlGUI::objectDetectionCallback, 
                                     this, _1, _2));
    
    // 发布器
    camera_view_mode_pub_ = nh_.advertise<std_msgs::Int32>("/stereo_camera/view_mode", 1, true);
}
void ArmControlGUI::createMenus() {}
void ArmControlGUI::setupCameraParameters() {}
bool ArmControlGUI::checkJointLimits(const std::vector<double>& joint_values)
{
    if (!kinematics_utils_) {
        // Fallback if kinematics_utils_ is not initialized
        ROS_ERROR("Kinematics utilities not initialized, cannot check joint limits");
        return false;
    }
    return kinematics_utils_->checkJointLimits(joint_values);
}
void ArmControlGUI::sendRelayOrder(const std::string& command) {}
void ArmControlGUI::sendPickCommand(const std::string& object_id) {}
void ArmControlGUI::sendPlaceCommand(double x, double y, double z) {}
void ArmControlGUI::sendPickObjectCommand(int object_index) {}
void ArmControlGUI::enableObjectDetection(bool enable) {}
void ArmControlGUI::updateCameraTransform(const geometry_msgs::Pose& end_effector_pose) {}
QVector3D ArmControlGUI::imagePointTo3D(const QPoint& image_point, float depth) { return QVector3D(); }
QPoint ArmControlGUI::point3DToImage(const QVector3D& point_3d) { return QPoint(); }
float ArmControlGUI::getDepthAtPoint(const QPoint& image_point) { return 0.0f; }
void ArmControlGUI::onCameraViewClicked(QPoint pos) {}
void ArmControlGUI::handleCameraError(const std::string& error_msg) {}
void ArmControlGUI::createPlaceholderImage(const std::string& message)
{
    int width = 640;
    int height = 480;
    
    // 创建占位图像
    QImage placeholder(width, height, QImage::Format_RGB888);
    placeholder.fill(QColor(40, 40, 40)); // 深灰色背景
    
    // 添加文本
    QPainter painter(&placeholder);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 设置字体
    QFont font = painter.font();
    font.setPointSize(16);
    font.setBold(true);
    painter.setFont(font);
    
    // 设置文本样式
    painter.setPen(Qt::white);
    
    // 绘制文本
    QString text = message.empty() ? "相机未连接" : QString::fromStdString(message);
    QRect textRect = painter.fontMetrics().boundingRect(text);
    int x = (width - textRect.width()) / 2;
    int y = (height - textRect.height()) / 2 + painter.fontMetrics().ascent();
    painter.drawText(x, y, text);
    
    // 在下方添加提示
    font.setPointSize(12);
    painter.setFont(font);
    QString hint = "正在尝试连接，请稍候...";
    textRect = painter.fontMetrics().boundingRect(hint);
    x = (width - textRect.width()) / 2;
    y += 40;
    painter.drawText(x, y, hint);
    
    // 保存占位图像
    current_camera_image_ = placeholder;
}

QImage ArmControlGUI::cvMatToQImage(const cv::Mat& mat)
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

void ArmControlGUI::attemptCameraReconnect() 
{
    // 如果相机已经可用，不需要重连
    if (is_camera_available_) {
        return;
    }
    
    ROS_INFO("尝试重新连接相机...");
    
    // 这里只需要显示更新后的状态信息，实际重连由stereo_camera_node负责
    // 我们只需要在这里更新UI状态
    
    // 更新占位图像消息，告知用户我们正在尝试重连
    static int reconnect_attempts = 0;
    reconnect_attempts++;
    
    QString message = QString("相机未连接，正在尝试重连... (尝试 %1 次)").arg(reconnect_attempts);
    createPlaceholderImage(message.toStdString());
    
    // 更新相机视图
    updateCameraView();
    
    // 记录到状态栏
    statusBar()->showMessage(message, 2000);
    
    // 检查是否有相机可用
    if (findAvailableCamera()) {
        ROS_INFO("找到可用相机");
        is_camera_available_ = true;
        reconnect_attempts = 0;
        statusBar()->showMessage("相机已重新连接", 2000);
    }
}

bool ArmControlGUI::findAvailableCamera()
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
    return false;
}

void ArmControlGUI::stereoMergedCallback(const sensor_msgs::Image::ConstPtr& msg)
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

void ArmControlGUI::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
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
                // 原始深度图 - 转换为可视化格式
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
            
            // 创建彩色深度图用于显示
            cv::Mat colored_depth = createColoredDepthMap(depth_float);
            if (!colored_depth.empty()) {
                // 转换为QImage并保存
                QImage depth_qimg = cvMatToQImage(colored_depth);
                if (!depth_qimg.isNull()) {
                    current_depth_image_ = depth_qimg;
                    
                    // 如果当前是深度视图模式，还要更新当前显示图像
                    if (camera_view_mode_ == 2) {
                        current_camera_image_ = current_depth_image_;
                    }
                    
                    // 如果有检测对象，则估计物体距离
                    if (!detected_objects_.empty()) {
                        estimateObjectDistances(detected_objects_, current_depth_map_);
                    }
                } else {
                    ROS_WARN("彩色深度图转换为QImage失败");
                }
            } else {
                ROS_WARN("无法创建彩色深度图");
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
            
            // 创建彩色深度图用于显示
            cv::Mat colored_depth = createColoredDepthMap(depth_16u);
            if (!colored_depth.empty()) {
                // 转换为QImage并保存
                QImage depth_qimg = cvMatToQImage(colored_depth);
                if (!depth_qimg.isNull()) {
                    current_depth_image_ = depth_qimg;
                    
                    // 如果当前是深度视图模式，还要更新当前显示图像
                    if (camera_view_mode_ == 2) {
                        current_camera_image_ = current_depth_image_;
                    }
                    
                    // 如果有检测对象，则估计物体距离
                    if (!detected_objects_.empty()) {
                        estimateObjectDistances(detected_objects_, current_depth_map_);
                    }
                } else {
                    ROS_WARN("彩色深度图转换为QImage失败");
                }
            } else {
                ROS_WARN("无法创建彩色深度图");
            }
        }
        else if (msg->encoding == "bgr8") {
            try {
                // 已经是彩色深度图
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                
                // 安全检查：确保cv_ptr和cv_ptr->image有效
                if (!cv_ptr || cv_ptr->image.empty()) {
                    ROS_WARN("BGR8深度图转换后为空");
                    return;
                }
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("CV Bridge BGR8转换错误: %s", e.what());
                return;
            }
            
            cv::Mat depth_color = cv_ptr->image;
            if (depth_color.empty() || depth_color.cols <= 0 || depth_color.rows <= 0) {
                ROS_WARN("BGR8深度图为空或尺寸无效");
                return;
            }
            
            // 标记深度图可用，但此时不更新原始深度图，因为我们只有彩色版本
            depth_available_ = false;
            
            // 保存为Qt图像
            QImage qimg = cvMatToQImage(depth_color);
            if (!qimg.isNull()) {
                current_depth_image_ = qimg;
                
                // 如果当前是深度视图模式，还要更新当前显示图像
                if (camera_view_mode_ == 2) {
                    current_camera_image_ = current_depth_image_;
                }
            } else {
                ROS_WARN("BGR8深度图转换为QImage失败");
                return;
            }
        }
        else {
            ROS_WARN("不支持的深度图编码格式: %s", msg->encoding.c_str());
            return;
        }
        
        // 标记为相机可用
        is_camera_available_ = true;
        stereo_camera_error_count_ = 0;
        
        // 触发UI更新
        QMetaObject::invokeMethod(this, "updateCameraView", Qt::QueuedConnection);
        
    } catch (std::exception& e) {
        ROS_ERROR("处理深度图像时发生错误: %s", e.what());
    } catch (...) {
        ROS_ERROR("处理深度图像时发生未知异常");
    }
}
void ArmControlGUI::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg) {}
void ArmControlGUI::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {}
void ArmControlGUI::yoloStatusCallback(const std_msgs::Bool::ConstPtr& msg) {}
void ArmControlGUI::objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg, 
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
        QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
        
    } catch (std::exception& e) {
        ROS_ERROR("处理检测结果时发生错误: %s", e.what());
    } catch (...) {
        ROS_ERROR("处理检测结果时发生未知异常");
    }
}
void ArmControlGUI::updateScene3D() {}
void ArmControlGUI::updateSceneObjects() {}
void ArmControlGUI::onEndEffectorDragged(QVector3D position) {}
void ArmControlGUI::setupJointLimits()
{
    // 使用KinematicsUtils设置关节限制
    if (kinematics_utils_) {
        kinematics_utils_->setupJointLimits();
        ROS_INFO("关节限制已设置");
    } else {
        ROS_ERROR("Kinematics utilities not initialized, cannot set up joint limits");
    }
}
void ArmControlGUI::setupDHParameters()
{
    // 使用KinematicsUtils设置DH参数
    if (kinematics_utils_) {
        kinematics_utils_->setupDHParameters();
        ROS_INFO("DH参数已设置");
    } else {
        ROS_ERROR("Kinematics utilities not initialized, cannot set up DH parameters");
    }
}

// 实现左视图切换按钮槽函数
void ArmControlGUI::onLeftViewButtonClicked()
{
    camera_view_mode_ = 0;  // 设置为左视图模式
    ROS_INFO("切换到左视图");
    
    // 更新按钮状态
    QList<QPushButton*> viewButtons = {
        findChild<QPushButton*>("leftViewButton"),
        findChild<QPushButton*>("rightViewButton"),
        findChild<QPushButton*>("depthViewButton")
    };
    
    for (auto btn : viewButtons) {
        if (btn) {
            QString name = btn->objectName();
            if (name == "leftViewButton") {
                btn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
            } else {
                btn->setStyleSheet("");
            }
        }
    }
    
    // 发布相机视图模式
    std_msgs::Int32 mode_msg;
    mode_msg.data = camera_view_mode_;
    if (camera_view_mode_pub_) {
        camera_view_mode_pub_.publish(mode_msg);
    }
    
    // 更新显示
    updateCameraView();
}

void ArmControlGUI::on_leftViewButton_clicked()
{
    onLeftViewButtonClicked();
}

// 实现右视图切换按钮槽函数
void ArmControlGUI::onRightViewButtonClicked()
{
    camera_view_mode_ = 1;  // 设置为右视图模式
    ROS_INFO("切换到右视图");
    
    // 更新按钮状态
    QList<QPushButton*> viewButtons = {
        findChild<QPushButton*>("leftViewButton"),
        findChild<QPushButton*>("rightViewButton"),
        findChild<QPushButton*>("depthViewButton")
    };
    
    for (auto btn : viewButtons) {
        if (btn) {
            QString name = btn->objectName();
            if (name == "rightViewButton") {
                btn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
            } else {
                btn->setStyleSheet("");
            }
        }
    }
    
    // 发布相机视图模式
    std_msgs::Int32 mode_msg;
    mode_msg.data = camera_view_mode_;
    if (camera_view_mode_pub_) {
        camera_view_mode_pub_.publish(mode_msg);
    }
    
    // 更新显示
    updateCameraView();
}

void ArmControlGUI::on_rightViewButton_clicked()
{
    onRightViewButtonClicked();
}

// 实现深度视图切换按钮槽函数
void ArmControlGUI::onDepthViewButtonClicked()
{
    camera_view_mode_ = 2;  // 设置为深度视图模式
    ROS_INFO("切换到深度视图");
    
    // 更新按钮状态
    QList<QPushButton*> viewButtons = {
        findChild<QPushButton*>("leftViewButton"),
        findChild<QPushButton*>("rightViewButton"),
        findChild<QPushButton*>("depthViewButton")
    };
    
    for (auto btn : viewButtons) {
        if (btn) {
            QString name = btn->objectName();
            if (name == "depthViewButton") {
                btn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
            } else {
                btn->setStyleSheet("");
            }
        }
    }
    
    // 发布相机视图模式
    std_msgs::Int32 mode_msg;
    mode_msg.data = camera_view_mode_;
    if (camera_view_mode_pub_) {
        camera_view_mode_pub_.publish(mode_msg);
    }
    
    // 更新显示
    updateCameraView();
}

void ArmControlGUI::on_depthViewButton_clicked()
{
    onDepthViewButtonClicked();
}

// 添加日志消息
void ArmControlGUI::logMessage(const QString& message)
{
    // 记录日志到状态栏
    statusBar()->showMessage(message, 3000);  // 显示3秒
    
    // 如果有日志区域，同时记录到日志区域
    QTextEdit* logArea = findChild<QTextEdit*>("logTextEdit");
    if (logArea) {
        QString timestamp = QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ");
        logArea->append(timestamp + message);
        
        // 滚动到最新的消息
        QTextCursor cursor = logArea->textCursor();
        cursor.movePosition(QTextCursor::End);
        logArea->setTextCursor(cursor);
    }
    
    // 同时输出到ROS日志
    ROS_INFO("%s", message.toStdString().c_str());
}

void ArmControlGUI::goToHomePosition()
{
    // Home Position是机械臂的初始安全位置
    logMessage("正在将机械臂移动到初始位置...");
    
    // 设置初始位置的关节角度
    std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0, 0.0, 10.0};  // 根据实际机械臂调整
    
    // 更新当前关节值
    current_joint_values_ = home_position;
    
    // 更新GUI
    updateGUIJointValues();
    
    // 发送关节命令
    sendJointCommand();
}

cv::Mat ArmControlGUI::qImageToCvMat(const QImage& image)
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

QImage ArmControlGUI::overlayText(const QImage& image, const QString& text, 
                               const QPoint& position, const QColor& color, int fontSize)
{
    if (image.isNull()) {
        ROS_ERROR("无法在空图像上叠加文本");
        return QImage();
    }
    
    try {
        // 创建可绘制的副本
        QImage result = image.copy();
        QPainter painter(&result);
        painter.setRenderHint(QPainter::Antialiasing);
        
        // 设置字体
        QFont font = painter.font();
        font.setPointSize(fontSize);
        font.setBold(true);
        painter.setFont(font);
        
        // 设置文本样式
        painter.setPen(QPen(Qt::black, 2)); // 外描边
        
        // 计算文本范围
        QFontMetrics fm(font);
        QRect textRect = fm.boundingRect(text);
        QRect backgroundRect = textRect.adjusted(-5, -2, 5, 2);
        backgroundRect.moveTopLeft(position);
        
        // 绘制半透明背景
        painter.fillRect(backgroundRect, QColor(0, 0, 0, 128));
        
        // 绘制文本描边
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx != 0 || dy != 0) {
                    painter.drawText(position.x() + dx, position.y() + dy + fm.ascent(), text);
                }
            }
        }
        
        // 绘制文本
        painter.setPen(color);
        painter.drawText(position.x(), position.y() + fm.ascent(), text);
        
        return result;
    } catch (const std::exception& e) {
        ROS_ERROR("叠加文本出错: %s", e.what());
        return image;  // 返回原始图像
    } catch (...) {
        ROS_ERROR("叠加文本出现未知错误");
        return image;  // 返回原始图像
    }
}

cv::Mat ArmControlGUI::createColoredDepthMap(const cv::Mat& depth_map)
{
    // 检查输入
    if (depth_map.empty() || (depth_map.type() != CV_32FC1 && depth_map.type() != CV_16UC1)) {
        ROS_WARN("深度图为空或类型不支持");
        return cv::Mat();
    }
    
    try {
        cv::Mat normalized_depth;
        double min_val = 0.1, max_val = 5.0; // 默认范围，单位米
        
        if (depth_map.type() == CV_32FC1) {
            // 32位浮点深度图
            // 创建有效区域掩码
            cv::Mat valid_mask = depth_map > 0.001; // 忽略接近0的无效值
            
            if (cv::countNonZero(valid_mask) > 0) {
                // 寻找有效深度值范围
                cv::minMaxLoc(depth_map, &min_val, &max_val, nullptr, nullptr, valid_mask);
                
                // 限制范围
                min_val = std::max(min_val, 0.1); // 至少10厘米
                max_val = std::min(max_val, 10.0); // 最多10米
                
                // 归一化深度图
                depth_map.copyTo(normalized_depth, valid_mask);
                normalized_depth = (normalized_depth - min_val) / (max_val - min_val);
                
                // 将范围限制在[0,1]
                cv::threshold(normalized_depth, normalized_depth, 1.0, 1.0, cv::THRESH_TRUNC);
                cv::threshold(normalized_depth, normalized_depth, 0.0, 0.0, cv::THRESH_TOZERO);
            } else {
                ROS_WARN("深度图中没有有效值");
                return cv::Mat();
            }
        } 
        else if (depth_map.type() == CV_16UC1) {
            // 16位整数深度图
            // 使用典型的转换比例
            double scale = 0.001; // 通常16位深度为毫米，转换为米
            
            cv::Mat depth_float;
            depth_map.convertTo(depth_float, CV_32FC1, scale);
            
            // 创建有效区域掩码
            cv::Mat valid_mask = depth_float > 0.001;
            
            if (cv::countNonZero(valid_mask) > 0) {
                // 寻找有效深度值范围
                cv::minMaxLoc(depth_float, &min_val, &max_val, nullptr, nullptr, valid_mask);
                
                // 限制范围
                min_val = std::max(min_val, 0.1);
                max_val = std::min(max_val, 10.0);
                
                // 归一化深度图
                depth_float.copyTo(normalized_depth, valid_mask);
                normalized_depth = (normalized_depth - min_val) / (max_val - min_val);
                
                // 将范围限制在[0,1]
                cv::threshold(normalized_depth, normalized_depth, 1.0, 1.0, cv::THRESH_TRUNC);
                cv::threshold(normalized_depth, normalized_depth, 0.0, 0.0, cv::THRESH_TOZERO);
            } else {
                ROS_WARN("深度图中没有有效值");
                return cv::Mat();
            }
        }
        
        // 转换为8位图像
        cv::Mat depth_8u;
        normalized_depth.convertTo(depth_8u, CV_8UC1, 255.0);
        
        // 应用彩色映射
        cv::Mat colored_depth;
        cv::applyColorMap(depth_8u, colored_depth, cv::COLORMAP_JET);
        
        // 使黑色部分透明（表示无效区域）
        cv::Mat mask = depth_map > 0.001;
        if (depth_map.type() == CV_16UC1) {
            mask = depth_map > 1; // 如果是16位，阈值应更高
        }
        
        // 给无效区域应用半透明灰色
        for (int y = 0; y < colored_depth.rows; y++) {
            for (int x = 0; x < colored_depth.cols; x++) {
                if (!mask.at<uchar>(y, x)) {
                    cv::Vec3b& pixel = colored_depth.at<cv::Vec3b>(y, x);
                    pixel[0] = 30; // B
                    pixel[1] = 30; // G
                    pixel[2] = 30; // R
                }
            }
        }
        
        return colored_depth;
    } 
    catch (const cv::Exception& e) {
        ROS_ERROR("创建彩色深度图时出错: %s", e.what());
        return cv::Mat();
    } 
    catch (const std::exception& e) {
        ROS_ERROR("创建彩色深度图时出错: %s", e.what());
        return cv::Mat();
    } 
    catch (...) {
        ROS_ERROR("创建彩色深度图时出现未知错误");
        return cv::Mat();
    }
}

void ArmControlGUI::estimateObjectDistances(const std::vector<DetectedObject>& objects, 
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

void ArmControlGUI::drawDetectionBoxes(QImage& image, const std::vector<DetectedObject>& objects)
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
                labelText += QString(" (%.1f cm)").arg(obj.z);
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

void ArmControlGUI::showObjectDistanceOverlay(QImage& image, const std::vector<DetectedObject>& objects)
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
