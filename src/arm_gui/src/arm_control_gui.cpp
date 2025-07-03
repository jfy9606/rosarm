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
    
    // 设置标志
    ignore_slider_events_ = false;
    ignore_spin_events_ = false;
    ui_processing_ = false;
    is_camera_available_ = false;
    stereo_camera_error_count_ = 0;
    camera_view_mode_ = 0;  // 默认左视图
    
    // 创建占位图像
    createPlaceholderImage();
    
    // 将界面设置为初始状态
    updateGUIJointValues();
    
    // 更新UI
    updateUI();
    
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
    // 初始化成员变量
    current_joint_values_ = std::vector<double>(6, 0.0);  // 6个关节，初始值为0
    ignore_slider_events_ = false;
    ignore_spin_events_ = false;
    ui_processing_ = false;
    arm_ready_ = false;
    
    // 初始化末端执行器状态
    current_end_position_ = QVector3D(0, 0, 30);  // 初始位置，单位厘米
    current_end_orientation_ = QQuaternion(1, 0, 0, 0);  // 初始方向（单位四元数）
    
    // 初始化真空吸盘状态
    vacuum_on_ = false;
    vacuum_power_ = 50;  // 默认功率50%
    
    // 初始化相机状态
    camera_view_mode_ = 0;  // 默认左视图
    is_camera_available_ = false;
    stereo_camera_error_count_ = 0;
    
    // 初始化DH参数和关节限制
    setupDHParameters();
    setupJointLimits();
    
    // 初始化定时器
    updateTimer = new QTimer(this);
    updateTimer->setInterval(100);  // 100ms更新一次
    updateTimer->start();
    
    // 连接定时器信号
    connect(updateTimer, &QTimer::timeout, this, &ArmControlGUI::updateUI);
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
            QString posText = QString("末端位置: X=%.2f, Y=%.2f, Z=%.2f cm")
                .arg(current_end_position_.x())
                .arg(current_end_position_.y())
                .arg(current_end_position_.z());
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
    return kinematics_utils_->degToRad(deg);
}

double ArmControlGUI::radToDeg(double rad)
{
    return kinematics_utils_->radToDeg(rad);
}

// 使用KinematicsUtils实现正向运动学计算
geometry_msgs::Pose ArmControlGUI::forwardKinematics(const std::vector<double>& joint_values)
{
    return kinematics_utils_->forwardKinematics(joint_values);
}

// 使用KinematicsUtils实现逆运动学计算
std::vector<double> ArmControlGUI::inverseKinematics(const geometry_msgs::Pose& target_pose, 
                                                   const std::vector<double>& initial_guess)
{
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
    
    // 根据当前视图模式显示相应的图像
    QLabel* cameraView = findChild<QLabel*>("cameraView");
    if (!cameraView) {
        ui_processing_ = false;
        return;
    }
    
    // 根据当前视图模式选择要显示的图像
    switch (camera_view_mode_) {
        case 0: // 左视图
            if (!left_camera_image_.isNull()) {
                cameraView->setPixmap(QPixmap::fromImage(left_camera_image_).scaled(
                    cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            } else {
                createPlaceholderImage();
                cameraView->setPixmap(QPixmap::fromImage(current_camera_image_).scaled(
                    cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            }
            break;
            
        case 1: // 右视图
            if (!right_camera_image_.isNull()) {
                cameraView->setPixmap(QPixmap::fromImage(right_camera_image_).scaled(
                    cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            } else {
                createPlaceholderImage();
                cameraView->setPixmap(QPixmap::fromImage(current_camera_image_).scaled(
                    cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            }
            break;
            
        case 2: // 深度视图
            if (!current_depth_image_.isNull()) {
                cameraView->setPixmap(QPixmap::fromImage(current_depth_image_).scaled(
                    cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            } else {
                createPlaceholderImage("深度图不可用");
                cameraView->setPixmap(QPixmap::fromImage(current_camera_image_).scaled(
                    cameraView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            }
            break;
    }
    
    // 更新深度视图按钮状态
    QPushButton* depthViewButton = findChild<QPushButton*>("depthViewButton");
    if (depthViewButton) {
        depthViewButton->setEnabled(!current_depth_image_.isNull());
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
    // 添加摄像头相关订阅
    stereo_merged_sub_ = nh_.subscribe("/stereo_camera/current_view", 1, 
                                     &ArmControlGUI::stereoMergedCallback, this);
    
    // 订阅深度图像
    depth_image_sub_ = nh_.subscribe("/stereo_camera/depth/image_raw", 1,
                                    &ArmControlGUI::depthImageCallback, this);
    
    // 添加相机视图模式发布器
    camera_view_mode_pub_ = nh_.advertise<std_msgs::Int32>("/stereo_camera/view_mode", 1);
}
void ArmControlGUI::createMenus() {}
void ArmControlGUI::setupCameraParameters() {}
bool ArmControlGUI::checkJointLimits(const std::vector<double>& joint_values)
{
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
    // 创建一个占位图像，显示自定义消息
    cv::Mat placeholder(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));
    cv::putText(placeholder, message.empty() ? "等待摄像头连接..." : message, 
                cv::Point(150, 240), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 200, 200), 2);
    current_camera_image_ = cvMatToQImage(placeholder);
}

QImage ArmControlGUI::cvMatToQImage(const cv::Mat& mat)
{
    // 将OpenCV的Mat转换为Qt的QImage
    if(mat.type() == CV_8UC1) {
        // 灰度图像
        QImage image(mat.cols, mat.rows, QImage::Format_Grayscale8);
        for(int i = 0; i < mat.rows; i++) {
            memcpy(image.scanLine(i), mat.ptr(i), mat.cols);
        }
        return image;
    } else if(mat.type() == CV_8UC3) {
        // BGR颜色图像转换为RGB
        cv::Mat rgbMat;
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
        return QImage(rgbMat.data, rgbMat.cols, rgbMat.rows, 
                     static_cast<int>(rgbMat.step), QImage::Format_RGB888).copy();
    } else if(mat.type() == CV_8UC4) {
        // BGRA颜色图像
        return QImage(mat.data, mat.cols, mat.rows, 
                    static_cast<int>(mat.step), QImage::Format_ARGB32).copy();
    }
    
    // 不支持的格式，返回空图像
    return QImage();
}

void ArmControlGUI::attemptCameraReconnect() {}
bool ArmControlGUI::findAvailableCamera() { return false; }
void ArmControlGUI::stereoMergedCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cv_image = cv_ptr->image;
        
        if (!cv_image.empty()) {
            // 保存当前图像
            current_image_ = cv_image.clone();
            
            // 根据当前视图模式决定使用哪个图像
            if (msg->header.frame_id == "stereo_left" || msg->header.frame_id == "") {
                left_camera_image_ = cvMatToQImage(cv_image);
                if (camera_view_mode_ == 0) {
                    current_camera_image_ = left_camera_image_;
                }
            }
            else if (msg->header.frame_id == "stereo_right") {
                right_camera_image_ = cvMatToQImage(cv_image);
                if (camera_view_mode_ == 1) {
                    current_camera_image_ = right_camera_image_;
                }
            }
            else {
                // 通用处理，直接显示收到的图像
                current_camera_image_ = cvMatToQImage(cv_image);
            }
            
            // 标记相机为可用
            is_camera_available_ = true;
            stereo_camera_error_count_ = 0;
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV Bridge error: %s", e.what());
        stereo_camera_error_count_++;
        
        // 如果错误次数过多，标记相机不可用
        if (stereo_camera_error_count_ > 10) {
            is_camera_available_ = false;
            createPlaceholderImage(); // 创建一个占位图像
        }
    }
}

void ArmControlGUI::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 根据深度图编码格式进行不同处理
        if (msg->encoding == "32FC1") {
            // 原始深度图 - 转换为可视化格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat depth_float = cv_ptr->image;
            
            // 寻找有效深度值的范围
            double min_val = 0.1, max_val = 5.0; // 默认范围
            cv::Mat valid_mask = depth_float > 0;
            if (cv::countNonZero(valid_mask) > 0) {
                cv::minMaxLoc(depth_float, &min_val, &max_val, NULL, NULL, valid_mask);
                if (min_val < 0.1) min_val = 0.1; // 避免过小值
                if (max_val > 10.0) max_val = 10.0; // 避免过大值
            }
            
            // 归一化并转换为彩色图像
            cv::Mat depth_norm, depth_color;
            depth_float.copyTo(depth_norm, valid_mask);
            depth_norm = (depth_norm - min_val) / (max_val - min_val);
            depth_norm = depth_norm * 255;
            depth_norm.convertTo(depth_norm, CV_8UC1);
            cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);
            
            // 保存为Qt图像
            current_depth_image_ = cvMatToQImage(depth_color);
        }
        else if (msg->encoding == "16UC1") {
            // 16位无符号整数深度图
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth_16u = cv_ptr->image;
            
            // 转换为可视化格式
            cv::Mat depth_norm, depth_color;
            double scale = 0.001; // 默认比例因子
            depth_16u.convertTo(depth_norm, CV_8UC1, scale * 255.0 / 10.0); // 假设最大10米
            cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);
            
            // 保存为Qt图像
            current_depth_image_ = cvMatToQImage(depth_color);
        }
        else if (msg->encoding == "bgr8") {
            // 已经是彩色深度图
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat depth_color = cv_ptr->image;
            current_depth_image_ = cvMatToQImage(depth_color);
        }
        else {
            ROS_WARN("Unsupported depth image encoding: %s", msg->encoding.c_str());
            return;
        }
        
        // 标记为相机可用
        is_camera_available_ = true;
        stereo_camera_error_count_ = 0;
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV Bridge error: %s", e.what());
    }
}
void ArmControlGUI::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg) {}
void ArmControlGUI::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {}
void ArmControlGUI::yoloStatusCallback(const std_msgs::Bool::ConstPtr& msg) {}
void ArmControlGUI::objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg, 
                                           const geometry_msgs::PoseArray::ConstPtr& poses_msg) {}
void ArmControlGUI::updateScene3D() {}
void ArmControlGUI::updateSceneObjects() {}
void ArmControlGUI::onEndEffectorDragged(QVector3D position) {}
void ArmControlGUI::setupJointLimits()
{
    // 使用KinematicsUtils设置关节限制
    kinematics_utils_->setupJointLimits();
    ROS_INFO("关节限制已设置");
}
void ArmControlGUI::setupDHParameters()
{
    // 使用KinematicsUtils设置DH参数
    kinematics_utils_->setupDHParameters();
    ROS_INFO("DH参数已设置");
}

// 实现左视图切换按钮槽函数
void ArmControlGUI::onLeftViewButtonClicked()
{
    // 切换到左视图模式 (mode=0)
    std_msgs::Int32 view_mode_msg;
    view_mode_msg.data = 0;
    camera_view_mode_pub_.publish(view_mode_msg);
    camera_view_mode_ = 0;
    logMessage("切换到左视图模式");
}

void ArmControlGUI::on_leftViewButton_clicked()
{
    onLeftViewButtonClicked();
}

// 实现右视图切换按钮槽函数
void ArmControlGUI::onRightViewButtonClicked()
{
    // 切换到右视图模式 (mode=1)
    std_msgs::Int32 view_mode_msg;
    view_mode_msg.data = 1;
    camera_view_mode_pub_.publish(view_mode_msg);
    camera_view_mode_ = 1;
    logMessage("切换到右视图模式");
}

void ArmControlGUI::on_rightViewButton_clicked()
{
    onRightViewButtonClicked();
}

// 实现深度视图切换按钮槽函数
void ArmControlGUI::onDepthViewButtonClicked()
{
    // 切换到深度视图模式 (mode=2)
    std_msgs::Int32 view_mode_msg;
    view_mode_msg.data = 2;
    camera_view_mode_pub_.publish(view_mode_msg);
    camera_view_mode_ = 2;
    logMessage("切换到深度视图模式");
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
