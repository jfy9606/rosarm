#include "arm_gui/arm_control_gui.h"
#include "ui_arm_control_main.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QTableWidgetItem>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>

#include <opencv2/opencv.hpp>
#include <cmath>

ArmControlGUI::ArmControlGUI(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::ArmControlMainWindow)
    , nh_(nh)
    , tf_listener_(tf_buffer_)
    , vacuum_on_(false)
    , vacuum_power_(0)
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
}

ArmControlGUI::~ArmControlGUI()
{
    if (updateTimer) {
        updateTimer->stop();
        delete updateTimer;
    }
    
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
    
    // 连接任务控制按钮
    connect(ui->pickButton, &QPushButton::clicked, this, &ArmControlGUI::onPickButtonClicked);
    connect(ui->placeButton, &QPushButton::clicked, this, &ArmControlGUI::onPlaceButtonClicked);
    connect(ui->sequenceButton, &QPushButton::clicked, this, &ArmControlGUI::onSequenceButtonClicked);
    
    // 创建并连接示例动作按钮
    QGroupBox* demoGroup = new QGroupBox("示例动作", ui->controlTab);
    QVBoxLayout* demoLayout = new QVBoxLayout(demoGroup);
    
    QPushButton* demoButton1 = new QPushButton("示例1：自由度展示");
    QPushButton* demoButton2 = new QPushButton("示例2：正常姿态");
    QPushButton* demoButton3 = new QPushButton("示例3：大角度姿态");
    
    demoLayout->addWidget(demoButton1);
    demoLayout->addWidget(demoButton2);
    demoLayout->addWidget(demoButton3);
    
    // 将示例动作组添加到控制标签页的布局中
    QVBoxLayout* controlLayout = qobject_cast<QVBoxLayout*>(ui->controlTab->layout());
    if (controlLayout) {
        controlLayout->addWidget(demoGroup);
    } else {
        // 如果控制标签页没有布局，创建一个新的布局
        controlLayout = new QVBoxLayout(ui->controlTab);
        controlLayout->addWidget(demoGroup);
    }
    
    // 连接示例动作按钮信号
    connect(demoButton1, &QPushButton::clicked, this, &ArmControlGUI::onDemoAction1Clicked);
    connect(demoButton2, &QPushButton::clicked, this, &ArmControlGUI::onDemoAction2Clicked);
    connect(demoButton3, &QPushButton::clicked, this, &ArmControlGUI::onDemoAction3Clicked);
    
    // 连接检测表格
    connect(ui->detectionsTable, &QTableWidget::cellClicked, this, &ArmControlGUI::onDetectionsTableCellClicked);
    
    // 设置检测表格
    ui->detectionsTable->setColumnWidth(0, 50);  // ID
    ui->detectionsTable->setColumnWidth(1, 100); // 类型
    ui->detectionsTable->setColumnWidth(2, 80);  // X
    ui->detectionsTable->setColumnWidth(3, 80);  // Y
    ui->detectionsTable->setColumnWidth(4, 80);  // Z
    ui->detectionsTable->setColumnWidth(5, 100); // 操作
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
    
    // 订阅摄像头图像
    left_camera_sub_ = nh_.subscribe("/left_camera/image_raw", 1, &ArmControlGUI::leftCameraCallback, this);
    right_camera_sub_ = nh_.subscribe("/right_camera/image_raw", 1, &ArmControlGUI::rightCameraCallback, this);
    depth_image_sub_ = nh_.subscribe("/stereo_vision/depth_image", 1, &ArmControlGUI::depthImageCallback, this);
    detection_image_sub_ = nh_.subscribe("/stereo_vision/detection_image", 1, &ArmControlGUI::detectionImageCallback, this);
    
    // 订阅检测结果
    detection_poses_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/stereo_vision/detected_poses", 10, &ArmControlGUI::detectionPosesCallback, this);
    
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
}

void ArmControlGUI::onJoint4SliderChanged(int value)
{
    ui->joint4_spin->setValue(value);
    current_joint_values_[3] = value * M_PI / 180.0; // 转换为弧度
    sendJointCommand(current_joint_values_);
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
    // 如果是"操作"列
    if (column == 5 && row >= 0 && row < static_cast<int>(detected_objects_.size())) {
        // 抓取被选中的物体
        sendPickCommand(detected_objects_[row].id);
        logMessage(QString("抓取物体 %1").arg(QString::fromStdString(detected_objects_[row].id)));
    }
}

// 定时器槽实现
void ArmControlGUI::onUpdateGUI()
{
    // 在这里更新GUI元素，如3D视图、状态等
    updateJointControlWidgets();
    updateEndEffectorPose();
    updateCameraViews();
    updateDetectionsTable();
    
    // 更新OpenGL渲染
    ui->armView->update();
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
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        left_camera_image_ = cvMatToQImage(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in leftCameraCallback: %s", e.what());
    }
}

void ArmControlGUI::rightCameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        right_camera_image_ = cvMatToQImage(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in rightCameraCallback: %s", e.what());
    }
}

void ArmControlGUI::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
        depth_image_ = cvMatToQImage(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in depthImageCallback: %s", e.what());
    }
}

void ArmControlGUI::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        detection_image_ = cvMatToQImage(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in detectionImageCallback: %s", e.what());
    }
}

void ArmControlGUI::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // 清空之前的检测结果
    detected_objects_.clear();
    
    // 生成新的检测列表
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        DetectedObject obj;
        obj.id = "object_" + std::to_string(i);
        obj.type = "未知";
        obj.x = msg->poses[i].position.x * 100; // 转换为厘米
        obj.y = msg->poses[i].position.y * 100;
        obj.z = msg->poses[i].position.z * 100;
        obj.pose = msg->poses[i];
        detected_objects_.push_back(obj);
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
    // 更新左摄像头视图
    if (!left_camera_image_.isNull()) {
        ui->cameraViewLeft->setPixmap(QPixmap::fromImage(left_camera_image_.scaled(
            ui->cameraViewLeft->width(), ui->cameraViewLeft->height(), 
            Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    }
    
    // 更新右摄像头视图
    if (!right_camera_image_.isNull()) {
        ui->cameraViewRight->setPixmap(QPixmap::fromImage(right_camera_image_.scaled(
            ui->cameraViewRight->width(), ui->cameraViewRight->height(), 
            Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    }
    
    // 更新深度图
    if (!depth_image_.isNull()) {
        ui->depthView->setPixmap(QPixmap::fromImage(depth_image_.scaled(
            ui->depthView->width(), ui->depthView->height(), 
            Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    }
    
    // 更新物体检测视图
    if (!detection_image_.isNull()) {
        ui->detectionView->setPixmap(QPixmap::fromImage(detection_image_.scaled(
            ui->detectionView->width(), ui->detectionView->height(), 
            Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    }
}

void ArmControlGUI::updateDetectionsTable()
{
    // 清空表格
    ui->detectionsTable->setRowCount(0);
    
    // 填充检测到的物体信息
    for (size_t i = 0; i < detected_objects_.size(); ++i) {
        const DetectedObject& obj = detected_objects_[i];
        
        ui->detectionsTable->insertRow(i);
        
        // ID
        QTableWidgetItem* idItem = new QTableWidgetItem(QString::fromStdString(obj.id));
        ui->detectionsTable->setItem(i, 0, idItem);
        
        // 类型
        QTableWidgetItem* typeItem = new QTableWidgetItem(QString::fromStdString(obj.type));
        ui->detectionsTable->setItem(i, 1, typeItem);
        
        // 坐标
        QTableWidgetItem* xItem = new QTableWidgetItem(QString::number(obj.x, 'f', 1));
        ui->detectionsTable->setItem(i, 2, xItem);
        
        QTableWidgetItem* yItem = new QTableWidgetItem(QString::number(obj.y, 'f', 1));
        ui->detectionsTable->setItem(i, 3, yItem);
        
        QTableWidgetItem* zItem = new QTableWidgetItem(QString::number(obj.z, 'f', 1));
        ui->detectionsTable->setItem(i, 4, zItem);
        
        // 操作按钮
        QPushButton* pickButton = new QPushButton("抓取");
        ui->detectionsTable->setCellWidget(i, 5, pickButton);
        
        // 连接按钮信号
        connect(pickButton, &QPushButton::clicked, [this, obj]() {
            sendPickCommand(obj.id);
            logMessage(QString("抓取物体 %1").arg(QString::fromStdString(obj.id)));
        });
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