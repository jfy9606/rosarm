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
#include <QQuaternion>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ArmControlGUI::ArmControlGUI(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::ArmControlMainWindow)
    , nh_(nh)
{
    // 创建UI
    ui->setupUi(this);
    
    // 初始化成员变量
    initializeMembers();
    
    // 初始化GUI连接
    initializeGUI();
    
    // 初始化关节控制连接
    initializeJointControlConnections();
    
    // 初始化ROS
    initializeROS();
    
    // 初始化OpenGL
    initializeOpenGL();
    
    // 添加日志
    logMessage("控制界面初始化完成");
    
    // 设置相机图像显示区域
    ui->cameraView->setScaledContents(false);
    ui->cameraView->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    
    // 设置鼠标追踪，以便实现图像点击选择物体
    ui->cameraView->setMouseTracking(true);
    ui->cameraView->installEventFilter(this);
    
    // 初始化3D场景
    scene_3d_renderer_ = new Scene3DRenderer(ui->openGLView);
    ui->openGLView->setLayout(new QHBoxLayout());
    ui->openGLView->layout()->addWidget(scene_3d_renderer_);
    
    // 连接3D场景物体选择信号
    connect(scene_3d_renderer_, &Scene3DRenderer::objectSelected, this, &ArmControlGUI::on3DViewObjectSelected);
    
    // 设置相机参数
    setupCameraParameters();
    
    // 订阅ROS话题
    setupROSSubscriptions();
    
    // 打印订阅的话题信息
    ROS_INFO("GUI已订阅摄像头话题:");
    ROS_INFO(" - /stereo_camera/image_merged (合成双目图像)");
    ROS_INFO(" - /camera/image_raw (原始摄像头图像)");
    ROS_INFO(" - /stereo_camera/detection_image");
    ROS_INFO(" - /yolo_detection/image");
    ROS_INFO(" - /yolo_detection/poses");
    
    // 输出调试信息到GUI日志
    logMessage("已订阅图像和检测结果话题，等待数据...");
    logMessage("3D视图已准备就绪，可通过鼠标右键旋转视角，滚轮缩放");
    
    // 设置窗口标题
    setWindowTitle("机械臂控制面板");
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
    
    // 释放消息过滤器同步器
    if (object_detection_sync_) {
        delete object_detection_sync_;
        object_detection_sync_ = nullptr;
    }
    
    // 释放3D渲染器
    if (scene_3d_renderer_) {
        delete scene_3d_renderer_;
    }
    
    delete ui;
}

// 事件过滤器实现，用于处理相机视图的鼠标事件
bool ArmControlGUI::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == ui->cameraView) {
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                // 处理相机视图点击
                onCameraViewClicked(mouseEvent->pos());
                return true;
            }
        }
    }
    
    return QMainWindow::eventFilter(watched, event);
}

// 初始化函数实现
void ArmControlGUI::initializeGUI()
{
    // 连接菜单操作
    connect(ui->actionOpen_Task_Sequence, &QAction::triggered, this, &ArmControlGUI::onOpenTaskSequence);
    connect(ui->actionSave_Task_Sequence, &QAction::triggered, this, &ArmControlGUI::onSaveTaskSequence);
    connect(ui->actionExit, &QAction::triggered, this, &ArmControlGUI::onExitApplication);
    connect(ui->actionRobot_Settings, &QAction::triggered, this, &ArmControlGUI::onRobotSettings);
    connect(ui->actionAbout, &QAction::triggered, this, &ArmControlGUI::onAbout);
    
    // 连接末端执行器控制按钮
    connect(ui->moveToPositionButton, &QPushButton::clicked, this, &ArmControlGUI::onMoveToPositionClicked);
    connect(ui->homeButton, &QPushButton::clicked, this, &ArmControlGUI::onHomeButtonClicked);
    
    // 连接吸附控制
    connect(ui->vacuumPowerSlider, &QSlider::valueChanged, this, &ArmControlGUI::onVacuumPowerSliderChanged);
    connect(ui->vacuumOnButton, &QPushButton::clicked, this, &ArmControlGUI::onVacuumOnButtonClicked);
    connect(ui->vacuumOffButton, &QPushButton::clicked, this, &ArmControlGUI::onVacuumOffButtonClicked);
    
    // 连接UI文件中定义的路径规划控件
    connect(ui->scanObjectsButton, &QPushButton::clicked, this, &ArmControlGUI::onScanObjectsClicked);
    connect(ui->executePathButton, &QPushButton::clicked, this, &ArmControlGUI::onExecutePathClicked);
    connect(ui->visualizeWorkspaceButton, &QPushButton::clicked, this, &ArmControlGUI::onVisualizeWorkspaceClicked);
    
    // 保存placement_area_combo_指针，以便在路径规划中使用
    placement_area_combo_ = ui->placement_area_combo;
    placement_area_combo_->clear();
    placement_area_combo_->addItem("区域1", "area_1");
    placement_area_combo_->addItem("区域2", "area_2");
    placement_area_combo_->addItem("区域3", "area_3");
    
    // 创建控制模式选择下拉框
    control_mode_combo_ = new QComboBox(this);
    control_mode_combo_->addItem("关节控制模式", static_cast<int>(ArmControlMode::JOINT_CONTROL));
    control_mode_combo_->addItem("笛卡尔控制模式", static_cast<int>(ArmControlMode::CARTESIAN_CONTROL));
    control_mode_combo_->addItem("视觉伺服模式", static_cast<int>(ArmControlMode::VISUAL_SERVO));
    
    // 默认选择关节控制模式
    control_mode_combo_->setCurrentIndex(0);
    current_control_mode_ = ArmControlMode::JOINT_CONTROL;
    
    // 连接模式切换信号
    connect(control_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        current_control_mode_ = static_cast<ArmControlMode>(control_mode_combo_->itemData(index).toInt());
        logMessage(QString("切换到%1").arg(control_mode_combo_->currentText()));
    });
    
    // 添加控制模式选择到顶部工具栏
    QLabel* modeLabel = new QLabel("控制模式: ");
    ui->toolBar->addWidget(modeLabel);
    ui->toolBar->addWidget(control_mode_combo_);
    
    // 创建YOLO检测控制
    QGroupBox* visionGroup = new QGroupBox("视觉检测");
    QVBoxLayout* visionLayout = new QVBoxLayout(visionGroup);
    
    // YOLO检测开关已移除
    
    QLabel* infoLabel = new QLabel("使用机械臂末端摄像头进行检测");
    infoLabel->setWordWrap(true);
    visionLayout->addWidget(infoLabel);
    
    // 将视觉控制组添加到controlWidget的布局中
    QVBoxLayout* controlLayout = qobject_cast<QVBoxLayout*>(ui->controlWidget->layout());
    if (controlLayout) {
        controlLayout->addWidget(visionGroup);
    }
    
    // 连接检测表格的单元格点击信号
    connect(ui->detectionsTable, &QTableWidget::cellClicked, this, &ArmControlGUI::onDetectionsTableCellClicked);
    
    // 设置检测结果表格
    ui->detectionsTable->setColumnCount(6);
    ui->detectionsTable->setHorizontalHeaderLabels({"ID", "类型", "X", "Y", "Z", "操作"});
    ui->detectionsTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    // 设置相机视图鼠标追踪
    ui->cameraView->setMouseTracking(true);
    ui->cameraView->installEventFilter(this);
    
    // 添加日志
    logMessage("控制界面初始化完成，使用机械臂末端双目摄像头");
    logMessage("YOLO检测器已加载，使用YOLOv8n模型");
    
    // 创建检测结果状态标签
    QLabel* detectionStatusLabel = new QLabel("未启用物体检测");
    detectionStatusLabel->setObjectName("detectionStatusLabel");
    
    // 创建检测图像视图
    QLabel* detectionView = new QLabel();
    detectionView->setObjectName("detectionView");
    detectionView->setMinimumSize(QSize(320, 240));
    detectionView->setAlignment(Qt::AlignCenter);
    detectionView->setText("检测图像将显示在此处");
    
    // 添加检测状态标签和检测图像视图到布局中
    QVBoxLayout* detectionLayout = new QVBoxLayout();
    detectionLayout->addWidget(detectionStatusLabel);
    detectionLayout->addWidget(detectionView);
    
    // 将检测布局添加到界面中
    QVBoxLayout* rightLayout = qobject_cast<QVBoxLayout*>(ui->detectionsDisplay->layout());
    if (!rightLayout) {
        rightLayout = new QVBoxLayout(ui->detectionsDisplay);
        ui->detectionsDisplay->setLayout(rightLayout);
    }
    rightLayout->addLayout(detectionLayout);

    // 初始化YOLO检测开关复选框
    // YOLO检测开关已移除

    // 初始化放置区域下拉框
    placement_area_combo_ = new QComboBox(this);
    placement_area_combo_->addItem("区域A", "area_a");
    placement_area_combo_->addItem("区域B", "area_b");
    placement_area_combo_->addItem("区域C", "area_c");
    ui->toolBar->addWidget(new QLabel("放置区域:", this));
    ui->toolBar->addWidget(placement_area_combo_);
    
    // 初始化控制模式选择
    control_mode_combo_ = new QComboBox(this);
    control_mode_combo_->addItem("关节控制", static_cast<int>(ArmControlMode::JOINT_CONTROL));
    control_mode_combo_->addItem("笛卡尔控制", static_cast<int>(ArmControlMode::CARTESIAN_CONTROL));
    control_mode_combo_->addItem("视觉伺服", static_cast<int>(ArmControlMode::VISUAL_SERVO));
    ui->toolBar->addWidget(new QLabel("控制模式:", this));
    ui->toolBar->addWidget(control_mode_combo_);
    
    // 连接控制模式选择信号
    connect(control_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            [this](int index) {
                current_control_mode_ = static_cast<ArmControlMode>(
                    control_mode_combo_->itemData(index).toInt());
                updateUI();
            });
    
    // 设置默认控制模式
    control_mode_combo_->setCurrentIndex(0); // 默认使用关节控制模式
    
    // 连接笛卡尔控制"移动到位置"按钮
    QPushButton* moveToPositionButton = ui->centralwidget->findChild<QPushButton*>("moveToPositionButton1");
    if (moveToPositionButton) {
        connect(moveToPositionButton, &QPushButton::clicked, this, &ArmControlGUI::onMoveToPositionClicked);
    } else {
        // 尝试查找原始名称
        moveToPositionButton = ui->centralwidget->findChild<QPushButton*>("moveToPositionButton");
        if (moveToPositionButton) {
            connect(moveToPositionButton, &QPushButton::clicked, this, &ArmControlGUI::onMoveToPositionClicked);
        }
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
    // 设置DH参数 (基于examples/path/main.m)
    // a1 = 13, a2 = 13, a3 = 25, d2 = [0, 43], d5 = [5, 15]
    dh_params_.resize(6);
    // [type, d, theta, a, alpha]
    // type: 0=revolute, 1=prismatic
    dh_params_[0] = std::make_tuple(2, 0.0, 0.0, 0.0, 0.0);     // 底座旋转关节
    dh_params_[1] = std::make_tuple(1, 0.0, 0.0, 0.0, 0.0);     // 伸缩关节
    dh_params_[2] = std::make_tuple(2, 0.0, 0.0, 0.4, 0.0);     // 肩部关节
    dh_params_[3] = std::make_tuple(2, 0.0, 0.0, 0.4, 0.0);     // 肘部关节
    dh_params_[4] = std::make_tuple(2, 0.0, 0.0, 0.0, 90.0);    // 末端旋转关节
    dh_params_[5] = std::make_tuple(1, 0.0, 0.0, 0.0, 0.0);     // 末端伸缩关节
    
    // 初始化关节限制
    joint_limits_.resize(6);
    joint_limits_[0] = std::make_pair(-180.0, 180.0);  // 底座旋转
    joint_limits_[1] = std::make_pair(0.0, 43.0);      // 伸缩
    joint_limits_[2] = std::make_pair(-90.0, 90.0);    // 肩部
    joint_limits_[3] = std::make_pair(0.0, 180.0);     // 肘部
    joint_limits_[4] = std::make_pair(-180.0, 180.0);  // 末端旋转
    joint_limits_[5] = std::make_pair(5.0, 15.0);      // 末端伸缩

    // 初始化订阅者和发布者
    joint_state_sub_ = nh_.subscribe("/arm1/joint_states", 1, &ArmControlGUI::jointStateCallback, this);
    
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm1/joint_command", 1);
    arm_command_pub_ = nh_.advertise<std_msgs::String>("/arm_command", 1);
    vacuum_command_pub_ = nh_.advertise<std_msgs::Bool>("/arm1/vacuum_command", 1);
    vacuum_power_pub_ = nh_.advertise<std_msgs::Float64>("/arm1/vacuum_power", 1);
    gripper_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/arm1/gripper_command", 1);
    servo_control_pub_ = nh_.advertise<servo_wrist::SerControl>("/arm1/servo_control", 1);
    motor_order_pub_ = nh_.advertise<liancheng_socket::MotorOrder>("/motor_orders", 1);
    relay_order_pub_ = nh_.advertise<std_msgs::String>("/relay_orders", 1);
    
    // 更新GUI以显示初始关节值
    updateGUIJointValues();
    
    // 记录初始化完成信息
    ROS_INFO("ROS接口初始化完成");
    logMessage("ROS接口初始化完成");
}

void ArmControlGUI::initializeOpenGL()
{
    // 这里不需要额外的初始化，Scene3DRenderer会自行初始化
    logMessage("3D渲染初始化完成");
}

// 设置相机参数
void ArmControlGUI::setupCameraParameters()
{
    // 设置相机内参矩阵 (根据实际相机标定结果调整)
    camera_intrinsic_.setToIdentity();
    camera_intrinsic_(0, 0) = 500.0; // 焦距x
    camera_intrinsic_(1, 1) = 500.0; // 焦距y
    camera_intrinsic_(0, 2) = 320.0; // 中心点x
    camera_intrinsic_(1, 2) = 240.0; // 中心点y
    camera_intrinsic_(2, 2) = 1.0;
    
    // 初始化相机外参矩阵
    camera_extrinsic_.setToIdentity();
    // 默认相机位置在原点，朝向z轴正方向
    camera_extrinsic_.setColumn(3, QVector4D(0.0f, 0.0f, 0.0f, 1.0f));
    
    logMessage("相机参数已初始化");
}

// 更新3D场景
void ArmControlGUI::updateScene3D()
{
    if (!scene_3d_renderer_) {
        return;
    }
        
        // 更新场景中的物体
        updateSceneObjects();
    
    // 更新机械臂姿态
    scene_3d_renderer_->setRobotPose(current_joint_values_);
    
    // 更新选中的物体
    scene_3d_renderer_->setSelectedObject(selected_object_index_);
}

// 更新场景中的物体
void ArmControlGUI::updateSceneObjects()
{
    // 清空场景中的物体
    scene_objects_.clear();
    
    // 场景中添加原点（坐标系中心）
    scene_objects_.push_back(std::make_pair(QVector3D(0, 0, 0), QColor(255, 255, 255)));
    
    // 计算机械臂末端位置
    geometry_msgs::Pose end_effector_pose = jointsToPos(current_joint_values_);
    
    // 将末端位置添加到场景
    scene_objects_.push_back(std::make_pair(
        QVector3D(end_effector_pose.position.x * 100, // 米转厘米
                 end_effector_pose.position.y * 100,
                 end_effector_pose.position.z * 100),
        QColor(0, 255, 0)));
    
    // 添加检测到的物体
    for (const DetectedObject& obj : detected_objects_) {
        // 确定物体的颜色
        QColor color(255, 0, 0); // 默认红色
        
        // 如果是水果，使用不同颜色
        if (obj.type == "apple") {
            color = QColor(255, 0, 0); // 红色
        } else if (obj.type == "banana") {
                color = QColor(255, 255, 0); // 黄色
        } else if (obj.type == "orange") {
            color = QColor(255, 165, 0); // 橙色
        } else if (obj.type == "tomato") {
            color = QColor(255, 99, 71); // 番茄红
        }
        
        // 将相机坐标系中的点转换到世界坐标系
        QVector4D position_camera(obj.x / 100.0f, obj.y / 100.0f, obj.z / 100.0f, 1.0f); // 厘米转米
        QVector4D position_world = camera_extrinsic_.inverted() * position_camera;
        
        // 添加到场景中
        scene_objects_.push_back(std::make_pair(
            QVector3D(position_world.x() * 100, // 米转厘米
                     position_world.y() * 100,
                     position_world.z() * 100),
            color));
    }
    
    // 更新3D渲染器
    if (scene_3d_renderer_) {
    scene_3d_renderer_->updateObjects(scene_objects_);
    }
}

// 新增函数：更新相机变换矩阵
void ArmControlGUI::updateCameraTransform(const geometry_msgs::Pose& end_effector_pose)
{
    // 创建从世界坐标系到机械臂末端的变换矩阵
    QMatrix4x4 world_to_end_effector;
    
    // 设置旋转部分（使用四元数）
    QQuaternion rotation(end_effector_pose.orientation.w,
        end_effector_pose.orientation.x,
        end_effector_pose.orientation.y,
                          end_effector_pose.orientation.z);
    world_to_end_effector.rotate(rotation);
    
    // 设置平移部分
    world_to_end_effector.setColumn(3, QVector4D(end_effector_pose.position.x,
                                               end_effector_pose.position.y,
                                               end_effector_pose.position.z, 1.0f));
    
    // 定义从末端到相机的变换矩阵（根据实际安装位置调整）
    QMatrix4x4 end_effector_to_camera;
    end_effector_to_camera.setToIdentity();
    // 假设相机安装在末端前方，偏移一定距离
    end_effector_to_camera.translate(0.0f, 0.0f, 0.1f); // 10cm前方
    
    // 计算相机的世界坐标系位置
    camera_extrinsic_ = world_to_end_effector * end_effector_to_camera;
}

// 从图像点获取3D位置
QVector3D ArmControlGUI::imagePointTo3D(const QPoint& image_point, float depth)
{
    // 将图像坐标转换为归一化坐标
    double x = (image_point.x() - camera_intrinsic_(0, 2)) / camera_intrinsic_(0, 0);
    double y = (image_point.y() - camera_intrinsic_(1, 2)) / camera_intrinsic_(1, 1);
    
    // 计算3D点在相机坐标系中的位置
    QVector3D point_camera(x * depth, y * depth, depth);
    
    // 转换到世界坐标系，考虑相机在机械臂末端的位置
    QVector4D point_world = camera_extrinsic_.inverted() * QVector4D(point_camera, 1.0f);
    
    return QVector3D(point_world.x(), point_world.y(), point_world.z());
}

// 从3D位置计算图像点
QPoint ArmControlGUI::point3DToImage(const QVector3D& point_3d)
{
    // 转换到相机坐标系，考虑相机在机械臂末端的位置
    QVector4D point_world(point_3d.x(), point_3d.y(), point_3d.z(), 1.0f);
    QVector4D point_camera = camera_extrinsic_ * point_world;
    
    // 忽略深度太小的点（在相机后面）
    if (point_camera.z() <= 0.01f) {
        return QPoint(-1, -1); // 无效点
    }
    
    // 计算图像坐标
    double x = (point_camera.x() / point_camera.z()) * camera_intrinsic_(0, 0) + camera_intrinsic_(0, 2);
    double y = (point_camera.y() / point_camera.z()) * camera_intrinsic_(1, 1) + camera_intrinsic_(1, 2);
    
    return QPoint(static_cast<int>(x), static_cast<int>(y));
}

// 从深度图获取深度值
float ArmControlGUI::getDepthAtPoint(const QPoint& image_point)
{
    // 这里应该从实际的深度图中获取深度值
    // 目前没有深度图，所以返回一个默认值
    return 1.0f;
}

// 处理相机视图点击
void ArmControlGUI::onCameraViewClicked(QPoint pos)
{
    if (current_camera_image_.isNull() || detected_objects_.empty()) {
        return;
    }
    
    // 调整点击位置到原始图像尺寸
    QLabel* camera_view = ui->cameraView;
    QSize scaled_size;
    if (camera_view->pixmap() != nullptr) {
        scaled_size = camera_view->pixmap()->size();
    } else {
        scaled_size = camera_view->size();
    }
    QSize original_size = current_camera_image_.size();
    
    // 计算缩放比例
    double x_ratio = static_cast<double>(original_size.width()) / scaled_size.width();
    double y_ratio = static_cast<double>(original_size.height()) / scaled_size.height();
    
    // 调整点击位置
    int x = static_cast<int>(pos.x() * x_ratio);
    int y = static_cast<int>(pos.y() * y_ratio);
    
    // 查找最近的物体 - 使用屏幕空间距离
    int closest_object = -1;
    double min_distance = 100.0; // 阈值，单位：像素
    
    for (size_t i = 0; i < detected_objects_.size(); ++i) {
        const DetectedObject& obj = detected_objects_[i];
        
        // 使用物体在相机坐标系中的原始位置
        QVector3D obj_camera(obj.x / 100.0f, obj.y / 100.0f, obj.z / 100.0f);
        
        // 检查深度是否有效
        if (obj_camera.z() <= 0.0f) {
            continue; // 忽略无效深度的物体
        }
        
        // 将3D位置投影到图像平面
        QPoint obj_image = QPoint(
            static_cast<int>(obj_camera.x() / obj_camera.z() * camera_intrinsic_(0, 0) + camera_intrinsic_(0, 2)),
            static_cast<int>(obj_camera.y() / obj_camera.z() * camera_intrinsic_(1, 1) + camera_intrinsic_(1, 2))
        );
        
        // 计算距离
        double distance = std::sqrt(std::pow(x - obj_image.x(), 2) + std::pow(y - obj_image.y(), 2));
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_object = i;
        }
    }
    
    // 如果找到最近的物体，则选中它
    if (closest_object >= 0) {
        selected_object_index_ = closest_object;
        scene_3d_renderer_->setSelectedObject(closest_object);
        
        // 更新检测表格的选择
        if (ui->detectionsTable->rowCount() > closest_object) {
            ui->detectionsTable->selectRow(closest_object);
        }
        
        // 日志
        const DetectedObject& obj = detected_objects_[closest_object];
        logMessage(QString("选中物体: %1 (%2, %3, %4)").arg(
            QString::fromStdString(obj.id)).arg(obj.x).arg(obj.y).arg(obj.z));
    }
}

// 处理3D视图中物体选择
void ArmControlGUI::on3DViewObjectSelected(int index)
{
    selected_object_index_ = index;
    
    // 更新检测表格的选择
    if (index >= 0 && ui->detectionsTable->rowCount() > index) {
        ui->detectionsTable->selectRow(index);
        
        // 日志
        const DetectedObject& obj = detected_objects_[index];
        logMessage(QString("3D视图中选中物体: %1 (%2, %3, %4)").arg(
            QString::fromStdString(obj.id)).arg(obj.x).arg(obj.y).arg(obj.z));
    }
}

// 发送物体拾取命令
void ArmControlGUI::sendPickObjectCommand(int object_index)
{
    if (object_index >= 0 && object_index < static_cast<int>(detected_objects_.size())) {
        const DetectedObject& obj = detected_objects_[object_index];
        sendPickCommand(obj.id);
        logMessage(QString("发送拾取命令: %1").arg(QString::fromStdString(obj.id)));
    }
}

// 关节控制槽实现
void ArmControlGUI::onJoint1SliderChanged(int value)
{
    ui->joint1_spin->setValue(value);
    current_joint_values_[0] = value * M_PI / 180.0; // 转换为弧度
    
    // 参照示例动作实现，使用底盘电机实现旋转
    // 底座旋转对应舵机ID 1，位置范围映射到舵机范围
    // 增大运动范围，使控制更加明显
    int servo_position = map(value, -180, 180, 400, 2600);
    
    // 直接使用电机控制命令而不是舵机命令
    int motor_pos = map(value, -180, 180, -50, 50); // 映射到电机位置范围
    sendMotorOrder(1, 11, 0, 0, 0, true, motor_pos, 30);
    QApplication::processEvents();
    
    // 仍然发送舵机命令以保持兼容性
    sendServoCommand(1, servo_position);
    
    // 发送关节命令
    sendJointCommand(current_joint_values_);
    
    logMessage(QString("关节1(底座)调整到: %1度").arg(value));
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
    
    // 参照示例动作实现，结合电机控制和舵机控制
    // 肩部关节对应舵机ID 2，位置范围映射到舵机范围
    // 增大运动范围，使控制更加明显
    int servo_position = map(value, -90, 90, 400, 2600);
    
    // 使用进给电机控制机械臂肩部，参考示例动作2的实现
    // 根据角度值映射到进给电机位置
    int motor_pos = map(value, -90, 90, -1500, 1500);
    
    // 使用示例动作中的电机控制序列，先准备电机
    sendMotorOrder(2, 100, 100, 0, 0, false, motor_pos, 10);
    QApplication::processEvents();
    ros::Duration(0.1).sleep();
    
    // 设置位置模式
    sendMotorOrder(2, 0, 100, 0, 0, true, motor_pos, 10);
    QApplication::processEvents();
    ros::Duration(0.1).sleep();
    
    // 执行移动
    sendMotorOrder(2, 99, 100, 0, 0, false, motor_pos, 10);
    QApplication::processEvents();
    
    // 仍然发送舵机命令以保持兼容性
    sendServoCommand(2, servo_position);
    
    // 发送关节命令
    sendJointCommand(current_joint_values_);
    
    logMessage(QString("关节3(肩部)调整到: %1度").arg(value));
}

void ArmControlGUI::onJoint4SliderChanged(int value)
{
    ui->joint4_spin->setValue(value);
    current_joint_values_[3] = value * M_PI / 180.0; // 转换为弧度
    
    // 参照示例动作实现，结合电机控制和舵机控制
    // 肘部关节对应舵机ID 3，位置范围映射到舵机范围
    // 增大运动范围，使控制更加明显
    int servo_position = map(value, 0, 180, 400, 2600);
    
    // 参考示例动作1中的夹持电机控制
    // 根据角度值映射到夹持电机位置
    int motor_pos = map(value, 0, 180, -25, 25);
    
    // 使用示例动作中的夹持电机控制方式
    sendMotorOrder(1, 11, 0, 0, 0, true, motor_pos, 30);
    QApplication::processEvents();
    
    // 仍然发送舵机命令以保持兼容性
    sendServoCommand(3, servo_position);
    
    // 发送关节命令
    sendJointCommand(current_joint_values_);
    
    logMessage(QString("关节4(肘部)调整到: %1度").arg(value));
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
    // 获取XYZ坐标值（厘米单位）
    // 尝试查找控件，如果不存在则使用默认值
    double x = 30.0; // 默认值 30cm
    double y = 0.0;  // 默认值 0cm
    double z = 20.0; // 默认值 20cm
    
    QDoubleSpinBox* xSpin = ui->centralwidget->findChild<QDoubleSpinBox*>("x_spin");
    QDoubleSpinBox* ySpin = ui->centralwidget->findChild<QDoubleSpinBox*>("y_spin");
    QDoubleSpinBox* zSpin = ui->centralwidget->findChild<QDoubleSpinBox*>("z_spin");
    
    if (xSpin) x = xSpin->value();
    if (ySpin) y = ySpin->value();
    if (zSpin) z = zSpin->value();
    
    // 记录日志
    logMessage(QString("移动到笛卡尔坐标位置: X=%1, Y=%2, Z=%3 cm").arg(x).arg(y).arg(z));
    
    // 创建位置目标消息（米单位）
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x / 100.0;  // 厘米转换为米
    target_pose.position.y = y / 100.0;  // 厘米转换为米
    target_pose.position.z = z / 100.0;  // 厘米转换为米
    
    // 设置朝向（默认保持垂直向下）
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;
    
    // 在发送ROS命令前，先使用电机控制进行简单的位置移动
    // 这使得操控效果更加明显
    
    // 1. 计算底座旋转角度 (围绕Z轴)
    double theta1 = atan2(y, x); // 弧度
    int base_angle = static_cast<int>(theta1 * 180.0 / M_PI); // 度
    
    // 2. 计算肩部角度 (根据高度)
    double distance_xy = sqrt(x*x + y*y);
    double theta3 = atan2(z - 20.0, distance_xy); // 假设基础高度为20cm
    int shoulder_angle = static_cast<int>(theta3 * 180.0 / M_PI);
    
    // 3. 限制在有效范围内
    base_angle = std::max(-180, std::min(base_angle, 180));
    shoulder_angle = std::max(-90, std::min(shoulder_angle, 90));
    
    // 4. 控制底座电机
    int motor_pos_base = map(base_angle, -180, 180, -50, 50);
    sendMotorOrder(1, 11, 0, 0, 0, true, motor_pos_base, 30);
    QApplication::processEvents();
    
    // 5. 控制肩部进给电机
    int motor_pos_shoulder = map(shoulder_angle, -90, 90, -1500, 1500);
    sendMotorOrder(2, 100, 100, 0, 0, false, motor_pos_shoulder, 10);
    QApplication::processEvents();
    sendMotorOrder(2, 0, 100, 0, 0, true, motor_pos_shoulder, 10);
    QApplication::processEvents();
    sendMotorOrder(2, 99, 100, 0, 0, false, motor_pos_shoulder, 10);
    QApplication::processEvents();
    
    // 发布目标位置消息
    std_msgs::String cmd_msg;
    cmd_msg.data = "move_to " + std::to_string(x/100.0) + " " + 
                   std::to_string(y/100.0) + " " + std::to_string(z/100.0);
    arm_command_pub_.publish(cmd_msg);
    
    // 更新当前控制模式
    current_control_mode_ = ArmControlMode::CARTESIAN_CONTROL;
    updateUI();
}

void ArmControlGUI::onHomeButtonClicked()
{
    logMessage("移动到初始位置");
    
    // 参照示例动作实现，使用进给电机控制回到初始位置
    // 执行示例动作中的进给电机归零序列
    sendMotorOrder(2, 100, 200, 0, 0, false, 2000, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    
    sendMotorOrder(2, 0, 200, 0, 0, true, 300, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    
    sendMotorOrder(2, 99, 200, 0, 0, false, 2000, 10);
    QApplication::processEvents();
    ros::Duration(1.0).sleep();
    
    sendMotorOrder(2, 100, 100, 0, 0, false, 2000, 10);
    QApplication::processEvents();
    
    // 同时发送标准的回到初始位置命令以保持兼容性
    sendHomeCommand();
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
    // 发送吸盘开启命令
    sendVacuumCommand(true, vacuum_power_);
    vacuum_on_ = true;
    
    // 更新UI
    updateVacuumStatus();
    
    // 记录操作
    logMessage(QString("吸盘已开启，功率: %1%").arg(vacuum_power_));
    
    // 发送继电器命令
    sendRelayOrder("11");
}

void ArmControlGUI::onVacuumOffButtonClicked()
{
    // 发送吸盘关闭命令
    sendVacuumCommand(false);
    vacuum_on_ = false;
    
    // 更新UI
    updateVacuumStatus();
    
    // 记录操作
    logMessage("吸盘已关闭");
    
    // 发送继电器命令
    sendRelayOrder("00");
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
    if (row >= 0 && row < static_cast<int>(detected_objects_.size())) {
        // 更新选中的物体索引
        selected_object_index_ = row;
        
        // 更新3D视图中的选择
        if (scene_3d_renderer_) {
            scene_3d_renderer_->setSelectedObject(row);
        }
        
        // 日志
        const DetectedObject& obj = detected_objects_[row];
        logMessage(QString("选中表格中的物体: %1 (%2, %3, %4)").arg(
            QString::fromStdString(obj.id)).arg(obj.x).arg(obj.y).arg(obj.z));
        
        // 如果点击的是操作列，不需要进一步处理
        if (column == 5) {
            return;
        }
    }
}

// 定时器槽实现
void ArmControlGUI::onUpdateGUI()
{
    // 更新关节信息
    updateJointInfo();
    
    // 更新末端执行器位姿
    updateEndEffectorPose();
    
    // 更新摄像头视图
    updateCameraViews();
    
    // 更新连接状态
    updateConnectionStatus();
}

// ROS回调函数实现
void ArmControlGUI::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // 更新当前关节值
    if (msg->position.size() >= 6) {
        for (size_t i = 0; i < 6 && i < msg->position.size(); ++i) {
                current_joint_values_[i] = msg->position[i];
            }
        
        // 记录接收到的关节状态 - 但只在值变化时记录，避免刷屏
        static std::vector<double> last_joint_values;
        bool has_changed = false;
        
        if (last_joint_values.size() != current_joint_values_.size()) {
            has_changed = true;
            last_joint_values = current_joint_values_;
        } else {
            for (size_t i = 0; i < current_joint_values_.size(); ++i) {
                if (std::fabs(current_joint_values_[i] - last_joint_values[i]) > 0.01) {
                    has_changed = true;
                    last_joint_values[i] = current_joint_values_[i];
                }
            }
        }
        
        if (has_changed) {
            QString logStr = "关节状态更新: ";
            for (size_t i = 0; i < current_joint_values_.size(); ++i) {
                logStr += QString("J%1=%2 ").arg(i+1).arg(current_joint_values_[i], 0, 'f', 2);
            }
            
            // 显示在状态栏而不是日志区域，避免占用太多空间
            ui->statusbar->showMessage(logStr, 3000);
        }
        
        // 更新3D视图中的机器人姿态
        if (scene_3d_renderer_) {
            scene_3d_renderer_->setRobotPose(current_joint_values_);
        }
        
        // 更新控制面板上的滑块和SpinBox
        QMetaObject::invokeMethod(this, "updateJointControlWidgets", Qt::QueuedConnection);
        QMetaObject::invokeMethod(this, "updateEndEffectorPose", Qt::QueuedConnection);
    }
    
    // 检查是否有附加数据（如传感器反馈、电机状态等）
    if (!msg->name.empty() && !msg->effort.empty() && msg->name.size() == msg->effort.size()) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& name = msg->name[i];
            const double effort = msg->effort[i];
            
            // 处理特殊传感器数据，如力传感器、电机电流等
            if (name.find("force_sensor") != std::string::npos) {
                // 显示力传感器数据
                ui->statusbar->showMessage(QString("力传感器: %1 N").arg(effort, 0, 'f', 2), 3000);
            } 
            else if (name.find("motor_current") != std::string::npos) {
                // 显示电机电流
                static QLabel* currentLabel = nullptr;
                if (!currentLabel) {
                    // 创建电流显示标签
                    currentLabel = new QLabel(this);
                    ui->statusbar->addPermanentWidget(currentLabel);
                }
                currentLabel->setText(QString("电机电流: %1 A").arg(effort, 0, 'f', 2));
            }
        }
    }
}

void ArmControlGUI::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 将ROS图像转换为OpenCV图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat detection_image = cv_ptr->image.clone();
        
        // 记录收到的图像信息
        ROS_INFO("收到检测图像: 尺寸=%dx%d, 通道=%d", 
                detection_image.cols, detection_image.rows, detection_image.channels());
        
        // 转换为QImage
        QImage image;
        if (detection_image.channels() == 3) {
            cv::cvtColor(detection_image, detection_image, cv::COLOR_BGR2RGB);
            image = QImage(detection_image.data, detection_image.cols, detection_image.rows,
                          detection_image.step, QImage::Format_RGB888);
        } else {
            image = QImage(detection_image.data, detection_image.cols, detection_image.rows,
                          detection_image.step, QImage::Format_Grayscale8);
        }
        
        // 保存检测结果图像
        current_camera_image_ = image.copy();
        
        // 在图像上添加检测物体的标记
        if (!detected_objects_.empty()) {
            QPainter painter(&current_camera_image_);
            painter.setPen(QPen(Qt::green, 2));
            painter.setFont(QFont("Arial", 12));
            
            for (const auto& obj : detected_objects_) {
                // 将3D位置转换为图像坐标
                double img_x = (obj.x / 100.0 + 0.5) * current_camera_image_.width();  // 厘米转米再转像素
                double img_y = (0.5 - obj.y / 100.0) * current_camera_image_.height(); // 厘米转米再转像素
                
                // 确保坐标在图像范围内
                if (img_x >= 0 && img_x < current_camera_image_.width() &&
                    img_y >= 0 && img_y < current_camera_image_.height()) {
                    
                    // 绘制圆圈标记物体位置
                    painter.drawEllipse(QPointF(img_x, img_y), 10, 10);
                    
                    // 添加文本标签
                    painter.drawText(QPointF(img_x + 15, img_y), QString::fromStdString(obj.id));
                }
            }
            
            ROS_INFO("在检测图像上标记了 %zu 个物体", detected_objects_.size());
        }
        
        // 更新UI
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
        
        // 确保检测表格也被更新
        QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
        
        // 记录日志
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 30 == 0) {  // 每30帧记录一次
            ROS_INFO("已接收检测图像: 帧 %d, 尺寸 %dx%d", 
                    frame_count, image.width(), image.height());
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("检测图像回调中的cv_bridge异常: %s", e.what());
    } catch (std::exception& e) {
        ROS_ERROR("检测图像回调中的标准异常: %s", e.what());
    }
}

void ArmControlGUI::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // 记录收到消息的信息
    ROS_INFO("收到检测位姿消息: %zu 个物体, frame_id=%s", 
            msg->poses.size(), msg->header.frame_id.c_str());
    
    // 没有检测到物体，清空表格并返回
    if (msg->poses.empty()) {
        ROS_INFO("检测位姿为空，清空表格");
        QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
        return;
    }
    
    // 清空之前的检测结果
    detected_objects_.clear();
    
    // 检查是否有附加信息（通常应该包含类别名称），当前假设这些信息在frames_id中
    bool has_object_ids = false;
    std::vector<std::string> class_names;
    
    if (!msg->header.frame_id.empty()) {
        // 尝试解析frame_id中可能包含的类别信息
        try {
            // 假设frame_id格式是 "class1,class2,class3,..."
            std::string frame_id = msg->header.frame_id;
            std::istringstream ss(frame_id);
            std::string token;
            
            while (std::getline(ss, token, ',')) {
                if (!token.empty()) {
                    class_names.push_back(token);
                }
            }
            
            has_object_ids = !class_names.empty();
            
            if (has_object_ids) {
                ROS_INFO("成功解析类别信息: %zu 个类别", class_names.size());
            }
        } catch (const std::exception& e) {
            ROS_WARN("解析物体类别信息失败: %s", e.what());
            has_object_ids = false;
        }
    }
    
    // 处理所有物体
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        DetectedObject obj;
        
        // 设置物体ID和类型
        if (has_object_ids && i < class_names.size()) {
            obj.id = class_names[i];
            obj.type = class_names[i]; // 使用类别名作为类型
        } else {
            obj.id = "object_" + std::to_string(i);
            
            // 根据Z轴高度猜测物体类型
            float height = msg->poses[i].position.z;
            if (height < 0.05) {
                obj.type = "低物体";
            } else if (height < 0.15) {
                obj.type = "中物体";
            } else {
                obj.type = "高物体";
            }
        }
        
        // 将3D位置转换为厘米单位用于显示
        double scale = 100.0; // 转换为厘米
        obj.x = msg->poses[i].position.x * scale;
        obj.y = msg->poses[i].position.y * scale;
        obj.z = msg->poses[i].position.z * scale;
        
        // 保存原始位姿
        obj.pose = msg->poses[i];
        detected_objects_.push_back(obj);
        
        // 记录成功添加的物体
        ROS_INFO("添加检测物体到表格: %s [%.2f, %.2f, %.2f]", 
                obj.id.c_str(), obj.x, obj.y, obj.z);
    }
    
    ROS_INFO("检测到 %zu 个物体，准备更新UI", detected_objects_.size());
    
    // 使用单一调用更新UI，避免重复更新
    QMetaObject::invokeMethod(this, "updateUI", Qt::QueuedConnection);
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
    if (current_joint_values_.size() >= 6) {
        // 使用正向运动学计算末端执行器位置
        geometry_msgs::Pose pose = jointsToPos(current_joint_values_);
        
        // 保存当前位置为3D向量和四元数
        current_end_position_ = QVector3D(
            pose.position.x,
            pose.position.y,
            pose.position.z
        );
        
        current_end_orientation_ = QQuaternion(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        );
        
        // 更新相机变换矩阵
        updateCameraTransform(pose);
        
        // 记录位置信息到日志（不更新UI）
        logMessage(QString("末端位置: X=%1, Y=%2, Z=%3 cm").arg(
            pose.position.x * 100.0, 0, 'f', 1).arg(
            pose.position.y * 100.0, 0, 'f', 1).arg(
            pose.position.z * 100.0, 0, 'f', 1));
    }
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
    // 如果没有图像，则跳过
    if (current_camera_image_.isNull()) {
        return;
    }
    
    // 获取当前相机视图组件的大小
    QSize viewSize = ui->cameraView->size();
    
    // 缩放图像以适应视图
    QImage scaledImage = current_camera_image_.scaled(viewSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    
    // 创建一个带有标记的副本
    QImage displayImage = scaledImage.copy();
    QPainter painter(&displayImage);
    
    // 添加检测到的物体标记
    for (size_t i = 0; i < detected_objects_.size(); i++) {
        // 将3D世界坐标转为图像坐标
        QVector3D objPos(detected_objects_[i].x, detected_objects_[i].y, detected_objects_[i].z);
        QPoint imgPos = point3DToImage(objPos);
        
        // 调整为当前显示比例
        float scaleX = static_cast<float>(scaledImage.width()) / current_camera_image_.width();
        float scaleY = static_cast<float>(scaledImage.height()) / current_camera_image_.height();
        imgPos.setX(imgPos.x() * scaleX);
        imgPos.setY(imgPos.y() * scaleY);
        
        // 设置绘制样式
        if (static_cast<int>(i) == selected_object_index_) {
            painter.setPen(QPen(Qt::red, 3));
        } else {
            painter.setPen(QPen(Qt::green, 2));
        }
        
        // 绘制标记
        painter.drawEllipse(imgPos, 15, 15);
        
        // 添加标签
        painter.setFont(QFont("Arial", 10, QFont::Bold));
        painter.drawText(imgPos.x() + 20, imgPos.y() + 5, 
                       QString("%1: %2").arg(detected_objects_[i].id.c_str())
                                       .arg(detected_objects_[i].type.c_str()));
    }
    
    // 设置图像到相机视图
    ui->cameraView->setPixmap(QPixmap::fromImage(displayImage));
    
    // 立即处理事件，避免UI卡顿和闪烁
    QApplication::processEvents();
    
    // 如果有检测标签和视图，更新它们
    QLabel* detectionStatusLabel = ui->centralwidget->findChild<QLabel*>("detectionStatusLabel");
    QLabel* detectionView = ui->centralwidget->findChild<QLabel*>("detectionView");
    
    if (detectionStatusLabel && detectionView) {
        // 更新状态文本
        QString status = QString("检测到 %1 个物体").arg(detected_objects_.size());
        detectionStatusLabel->setText(status);
        
        // 如果有检测图像，显示它
        if (!current_camera_image_.isNull()) {
            detectionView->setPixmap(QPixmap::fromImage(current_camera_image_.scaled(
                detectionView->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)));
        }
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
    
    // 计算末端执行器位置（简化版本，实际应使用完整的正向运动学）
    // 这样可以在日志中显示关节控制对应的笛卡尔空间位置
    double x = 0.0, y = 0.0, z = 0.0;
    
    // 简化的位置计算
    double theta1 = joint_values[0]; // 底座旋转角度
    double d2 = joint_values[1];     // 伸缩关节长度
    double theta3 = joint_values[2]; // 肩部关节角度
    double theta4 = joint_values[3]; // 肘部关节角度
    double d6 = joint_values[5];     // 末端伸缩长度
    
    // 非常简化的正向运动学计算
    double arm_length = 30.0 + d2;      // 基础长度 + 伸缩值
    double reach_distance = arm_length * cos(theta3) * cos(theta4); // 水平伸展距离
    
    x = reach_distance * cos(theta1);   // X 坐标
    y = reach_distance * sin(theta1);   // Y 坐标
    z = 20.0 + arm_length * sin(theta3); // Z 坐标，假设基础高度为20cm
    
    // 转换为厘米
    x *= 100.0;
    y *= 100.0;
    z *= 100.0;
    
    // 仅在值明显变化时更新日志（避免刷屏）
    static double last_x = 0.0, last_y = 0.0, last_z = 0.0;
    if (fabs(x - last_x) > 1.0 || fabs(y - last_y) > 1.0 || fabs(z - last_z) > 1.0) {
        logMessage(QString("关节控制：末端位置约为 X=%1, Y=%2, Z=%3 cm").arg(
            QString::number(x, 'f', 1)).arg(
            QString::number(y, 'f', 1)).arg(
            QString::number(z, 'f', 1)));
        
        last_x = x;
        last_y = y;
        last_z = z;
    }
}

void ArmControlGUI::sendVacuumCommand(bool on, int power)
{
    // 创建吸附命令消息
    std_msgs::Bool vacuum_cmd;
    vacuum_cmd.data = on;
    
    // 创建功率设置消息
    std_msgs::Float64 power_cmd;
    power_cmd.data = power / 100.0; // 转换为0-1范围
    
    // 增大功率值，使控制效果更明显
    power_cmd.data = power_cmd.data * 1.5; // 增加50%的功率值
    if (power_cmd.data > 1.0) power_cmd.data = 1.0; // 确保不超过最大值
    
    // 发布吸附命令
    vacuum_command_pub_.publish(vacuum_cmd);
    
    // 更新状态
    vacuum_on_ = on;
    vacuum_power_ = power;
}

void ArmControlGUI::sendPickCommand(const std::string& object_id)
{
    // 在实际拾取物体前，首先查找该物体是否在检测列表中
    bool found = false;
    DetectedObject target_obj;
    
    for (const auto& obj : detected_objects_) {
        if (obj.id == object_id) {
            target_obj = obj;
            found = true;
            break;
        }
    }
    
    // 如果找到了物体，使用统一的坐标系和尺度
    if (found) {
        // 创建统一的命令，确保与其他控制模式使用相同的单位和尺度
        std_msgs::String cmd_msg;
        
        // 从物体位置和朝向构建命令，统一使用米为单位
        // 确保与三维坐标控制使用相同的尺度
        double x = target_obj.x / 100.0; // 从cm转换为m
        double y = target_obj.y / 100.0;
        double z = target_obj.z / 100.0;
        
        cmd_msg.data = "pick arm1 " + object_id + " " + 
                       std::to_string(x) + " " + 
                       std::to_string(y) + " " + 
                       std::to_string(z);
        arm_command_pub_.publish(cmd_msg);
        
        logMessage(QString("拾取物体: %1，坐标: (%2, %3, %4)米").arg(
            QString::fromStdString(object_id)).arg(x).arg(y).arg(z));
    }
    else {
        // 如果没有找到物体，使用基本命令
        std_msgs::String cmd_msg;
        cmd_msg.data = "pick arm1 " + object_id;
        arm_command_pub_.publish(cmd_msg);
        logMessage(QString("拾取未知物体: %1").arg(QString::fromStdString(object_id)));
    }
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
    pose.position.x = 0.3; // 默认在前方30cm
    pose.position.y = 0.0;
    pose.position.z = 0.2; // 默认高度20cm
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
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
    
    // 记录操作
    logMessage(QString("发送继电器命令: %1").arg(command.c_str()));
}

// 修改为简单的状态回调函数
void ArmControlGUI::yoloStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // 更新状态栏信息
    QString status_msg = "检测系统已连接";
    ui->statusbar->showMessage(status_msg, 3000);
    
    // 确保UI反映当前状态
    updateCameraViews();
    updateDetectionsTable();
}

// YOLO检测功能已移除

// 添加路径规划相关的槽函数
void ArmControlGUI::onScanObjectsClicked()
{
    logMessage("开始扫描物体...");
    
    // 发送扫描命令 - 使用前进行物体扫描
    std_msgs::String cmd_msg;
    cmd_msg.data = "scan";
    arm_command_pub_.publish(cmd_msg);
    
    // 显示状态信息
    ui->statusbar->showMessage("正在扫描工作区域内的物体...", 2000);
    logMessage("扫描工作区域内的物体");
}

void ArmControlGUI::onExecutePathClicked()
{
    logMessage("执行规划的路径...");
    
    // 发送执行命令
    std_msgs::String cmd_msg;
    cmd_msg.data = "execute";
    arm_command_pub_.publish(cmd_msg);
    
    // 更新状态栏
    ui->statusbar->showMessage("正在执行路径规划动作...", 3000);
}

void ArmControlGUI::onVisualizeWorkspaceClicked()
{
    logMessage("可视化工作空间...");
    
    // 发送可视化命令
    std_msgs::String cmd_msg;
    cmd_msg.data = "visualize_workspace";
    arm_command_pub_.publish(cmd_msg);
    
    // 更新状态栏
    ui->statusbar->showMessage("正在可视化工作空间...", 3000);
}

// 新增统一的物体检测回调函数，处理YOLO格式的检测结果
void ArmControlGUI::objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg, 
                                         const geometry_msgs::PoseArray::ConstPtr& poses_msg)
{
    try {
        // 处理检测图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        cv::Mat detection_image = cv_ptr->image.clone();
        
        // 记录收到的图像和位姿信息
        ROS_INFO("收到同步检测数据: 图像=%dx%d, 位姿=%zu个", 
                 detection_image.cols, detection_image.rows, poses_msg->poses.size());
        
            // 处理所有检测结果
    ROS_INFO("处理物体检测结果");
    
    // 处理前先清除之前的检测结果
    detected_objects_.clear();
        
        // 检查是否有附加信息（通常应该包含类别名称），当前假设这些信息在frames_id中
        bool has_object_ids = false;
        if (!poses_msg->header.frame_id.empty()) {
            // 尝试解析frame_id中可能包含的类别信息
            try {
                // 假设frame_id格式是 "class1,class2,class3,..."
                std::string frame_id = poses_msg->header.frame_id;
                std::istringstream ss(frame_id);
                std::string token;
                std::vector<std::string> class_names;
                
                while (std::getline(ss, token, ',')) {
                    if (!token.empty()) {
                        class_names.push_back(token);
                    }
                }
                
                has_object_ids = !class_names.empty() && class_names.size() == poses_msg->poses.size();
                
                // 如果成功解析类别信息，直接使用它们
                if (has_object_ids) {
                    ROS_INFO("成功解析类别信息: %zu 个类别", class_names.size());
        for (size_t i = 0; i < poses_msg->poses.size(); ++i) {
            DetectedObject obj;
                        obj.id = class_names[i];  // 使用传递过来的类别名称
                        obj.type = class_names[i];  // 使用类别名称作为物体类型
                        
                        // 将3D位置转换为厘米单位用于显示
                        double scale = 100.0; // 转换为厘米
                        obj.x = poses_msg->poses[i].position.x * scale;
                        obj.y = poses_msg->poses[i].position.y * scale;
                        obj.z = poses_msg->poses[i].position.z * scale;
                        
                        // 保存原始位姿
            obj.pose = poses_msg->poses[i];
            detected_objects_.push_back(obj);
                        
                        // 记录成功添加的物体
                        ROS_INFO("添加检测物体到表格: %s [%.2f, %.2f, %.2f]", 
                                 obj.id.c_str(), obj.x, obj.y, obj.z);
                    }
                }
            } catch (const std::exception& e) {
                ROS_WARN("解析物体类别信息失败: %s", e.what());
                has_object_ids = false;
            }
        }
        
        // 如果无法从frame_id中获取类别信息，使用默认物体命名方式
        if (!has_object_ids) {
            // 使用自定义物品名称而不是object_*
            // 为每个检测分配一个更有意义的名称和类型
            const std::vector<std::string> item_names = {
                "水杯", "鼠标", "键盘", "手机", "书本", 
                "笔", "剪刀", "USB盘", "眼镜", "钥匙", 
                "螺丝刀", "玩具", "橡皮擦", "胶带", "标签"
            };
            
            for (size_t i = 0; i < poses_msg->poses.size(); ++i) {
                DetectedObject obj;
                
                // 根据物体位置特征决定类型名称
                float height = poses_msg->poses[i].position.z;
                
                std::string type_name;
                if (height < 0.05) {
                    type_name = "低矮物品";
                } else if (height < 0.15) {
                    type_name = "中等物品";
                } else {
                    type_name = "高大物品";
                }
                
                // 根据索引分配名称，如果超出范围则使用通用名称
                if (i < item_names.size()) {
                    obj.id = item_names[i];
                } else {
                    obj.id = "物品_" + std::to_string(i+1);
                }
                
                obj.type = type_name;
                
                // 将3D位置转换为厘米单位用于显示
                double scale = 100.0; // 转换为厘米
                obj.x = poses_msg->poses[i].position.x * scale;
                obj.y = poses_msg->poses[i].position.y * scale;
                obj.z = poses_msg->poses[i].position.z * scale;
                
                // 保存原始位姿
                obj.pose = poses_msg->poses[i];
                detected_objects_.push_back(obj);
                
                // 记录成功添加的物体
                ROS_INFO("添加物体到表格: %s [%.2f, %.2f, %.2f]", 
                         obj.id.c_str(), obj.x, obj.y, obj.z);
            }
        }
        
        // 转换为QImage并保存
        QImage detected_image;
        if (detection_image.channels() == 3) {
            cv::cvtColor(detection_image, detection_image, cv::COLOR_BGR2RGB);
            detected_image = QImage(detection_image.data, detection_image.cols, detection_image.rows,
                                   detection_image.step, QImage::Format_RGB888);
        } else {
            detected_image = QImage(detection_image.data, detection_image.cols, detection_image.rows,
                                   detection_image.step, QImage::Format_Grayscale8);
        }
        
        // 保存为当前图像以显示
        current_camera_image_ = detected_image.copy();
        
        // 记录检测到的物体数量
        ROS_INFO("同步检测到 %zu 个物体，准备更新UI", detected_objects_.size());
        
        // 更新UI
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
        QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
        QMetaObject::invokeMethod(this, "updateUI", Qt::QueuedConnection);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("物体检测回调中的cv_bridge异常: %s", e.what());
    } catch (std::exception& e) {
        ROS_ERROR("物体检测回调中的标准异常: %s", e.what());
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
    
    // 更新状态栏
    QString status;
    if (ros_ok) {
        status += "ROS: 已连接 | ";
    } else {
        status += "ROS: 未连接 | ";
    }
    
    if (camera_ok) {
        status += "相机: 已连接";
    } else {
        status += "相机: 未连接";
    }
    
    // 设置状态栏文本
    statusBar()->showMessage(status);
}

// 更新检测结果表格
void ArmControlGUI::updateDetectionsTable()
{
    // 阻止表格更新信号，避免不必要的刷新
    ui->detectionsTable->blockSignals(true);
    
    // 记录当前选中行
    int currentRow = ui->detectionsTable->currentRow();
    
    // 保存现有按钮，避免重复创建
    QMap<int, QPushButton*> existingButtons;
    for (int i = 0; i < ui->detectionsTable->rowCount(); i++) {
        QWidget* widget = ui->detectionsTable->cellWidget(i, 5);
        if (QPushButton* btn = qobject_cast<QPushButton*>(widget)) {
            existingButtons[i] = btn;
            // 断开之前的连接，避免信号重复
            disconnect(btn, nullptr, this, nullptr);
        }
    }
    
    // 清空表格
    ui->detectionsTable->setRowCount(0);
    
    // 如果没有检测到物体，直接返回
    if (detected_objects_.empty()) {
        ui->detectionsTable->blockSignals(false);
        return;
    }
    
    // 设置表格行数
    ui->detectionsTable->setRowCount(detected_objects_.size());
    
    // 填充表格
    for (size_t i = 0; i < detected_objects_.size(); ++i) {
        const auto& obj = detected_objects_[i];
        
        // 设置ID
        QTableWidgetItem* idItem = new QTableWidgetItem(QString::fromStdString(obj.id));
        idItem->setFlags(idItem->flags() & ~Qt::ItemIsEditable);
        ui->detectionsTable->setItem(i, 0, idItem);
        
        // 设置类型
        QTableWidgetItem* typeItem = new QTableWidgetItem(QString::fromStdString(obj.type));
        typeItem->setFlags(typeItem->flags() & ~Qt::ItemIsEditable);
        ui->detectionsTable->setItem(i, 1, typeItem);
        
        // 设置X坐标
        QTableWidgetItem* xItem = new QTableWidgetItem(QString::number(obj.x, 'f', 2));
        xItem->setFlags(xItem->flags() & ~Qt::ItemIsEditable);
        ui->detectionsTable->setItem(i, 2, xItem);
        
        // 设置Y坐标
        QTableWidgetItem* yItem = new QTableWidgetItem(QString::number(obj.y, 'f', 2));
        yItem->setFlags(yItem->flags() & ~Qt::ItemIsEditable);
        ui->detectionsTable->setItem(i, 3, yItem);
        
        // 设置Z坐标
        QTableWidgetItem* zItem = new QTableWidgetItem(QString::number(obj.z, 'f', 2));
        zItem->setFlags(zItem->flags() & ~Qt::ItemIsEditable);
        ui->detectionsTable->setItem(i, 4, zItem);
        
        // 添加或复用拾取按钮
        QPushButton* pickButton;
        if (existingButtons.contains(i)) {
            // 复用现有按钮
            pickButton = existingButtons[i];
        } else {
            // 创建新按钮
            pickButton = new QPushButton("拾取");
        }
        
        // 设置按钮属性
        pickButton->setProperty("object_index", static_cast<int>(i));
        
        // 连接点击事件，使用lambda捕获当前索引i
        connect(pickButton, &QPushButton::clicked, [this, i]() {
            sendPickObjectCommand(i);
        });
        
        // 设置按钮到单元格
        ui->detectionsTable->setCellWidget(i, 5, pickButton);
    }
    
    // 恢复之前选中的行
    if (currentRow >= 0 && currentRow < ui->detectionsTable->rowCount()) {
        ui->detectionsTable->selectRow(currentRow);
    }
    
    // 立即处理事件，避免UI卡顿和闪烁
    QApplication::processEvents();
    
    // 恢复表格信号
    ui->detectionsTable->blockSignals(false);
}



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
    logMessage("打开夹持器");
    
    // 参照示例动作1中的夹持电机实现
    // 打开夹持器对应夹持电机归零
    sendMotorOrder(1, 11, 0, 0, 0, true, 0, 30);
    QApplication::processEvents();
    
    // 同时发送标准的夹持器命令以保持兼容性
    sendGripperCommand(true);
}

void ArmControlGUI::on_gripperCloseButton_clicked()
{
    logMessage("关闭夹持器");
    
    // 参照示例动作1中的夹持电机实现
    // 关闭夹持器对应夹持电机移动到-25位置
    sendMotorOrder(1, 11, 0, 0, 0, true, -25, 30);
    QApplication::processEvents();
    
    // 同时发送标准的夹持器命令以保持兼容性
    sendGripperCommand(false);
}

void ArmControlGUI::on_vacuumOnButton_clicked()
{
    onVacuumOnButtonClicked();
}

void ArmControlGUI::on_vacuumOffButton_clicked()
{
    onVacuumOffButtonClicked();
}

// 更新UI
void ArmControlGUI::updateUI()
{
    // 更新检测表格
    updateDetectionsTable();
    
    // 更新相机视图
    updateCameraViews();
    
    // 更新关节状态
    updateJointInfo();
    
    // 更新末端执行器位姿
    updateEndEffectorPose();
}

void ArmControlGUI::sendGripperCommand(bool open)
{
    // 创建夹持器命令消息
    std_msgs::Bool gripper_cmd;
    gripper_cmd.data = open;
    
    // 发布夹持器命令
    gripper_cmd_pub_.publish(gripper_cmd);
    
    // 立即处理事件，避免UI卡顿
    QApplication::processEvents();
    
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
        cv::Mat camera_image = cv_ptr->image.clone();
        
        // 处理双目图像 - 1280*480的图像分为左右两部分
        cv::Mat single_view;
        
        if (camera_image.cols == 1280 && camera_image.rows == 480) {
            // 这是一个双目图像，提取左图像作为主视图
            cv::Rect left_roi(0, 0, 640, 480);  // 左半部分
            cv::Rect right_roi(640, 0, 640, 480);  // 右半部分
            
            cv::Mat left_view = camera_image(left_roi);
            // 创建合成视图 - 这里简单地使用左视图
            single_view = left_view.clone();
            
        } else {
            // 其他尺寸的图像，直接使用
            single_view = camera_image;
        }
        
        // 记录成功接收的图像帧
        static int frame_count = 0;
        frame_count++;
        
        if (frame_count % 30 == 0) {  // 每30帧记录一次
            ROS_INFO("成功接收到立体合并图像：帧 %d (尺寸: %dx%d → 合成: %dx%d)", 
                   frame_count, camera_image.cols, camera_image.rows, 
                   single_view.cols, single_view.rows);
        }
        
        // 转换为QImage，注意格式转换
        QImage image;
        if (single_view.channels() == 3) {
            // BGR转RGB
            cv::cvtColor(single_view, single_view, cv::COLOR_BGR2RGB);
            image = QImage(single_view.data, single_view.cols, single_view.rows,
                          single_view.step, QImage::Format_RGB888);
        } else if (single_view.channels() == 1) {
            // 灰度图
            image = QImage(single_view.data, single_view.cols, single_view.rows,
                          single_view.step, QImage::Format_Grayscale8);
        } else {
            ROS_WARN("不支持的图像格式: %d通道", single_view.channels());
            return;
        }
        
        // 保存图像的深拷贝，避免内存问题
        left_camera_image_ = image.copy();
        current_camera_image_ = image.copy();
        
        // 更新UI (在主线程中)
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("立体图像回调中的cv_bridge异常: %s", e.what());
    }
    catch (std::exception& e) {
        ROS_ERROR("立体图像回调中的标准异常: %s", e.what());
    }
    catch (...) {
        ROS_ERROR("立体图像回调中的未知异常");
    }
}

// 旧版updateCameraView函数实现（已被updateCameraViews替代，但为了兼容保留）
void ArmControlGUI::updateCameraView()
{
    updateCameraViews();
}

// 添加updateGUIJointValues函数实现
void ArmControlGUI::updateGUIJointValues()
{
    // 首次初始化默认关节值
    if (current_joint_values_.empty() || current_joint_values_.size() < 6) {
        current_joint_values_ = std::vector<double>{0, 0, 0, 0, M_PI/2, 5};
    }
    
    // 更新滑块和数值显示 
    ui->joint1_slider->setValue(static_cast<int>(current_joint_values_[0] * 180.0 / M_PI)); // 转换为度数
    ui->joint1_spin->setValue(current_joint_values_[0] * 180.0 / M_PI);
    
    ui->joint2_slider->setValue(static_cast<int>(current_joint_values_[1]));
    ui->joint2_spin->setValue(current_joint_values_[1]);
    
    ui->joint3_slider->setValue(static_cast<int>(current_joint_values_[2] * 180.0 / M_PI));
    ui->joint3_spin->setValue(current_joint_values_[2] * 180.0 / M_PI);
    
    ui->joint4_slider->setValue(static_cast<int>(current_joint_values_[3] * 180.0 / M_PI));
    ui->joint4_spin->setValue(current_joint_values_[3] * 180.0 / M_PI);
    
    // 关节5是固定的
    ui->joint6_slider->setValue(static_cast<int>(current_joint_values_[5]));
    ui->joint6_spin->setValue(current_joint_values_[5]);
}

// 添加初始化成员变量的函数
void ArmControlGUI::initializeMembers()
{
    // 机械臂状态初始化
    current_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    target_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // 关节限制
    joint_min_values_ = {-180.0, 0.0, -90.0, 0.0, 0.0, 5.0};
    joint_max_values_ = {180.0, 43.0, 90.0, 180.0, 0.0, 15.0};
    
    // 末端执行器状态
    gripper_open_ = true;
    vacuum_on_ = false;
    vacuum_power_ = 50;
    
    // 控制模式
    current_control_mode_ = ArmControlMode::JOINT_CONTROL;
    
    // 视觉相关
    visual_servo_active_ = false;
    selected_object_index_ = -1;
    
    // 更新定时器
    updateTimer = new QTimer(this);
    updateTimer->setInterval(100); // 10Hz更新
    connect(updateTimer, &QTimer::timeout, this, &ArmControlGUI::updateUI);
    updateTimer->start();
    
    // 日志消息
    logMessage("系统初始化完成，使用机械臂末端双目摄像头");
}

// 添加设置ROS订阅的函数
void ArmControlGUI::setupROSSubscriptions()
{
    // 订阅关节状态
    joint_state_sub_ = nh_.subscribe("/arm1/joint_states", 10, &ArmControlGUI::jointStateCallback, this);
    
    // 订阅合成立体图像
    stereo_merged_sub_ = nh_.subscribe("/stereo_camera/image_merged", 1, &ArmControlGUI::stereoMergedCallback, this);
    
    // 订阅检测图像
    detection_image_sub_ = nh_.subscribe("/detections/image", 1, &ArmControlGUI::detectionImageCallback, this);
    
    // 订阅检测位姿
    detection_poses_sub_ = nh_.subscribe("/detections/poses", 1, &ArmControlGUI::detectionPosesCallback, this);
    
    // 订阅YOLO状态
    yolo_status_sub_ = nh_.subscribe("/yolo/status", 1, &ArmControlGUI::yoloStatusCallback, this);
    
    // 设置同步订阅器用于同步处理检测图像和位姿
    detection_image_sub_filter_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/detections/image", 1);
    detection_poses_sub_filter_ = new message_filters::Subscriber<geometry_msgs::PoseArray>(nh_, "/detections/poses", 1);
    
    // 创建同步器
    object_detection_sync_ = new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(10), *detection_image_sub_filter_, *detection_poses_sub_filter_);
    
    // 注册回调函数
    object_detection_sync_->registerCallback(boost::bind(&ArmControlGUI::objectDetectionCallback, this, _1, _2));
    
    // 设置发布器
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm1/joint_command", 10);
    gripper_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/arm1/gripper_command", 10);
    vacuum_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/arm1/vacuum_command", 10);
    vacuum_power_pub_ = nh_.advertise<std_msgs::Float64>("/arm1/vacuum_power", 10);
    arm_command_pub_ = nh_.advertise<std_msgs::String>("/arm_commands", 10);
    
    // YOLO控制客户端
    yolo_control_client_ = nh_.serviceClient<std_srvs::SetBool>("/yolo_detector/enable");
    
    // 日志消息
    logMessage("已订阅ROS话题，等待数据...");
}