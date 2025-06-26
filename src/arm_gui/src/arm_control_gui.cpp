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

ArmControlGUI::ArmControlGUI(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::ArmControlMainWindow)
    , nh_(nh)
    , vacuum_on_(false)
    , vacuum_power_(0)
    , gripper_open_(false)
    , yolo_detection_enabled_(false)
    , selected_object_index_(-1)
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
    
    // 设置定时器用于更新3D场景
    QTimer *scene_timer = new QTimer(this);
    connect(scene_timer, &QTimer::timeout, this, &ArmControlGUI::updateScene3D);
    scene_timer->start(100); // 10Hz
    
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
    // 这里不需要额外的初始化，Scene3DRenderer会自行初始化
    logMessage("3D渲染初始化完成");
}

// 设置相机参数
void ArmControlGUI::setupCameraParameters()
{
    // 相机内参矩阵（fx, 0, cx; 0, fy, cy; 0, 0, 1）
    camera_intrinsic_.setToIdentity();
    camera_intrinsic_(0, 0) = 525.0; // fx
    camera_intrinsic_(1, 1) = 525.0; // fy
    camera_intrinsic_(0, 2) = 320.0; // cx
    camera_intrinsic_(1, 2) = 240.0; // cy
    
    // 初始化相机外参（世界坐标系到相机坐标系的变换）
    // 这里我们不再使用固定变换，而是会动态更新为末端执行器的位置
    camera_extrinsic_.setToIdentity();
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
    scene_objects_.clear();
    
    // 如果机械臂关节值为空，使用默认值
    if (current_joint_values_.empty() || current_joint_values_.size() < 6) {
        return;
    }
    
    // 计算末端执行器的位置和姿态
    geometry_msgs::Pose end_effector_pose = jointsToPos(current_joint_values_);
    
    // 更新相机外参矩阵，使其跟随末端执行器
    updateCameraTransform(end_effector_pose);
    
    // 将检测到的物体添加到场景中，但考虑从相机视角进行变换
    for (const DetectedObject& obj : detected_objects_) {
        // 创建物体在相机坐标系中的位置和颜色
        QVector3D position_camera(obj.x / 100.0f, obj.y / 100.0f, obj.z / 100.0f);
        
        // 转换到世界坐标系，考虑相机在机械臂末端的位置
        QVector4D position_world = camera_extrinsic_.inverted() * QVector4D(position_camera, 1.0f);
        QVector3D position(position_world.x(), position_world.y(), position_world.z());
        
        // 根据物体类型设置不同颜色
        QColor color(255, 0, 0); // 默认红色
        if (!obj.type.empty()) {
            // 根据物体类型设置不同颜色
            if (obj.type == "cube" || obj.type.find("cube") != std::string::npos) {
                color = QColor(0, 0, 255); // 蓝色
            } else if (obj.type == "cylinder" || obj.type.find("cylinder") != std::string::npos) {
                color = QColor(0, 255, 0); // 绿色
            } else if (obj.type == "sphere" || obj.type.find("sphere") != std::string::npos) {
                color = QColor(255, 255, 0); // 黄色
            }
        }
        
        scene_objects_.push_back(std::make_pair(position, color));
    }
    
    // 更新场景渲染器中的物体
    scene_3d_renderer_->updateObjects(scene_objects_);
}

// 新增函数：更新相机变换矩阵
void ArmControlGUI::updateCameraTransform(const geometry_msgs::Pose& end_effector_pose)
{
    // 创建从世界坐标系到末端执行器的变换矩阵
    QMatrix4x4 world_to_end_effector;
    
    // 设置平移部分
    world_to_end_effector.setColumn(3, QVector4D(
        end_effector_pose.position.x,
        end_effector_pose.position.y,
        end_effector_pose.position.z,
        1.0f
    ));
    
    // 从四元数创建旋转矩阵
    QQuaternion q(
        end_effector_pose.orientation.w,  // Qt quaternion 构造顺序是 w,x,y,z
        end_effector_pose.orientation.x,
        end_effector_pose.orientation.y,
        end_effector_pose.orientation.z
    );
    
    // 创建旋转矩阵并应用到变换矩阵
    QMatrix3x3 rot_mat;
    float x2 = q.x() * q.x();
    float y2 = q.y() * q.y();
    float z2 = q.z() * q.z();
    float xy = q.x() * q.y();
    float xz = q.x() * q.z();
    float yz = q.y() * q.z();
    float wx = q.scalar() * q.x();
    float wy = q.scalar() * q.y();
    float wz = q.scalar() * q.z();
    
    // 填充旋转矩阵
    rot_mat(0, 0) = 1.0f - 2.0f * (y2 + z2);
    rot_mat(0, 1) = 2.0f * (xy - wz);
    rot_mat(0, 2) = 2.0f * (xz + wy);
    rot_mat(1, 0) = 2.0f * (xy + wz);
    rot_mat(1, 1) = 1.0f - 2.0f * (x2 + z2);
    rot_mat(1, 2) = 2.0f * (yz - wx);
    rot_mat(2, 0) = 2.0f * (xz - wy);
    rot_mat(2, 1) = 2.0f * (yz + wx);
    rot_mat(2, 2) = 1.0f - 2.0f * (x2 + y2);
    
    // 设置旋转部分
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            world_to_end_effector(i, j) = rot_mat(i, j);
        }
    }
    
    // 相机在末端执行器上的偏移（吸盘位置，向下看）
    QMatrix4x4 end_effector_to_camera;
    end_effector_to_camera.setToIdentity();
    // 假设相机安装在末端吸盘上，朝向下方
    // 旋转90度使相机朝下
    end_effector_to_camera.rotate(90, 1, 0, 0);
    // 平移到吸盘位置
    end_effector_to_camera.translate(0, 0, 0.05); // 5cm偏移
    
    // 最终的相机外参矩阵 = 世界到末端 * 末端到相机
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
        
        // 保存检测图像
        detection_image_ = image;
        
        // 使用检测图像作为当前显示图像
        current_camera_image_ = image;
        
        // 更新UI
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge 异常 in detection callback: %s", e.what());
    }
}

void ArmControlGUI::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // 清空之前的检测结果
    detected_objects_.clear();
    
    // 如果没有检测到物体，直接返回
    if (msg->poses.empty()) {
        // 更新UI (在主线程中)
        QMetaObject::invokeMethod(this, "updateDetectionsTableUI", Qt::QueuedConnection);
        QMetaObject::invokeMethod(this, "updateScene3D", Qt::QueuedConnection);
        return;
    }
    
    // 处理新的检测结果
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        DetectedObject obj;
        obj.id = "object_" + std::to_string(i + 1);
        obj.type = "未知"; // 默认类型
        
        // 提取3D位置信息（单位：厘米）
        // 注意：这里的位置是在相机坐标系中的，不需要转换
        obj.x = msg->poses[i].position.x * 100.0;
        obj.y = msg->poses[i].position.y * 100.0;
        obj.z = msg->poses[i].position.z * 100.0;
        
        // 保存原始位姿
        obj.pose = msg->poses[i];
        
        // 添加到检测结果列表
        detected_objects_.push_back(obj);
    }
    
    // 更新UI (在主线程中)
    QMetaObject::invokeMethod(this, "updateDetectionsTableUI", Qt::QueuedConnection);
    
    // 也更新3D场景
    QMetaObject::invokeMethod(this, "updateScene3D", Qt::QueuedConnection);
    
    // 记录信息
    logMessage(QString("已检测到 %1 个物体").arg(detected_objects_.size()));
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
    // 检查是否有有效图像
    if (current_camera_image_.isNull()) {
        return;
    }
    
    // 创建用于显示的图像副本
    QImage display_image = current_camera_image_.copy();
    QPainter painter(&display_image);
    
    // 绘制检测到的物体
    if (yolo_detection_enabled_ && !detected_objects_.empty()) {
    for (size_t i = 0; i < detected_objects_.size(); ++i) {
        const DetectedObject& obj = detected_objects_[i];
            
            // 使用物体在相机坐标系中的原始位置
            QVector3D obj_camera(obj.x / 100.0f, obj.y / 100.0f, obj.z / 100.0f);
        
        // 将3D位置投影到图像平面
            QPoint obj_image = QPoint(
                static_cast<int>(obj_camera.x() / obj_camera.z() * camera_intrinsic_(0, 0) + camera_intrinsic_(0, 2)),
                static_cast<int>(obj_camera.y() / obj_camera.z() * camera_intrinsic_(1, 1) + camera_intrinsic_(1, 2))
            );
            
            // 确保点在图像范围内
            if (obj_image.x() < 0 || obj_image.x() >= display_image.width() ||
                obj_image.y() < 0 || obj_image.y() >= display_image.height() ||
                obj_camera.z() <= 0.0f) {
                continue; // 不在视野内的物体不显示
            }
        
        // 设置画笔颜色和宽度
        if (static_cast<int>(i) == selected_object_index_) {
            painter.setPen(QPen(Qt::yellow, 3));
        } else {
            painter.setPen(QPen(Qt::red, 2));
        }
        
        // 计算矩形大小（基于物体距离）
            // 距离越远，框越小
            float distance_factor = 0.5f / obj_camera.z();
            int box_size = static_cast<int>(40 * distance_factor);
            box_size = std::max(20, std::min(80, box_size)); // 限制大小范围
            
            QRect rect(obj_image.x() - box_size/2, obj_image.y() - box_size/2, box_size, box_size);
                    
                    // 绘制矩形框
        painter.drawRect(rect);
        
        // 设置字体和颜色
        QFont font = painter.font();
        font.setBold(true);
        font.setPointSize(10);
                    painter.setFont(font);
        
        if (static_cast<int>(i) == selected_object_index_) {
            painter.setPen(QPen(Qt::yellow));
        } else {
            painter.setPen(QPen(Qt::white));
        }
        
        // 准备文本
        QString text = QString::fromStdString(obj.id);
        
        // 获取文本尺寸
                    QFontMetrics fm(font);
        QRect text_rect = fm.boundingRect(text);
        
        // 绘制背景
        QRect background_rect(
                obj_image.x() - text_rect.width()/2 - 2,
                obj_image.y() - box_size/2 - text_rect.height() - 4,
            text_rect.width() + 4,
            text_rect.height() + 4
        );
        
        painter.fillRect(background_rect, QColor(0, 0, 0, 128));
        
        // 绘制文本
        painter.drawText(
                obj_image.x() - text_rect.width()/2,
                obj_image.y() - box_size/2 - 4,
            text
        );
            
            // 绘制距离信息
            QString distance_text = QString::number(obj_camera.z(), 'f', 2) + "m";
            QRect distance_rect = fm.boundingRect(distance_text);
            
            QRect distance_bg_rect(
                obj_image.x() - distance_rect.width()/2 - 2,
                obj_image.y() + box_size/2 + 2,
                distance_rect.width() + 4,
                distance_rect.height() + 4
            );
            
            painter.fillRect(distance_bg_rect, QColor(0, 0, 0, 128));
            
            painter.drawText(
                obj_image.x() - distance_rect.width()/2,
                obj_image.y() + box_size/2 + distance_rect.height() + 2,
                distance_text
            );
        }
    }
    
    // 更新相机视图
    ui->cameraView->setPixmap(QPixmap::fromImage(display_image));
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

// 修改YOLO状态回调函数
void ArmControlGUI::yoloStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
    bool was_enabled = yolo_detection_enabled_;
    yolo_detection_enabled_ = msg->data;
    
    if (was_enabled != yolo_detection_enabled_) {
        // 状态发生变化，更新UI
        if (yolo_checkbox_) {
            yolo_checkbox_->blockSignals(true);
            yolo_checkbox_->setChecked(yolo_detection_enabled_);
            yolo_checkbox_->blockSignals(false);
        }
        
        logMessage(QString("YOLO检测状态变为: %1").arg(yolo_detection_enabled_ ? "启用" : "禁用"));
        
        // 更新视图
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
    }
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
    // 生成相机测试图像 - 仅用于初始显示，直到真实相机连接
    QImage testCamera(640, 480, QImage::Format_RGB888);
    testCamera.fill(Qt::black);
    
    QPainter painterCamera(&testCamera);
    painterCamera.setPen(QPen(Qt::white, 2));
    painterCamera.setFont(QFont("Arial", 14, QFont::Bold));
    
    painterCamera.drawText(50, 100, "等待相机连接");
    painterCamera.drawText(50, 150, "请确保相机已连接");
    painterCamera.drawText(50, 200, "并启用YOLO目标检测");
    
    // 保存测试图像
    left_camera_image_ = testCamera;
    current_camera_image_ = testCamera;
    
    // 不再生成测试物体，只使用摄像头检测到的真实物体
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
    // 保存当前选中行
    int currentRow = ui->detectionsTable->currentRow();
    
    // 清空表格
    ui->detectionsTable->setRowCount(0);
    
    // 填充表格
    for (size_t i = 0; i < detected_objects_.size(); ++i) {
        const DetectedObject& obj = detected_objects_[i];
        
        int row = ui->detectionsTable->rowCount();
        ui->detectionsTable->insertRow(row);
        
        // ID列
        ui->detectionsTable->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(obj.id)));
        
        // 类型列
        ui->detectionsTable->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(obj.type)));
        
        // X坐标列
        ui->detectionsTable->setItem(row, 2, new QTableWidgetItem(QString::number(obj.x, 'f', 1)));
        
        // Y坐标列
        ui->detectionsTable->setItem(row, 3, new QTableWidgetItem(QString::number(obj.y, 'f', 1)));
        
        // Z坐标列
        ui->detectionsTable->setItem(row, 4, new QTableWidgetItem(QString::number(obj.z, 'f', 1)));
        
        // 操作列 - 添加抓取按钮
        QPushButton* pickButton = new QPushButton("抓取");
        pickButton->setProperty("object_id", QString::fromStdString(obj.id));
        pickButton->setProperty("object_index", static_cast<int>(i));
        
        connect(pickButton, &QPushButton::clicked, this, [this, i]() {
            this->sendPickObjectCommand(i);
        });
        
        ui->detectionsTable->setCellWidget(row, 5, pickButton);
    }
    
    // 恢复选中行
    if (currentRow >= 0 && currentRow < ui->detectionsTable->rowCount()) {
        ui->detectionsTable->selectRow(currentRow);
    } else if (selected_object_index_ >= 0 && selected_object_index_ < ui->detectionsTable->rowCount()) {
        ui->detectionsTable->selectRow(selected_object_index_);
    }
    
    // 连接表格选择信号
    connect(ui->detectionsTable, &QTableWidget::cellClicked, 
            this, &ArmControlGUI::onDetectionsTableCellClicked, Qt::UniqueConnection);
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
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in stereo merged callback: %s", e.what());
    }
}

// 旧版updateCameraView函数实现（已被updateCameraViews替代，但为了兼容保留）
void ArmControlGUI::updateCameraView()
{
    updateCameraViews();
} 