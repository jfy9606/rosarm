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
    
    // 连接UI文件中定义的路径规划控件
    connect(ui->scanObjectsButton, &QPushButton::clicked, this, &ArmControlGUI::onScanObjectsClicked);
    connect(ui->planPathButton, &QPushButton::clicked, this, &ArmControlGUI::onPlanPathClicked);
    connect(ui->executePathButton, &QPushButton::clicked, this, &ArmControlGUI::onExecutePathClicked);
    connect(ui->visualizeWorkspaceButton, &QPushButton::clicked, this, &ArmControlGUI::onVisualizeWorkspaceClicked);
    
    // 保存placement_area_combo_指针，以便在路径规划中使用
    placement_area_combo_ = ui->placement_area_combo;
    placement_area_combo_->clear();
    placement_area_combo_->addItem("区域1", "area_1");
    placement_area_combo_->addItem("区域2", "area_2");
    placement_area_combo_->addItem("区域3", "area_3");
    
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
    
    // 将示例动作组和视觉控制组添加到controlWidget的布局中
    QVBoxLayout* controlLayout = qobject_cast<QVBoxLayout*>(ui->controlWidget->layout());
    if (controlLayout) {
        controlLayout->addWidget(demoGroup);
        controlLayout->addWidget(visionGroup);  // 添加视觉控制组
        // 注意：不再创建和添加新的路径规划控件，使用UI中已有的
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
    // 设置DH参数 (基于examples/path/main.m)
    // a1 = 13, a2 = 13, a3 = 25, d2 = [0, 43], d5 = [5, 15]
    dh_params_.resize(6);
    // [type, d, theta, a, alpha]
    // type: 0=revolute, 1=prismatic
    dh_params_[0] = std::make_tuple(0, 0.0, 0.0, 13.0, M_PI/2);  // 第一关节 (旋转)
    dh_params_[1] = std::make_tuple(1, 0.0, M_PI/4, 0.0, 0.0);   // 第二关节 (伸缩)
    dh_params_[2] = std::make_tuple(0, 13.0, 0.0, 0.0, M_PI/2);  // 第三关节 (旋转)
    dh_params_[3] = std::make_tuple(0, 0.0, 0.0, 25.0, M_PI/2);  // 第四关节 (旋转)
    dh_params_[4] = std::make_tuple(0, 0.0, M_PI/2, 0.0, M_PI/2); // 第五关节 (固定)
    dh_params_[5] = std::make_tuple(1, 5.0, M_PI/2, 0.0, 0.0);   // 第六关节 (伸缩)
    
    // 关节限制，确保与examples中一致
    joint_limits_.resize(6);
    joint_limits_[0] = std::make_pair(-M_PI, M_PI);       // theta1 (rad)
    joint_limits_[1] = std::make_pair(0.0, 43.0);         // d2 (cm)
    joint_limits_[2] = std::make_pair(-M_PI/2, M_PI/2);   // theta3 (rad)
    joint_limits_[3] = std::make_pair(0.0, M_PI);         // theta4 (rad)
    joint_limits_[4] = std::make_pair(M_PI/2, M_PI/2);    // theta5 (rad, fixed)
    joint_limits_[5] = std::make_pair(5.0, 15.0);         // d6 (cm)

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
    // 相机内参矩阵（fx, 0, cx; 0, fy, cy; 0, 0, 1）
    camera_intrinsic_.setToIdentity();
    camera_intrinsic_(0, 0) = 525.0; // fx
    camera_intrinsic_(1, 1) = 525.0; // fy
    camera_intrinsic_(0, 2) = 320.0; // cx
    camera_intrinsic_(1, 2) = 240.0; // cy
    
    // 初始化相机外参（世界坐标系到相机坐标系的变换）
    // 这里我们不再使用固定变换，而是会动态更新为末端执行器的位置
    camera_extrinsic_.setToIdentity();
    
    // 设置相机帧率为30FPS（摄像头最高支持30FPS，不支持60FPS）
    ros::NodeHandle nh_private("~");
    nh_private.setParam("/camera/frame_rate", 30);
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
    double x = ui->pos_x->value();
    double y = ui->pos_y->value();
    double z = ui->pos_z->value();
    
    logMessage(QString("移动到位置 (%1, %2, %3)").arg(x).arg(y).arg(z));
    
    // 参照示例动作2实现，使用进给电机控制末端执行器移动
    // 将xyz坐标转换为电机位置，这里使用z坐标作为进给深度
    int motor_pos = -static_cast<int>(z * 20); // 将z坐标(mm)映射到电机位置值
    
    // 限制电机位置范围
    motor_pos = std::max(-2500, std::min(motor_pos, 0));
    
    // 执行示例动作2中的电机控制序列
    // 先设置电机速度和模式
    sendMotorOrder(2, 100, 100, 0, 0, false, motor_pos, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    
    // 设置目标位置
    sendMotorOrder(2, 0, 100, 0, 0, false, motor_pos, 10);
    QApplication::processEvents();
    ros::Duration(0.5).sleep();
    
    // 执行移动
    sendMotorOrder(2, 99, 100, 0, 0, false, motor_pos, 10);
    QApplication::processEvents();
    
    // 同时发送标准的位置命令以保持兼容性
    sendPlaceCommand(x, y, z);
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
    vacuum_on_ = true;
    
    // 参照示例动作2实现，使用继电器命令控制吸附
    sendRelayOrder("11");
    QApplication::processEvents();
    
    // 同时发送标准的吸附命令以保持兼容性
    sendVacuumCommand(true, vacuum_power_);
    
    logMessage(QString("开启吸附，功率: %1%").arg(vacuum_power_));
}

void ArmControlGUI::onVacuumOffButtonClicked()
{
    vacuum_on_ = false;
    
    // 参照示例动作2实现，使用继电器命令控制吸附
    sendRelayOrder("00");
    QApplication::processEvents();
    
    // 同时发送标准的吸附命令以保持兼容性
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
        cv::Mat detection_image = cv_ptr->image.clone();
        
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
        
        // 保存YOLOv8检测结果图像
        current_camera_image_ = image.copy();
        
        // 更新UI
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
        
        // 记录日志
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 30 == 0) {  // 每30帧记录一次
            ROS_INFO("已接收YOLOv8检测图像: 帧 %d, 尺寸 %dx%d", 
                    frame_count, image.width(), image.height());
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("YOLOv8检测图像回调中的cv_bridge异常: %s", e.what());
    } catch (std::exception& e) {
        ROS_ERROR("YOLOv8检测图像回调中的标准异常: %s", e.what());
    }
}

void ArmControlGUI::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // 处理物体位置数据
    detected_objects_.clear();
    
    // 没有检测到物体，清空表格并返回
    if (msg->poses.empty()) {
        QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
        return;
    }
    
    // 启用YOLO检测状态，确保GUI显示检测结果
    yolo_detection_enabled_ = true;
    
    // 检查是否有附加信息（通常应该包含类别名称），当前假设这些信息在frames_id中
    bool has_object_ids = false;
    if (!msg->header.frame_id.empty()) {
        // 尝试解析frame_id中可能包含的类别信息
        try {
            // 假设frame_id格式是 "class1,class2,class3,..."
            std::string frame_id = msg->header.frame_id;
            std::istringstream ss(frame_id);
            std::string token;
            std::vector<std::string> class_names;
            
            while (std::getline(ss, token, ',')) {
                if (!token.empty()) {
                    class_names.push_back(token);
                }
            }
            
            has_object_ids = !class_names.empty() && class_names.size() == msg->poses.size();
            
            // 如果成功解析类别信息，直接使用它们
            if (has_object_ids) {
                for (size_t i = 0; i < msg->poses.size(); ++i) {
                    DetectedObject obj;
                    obj.id = class_names[i];  // 使用传递过来的类别名称
                    obj.type = "检测物品";    // 通用类型描述
                    
                    // 将3D位置转换为厘米单位用于显示
                    double scale = 100.0; // 转换为厘米
                    obj.x = msg->poses[i].position.x * scale;
                    obj.y = msg->poses[i].position.y * scale;
                    obj.z = msg->poses[i].position.z * scale;
                    
                    // 保存原始位姿
                    obj.pose = msg->poses[i];
                    detected_objects_.push_back(obj);
                    
                    // 记录成功添加的物体（仅用于调试）
                    ROS_DEBUG("添加检测物体到表格: %s [%.2f, %.2f, %.2f]", 
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
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            DetectedObject obj;
            obj.id = "object_" + std::to_string(i);  // 通用对象名
            
            // 根据物体位置特征决定类型名称
            float height = msg->poses[i].position.z;
            if (height < 0.05) {
                obj.type = "low object";
            } else if (height < 0.15) {
                obj.type = "medium object";
            } else {
                obj.type = "tall object";
            }
            
            // 将3D位置转换为厘米单位用于显示
            double scale = 100.0; // 转换为厘米
            obj.x = msg->poses[i].position.x * scale;
            obj.y = msg->poses[i].position.y * scale;
            obj.z = msg->poses[i].position.z * scale;
            
            // 保存原始位姿
            obj.pose = msg->poses[i];
            detected_objects_.push_back(obj);
            
            // 记录成功添加的物体（仅用于调试）
            ROS_DEBUG("添加通用物体到表格: %s [%.2f, %.2f, %.2f]", 
                     obj.id.c_str(), obj.x, obj.y, obj.z);
        }
    }
    
    // 确保在主线程中更新UI，立即更新表格
    QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
    
    // 记录检测到的物体数量（仅用于调试）
    ROS_DEBUG("检测到 %zu 个物体，已更新表格", detected_objects_.size());
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
    // 如果机械臂坐标不是必要信息，则不显示在GUI上
    // 我们只在开发或调试模式下显示这些信息
    
    // 获取是否处于调试模式 - 可以通过参数或环境变量控制
    bool debug_mode = false;  // 默认不显示机械臂坐标
    
    if (debug_mode) {
        // 计算末端位姿
        current_end_effector_pose_ = jointsToPos(current_joint_values_);
        
        // 更新位置输入框(转换为厘米)
        ui->pos_x->setValue(current_end_effector_pose_.position.x * 100);
        ui->pos_y->setValue(current_end_effector_pose_.position.y * 100);
        ui->pos_z->setValue(current_end_effector_pose_.position.z * 100);
    } else {
        // 在非调试模式下，不显示机械臂坐标
        // 可以将位置输入框设为零或隐藏
        ui->pos_x->setValue(0);
        ui->pos_y->setValue(0);
        ui->pos_z->setValue(0);
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
    // 检查是否有有效图像
    if (current_camera_image_.isNull()) {
        return;
    }
    
    // 创建用于显示的图像副本
    QImage display_image = current_camera_image_.copy();
    QPainter painter(&display_image);
    
    // 在图像底部添加分辨率信息
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 10));
    QString resolution_text = QString("%1 x %2").arg(display_image.width()).arg(display_image.height());
    painter.drawText(10, display_image.height() - 10, resolution_text);
    
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
        
            // 准备文本 - 使用物体实际名称
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
        
            // 绘制物体名称
            painter.drawText(
                obj_image.x() - text_rect.width()/2,
                obj_image.y() - box_size/2 - 4,
                text
            );
            
            // 绘制类型信息而非距离
            QString type_text = QString::fromStdString(obj.type);
            QRect type_rect = fm.boundingRect(type_text);
            
            QRect type_bg_rect(
                obj_image.x() - type_rect.width()/2 - 2,
                obj_image.y() + box_size/2 + 2,
                type_rect.width() + 4,
                type_rect.height() + 4
            );
            
            painter.fillRect(type_bg_rect, QColor(0, 0, 0, 128));
            
            painter.drawText(
                obj_image.x() - type_rect.width()/2,
                obj_image.y() + box_size/2 + type_rect.height() + 2,
                type_text
            );
        }
    }
    
    // 更新相机视图，缩小以适配显示区域
    QPixmap pixmap = QPixmap::fromImage(display_image);
    
    // 获取相机视图控件的尺寸
    QSize viewSize = ui->cameraView->size();
    
    // 计算合适的缩放比例，保持图像比例不变
    float scaleFactor = qMin(
        static_cast<float>(viewSize.width()) / pixmap.width(),
        static_cast<float>(viewSize.height()) / pixmap.height()
    ) * 0.95; // 再缩小5%以确保完全显示
    
    // 缩放图像
    QSize scaledSize(pixmap.width() * scaleFactor, pixmap.height() * scaleFactor);
    pixmap = pixmap.scaled(scaledSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    
    // 设置显示
    ui->cameraView->setPixmap(pixmap);
    
    // 更新检测表格
    if (yolo_detection_enabled_ && !detected_objects_.empty()) {
        updateDetectionsTable();
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
    // 更新YOLO检测状态
    bool new_status = msg->data;
    
    // 只有状态变化时才记录日志和更新UI
    if (new_status != yolo_detection_enabled_) {
        yolo_detection_enabled_ = new_status;
        
        // 更新复选框状态，但不触发回调
        if (yolo_checkbox_) {
            yolo_checkbox_->blockSignals(true);
            yolo_checkbox_->setChecked(yolo_detection_enabled_);
            yolo_checkbox_->blockSignals(false);
        }
        
        // 更新状态栏信息
        QString status_msg = QString("YOLO目标检测已%1").arg(yolo_detection_enabled_ ? "启用" : "禁用");
        ui->statusbar->showMessage(status_msg, 3000);
    }
    
    // 无论是否有变化，都确保UI反映当前状态
    updateCameraViews();
    updateDetectionsTable();
}

// 修改YOLO切换处理函数
void ArmControlGUI::onYoloDetectionToggled(bool checked)
{
    // 记录操作到日志
    logMessage(QString("尝试%1 YOLO检测...").arg(checked ? "启用" : "禁用"));
    
    // 无论服务是否可用，先将状态保存起来
    yolo_detection_enabled_ = checked;
    
    // 如果禁用检测，清空物体表格
    if (!checked) {
        detected_objects_.clear();
        updateDetectionsTable();
    }
    
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
            
            // 如果禁用检测，再次确保清空物体表格
            if (!checked) {
                detected_objects_.clear();
                updateDetectionsTable();
            }
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
    updateDetectionsTable();
}

// 添加路径规划相关的槽函数
void ArmControlGUI::onScanObjectsClicked()
{
    logMessage("开始扫描物体...");
    
    // 确保YOLO检测已启用
    if (!yolo_detection_enabled_) {
        logMessage("自动启用YOLO检测以扫描物体");
        if (yolo_checkbox_) {
            yolo_checkbox_->setChecked(true);
        }
        
        // 等待YOLO服务启动
        QApplication::processEvents();
        ros::Duration(0.5).sleep();
    }
    
    // 发送扫描命令 - 使用YOLOv8s进行实时扫描
    std_msgs::String cmd_msg;
    cmd_msg.data = "scan yolov8s";  // 指定使用YOLOv8s模型
    arm_command_pub_.publish(cmd_msg);
    
    // 显示状态信息
    ui->statusbar->showMessage("正在使用YOLOv8s模型扫描物体...", 2000);
    logMessage("使用YOLOv8s模型扫描合成摄像头图像");
}

void ArmControlGUI::onPlanPathClicked()
{
    logMessage("开始规划路径...");
    
    // 检查是否有检测到的物体
    if (detected_objects_.empty()) {
        logMessage("没有检测到物体，请先扫描物体");
        ui->statusbar->showMessage("错误: 没有检测到物体，请先扫描物体", 3000);
        return;
    }
    
    // 获取当前选中的物体（如果有在表格中选中）
    int selected_row = ui->detectionsTable->currentRow();
    const DetectedObject* selected_obj = nullptr;
    
    // 如果有选中的行，使用该行对应的物体，否则使用第一个物体
    if (selected_row >= 0 && selected_row < static_cast<int>(detected_objects_.size())) {
        selected_obj = &detected_objects_[selected_row];
    } else {
        selected_obj = &detected_objects_[0];
    }
    
    QString object_id = QString::fromStdString(selected_obj->id);
    
    // 获取选中的放置区
    QString placement_area = placement_area_combo_->currentData().toString();
    
    // 发送规划命令
    std_msgs::String cmd_msg;
    cmd_msg.data = QString("plan %1 %2").arg(object_id).arg(placement_area).toStdString();
    arm_command_pub_.publish(cmd_msg);
    
    logMessage(QString("已规划从物体 %1 到放置区 %2 的路径").arg(object_id).arg(placement_area));
    ui->statusbar->showMessage(QString("已规划路径: 物体 %1 → 放置区 %2").arg(object_id).arg(placement_area), 3000);
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
        
        // 如果没有启用YOLO检测，则不处理
        if (!yolo_detection_enabled_) {
            return;
        }
        
        // 处理物体位置数据
        detected_objects_.clear();
        for (size_t i = 0; i < poses_msg->poses.size(); ++i) {
            DetectedObject obj;
            
            // 使用自定义物品名称而不是object_*
            // 为每个检测分配一个更有意义的名称和类型
            float height = poses_msg->poses[i].position.z;
            
            // 根据物体位置特征决定类型名称
            std::string type_name;
            if (height < 0.05) {
                type_name = "低矮物品";
            } else if (height < 0.15) {
                type_name = "中等物品";
            } else {
                type_name = "高大物品";
            }
            
            // 根据检测顺序分配物品名称，这里列出了常见物品
            const std::vector<std::string> item_names = {
                "水杯", "鼠标", "键盘", "手机", "书本", 
                "笔", "剪刀", "USB盘", "眼镜", "钥匙", 
                "螺丝刀", "玩具", "橡皮擦", "胶带", "标签"
            };
            
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
            
            // 在图像上绘制检测框和物体名称
            float pos_x = (poses_msg->poses[i].position.x + 0.5) * detection_image.cols;
            float pos_y = (0.5 - poses_msg->poses[i].position.y) * detection_image.rows;
            
            // 确保坐标在图像范围内
            if (pos_x >= 0 && pos_x < detection_image.cols && 
                pos_y >= 0 && pos_y < detection_image.rows) {
                
                // 绘制圆圈标记物体位置
                cv::circle(detection_image, cv::Point(pos_x, pos_y), 10, cv::Scalar(0, 255, 0), 2);
                
                // 添加文本标签 - 显示实际物品名称
                cv::putText(detection_image, obj.id, cv::Point(pos_x, pos_y - 15),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
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
        
        // 更新UI
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
        QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
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

// 添加测试图像生成函数
void ArmControlGUI::generateTestImages()
{
    // 这个函数将被移除，因为我们使用真实相机图像
}

// 添加更新检测结果表格的函数
void ArmControlGUI::updateDetectionsTable()
{
    // 获取表格控件
    QTableWidget* table = ui->detectionsTable;
    
    // 保存当前选中的行
    int currentRow = table->currentRow();
    
    // 阻止信号以避免触发不必要的事件
    table->blockSignals(true);
    
    // 清空表格
    table->clearContents();
    table->setRowCount(detected_objects_.size());
    
    // 设置表头
    if (table->columnCount() < 5) {
        table->setColumnCount(5);
        table->setHorizontalHeaderLabels({"名称", "类型", "X(cm)", "Y(cm)", "Z(cm)"});
    }
    
    // 填充表格数据
    for (size_t i = 0; i < detected_objects_.size(); ++i) {
        const DetectedObject& obj = detected_objects_[i];
        
        // 名称
        QTableWidgetItem* nameItem = new QTableWidgetItem(QString::fromStdString(obj.id));
        nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable); // 设为只读
        table->setItem(i, 0, nameItem);
        
        // 类型
        QTableWidgetItem* typeItem = new QTableWidgetItem(QString::fromStdString(obj.type));
        typeItem->setFlags(typeItem->flags() & ~Qt::ItemIsEditable); // 设为只读
        table->setItem(i, 1, typeItem);
        
        // X坐标（厘米）
        QTableWidgetItem* xItem = new QTableWidgetItem(QString::number(obj.x, 'f', 1));
        xItem->setFlags(xItem->flags() & ~Qt::ItemIsEditable); // 设为只读
        table->setItem(i, 2, xItem);
        
        // Y坐标（厘米）
        QTableWidgetItem* yItem = new QTableWidgetItem(QString::number(obj.y, 'f', 1));
        yItem->setFlags(yItem->flags() & ~Qt::ItemIsEditable); // 设为只读
        table->setItem(i, 3, yItem);
        
        // Z坐标（厘米）
        QTableWidgetItem* zItem = new QTableWidgetItem(QString::number(obj.z, 'f', 1));
        zItem->setFlags(zItem->flags() & ~Qt::ItemIsEditable); // 设为只读
        table->setItem(i, 4, zItem);
        
        // 如果这个物体是当前选中的物体，设置其背景颜色
        if (static_cast<int>(i) == selected_object_index_) {
            for (int col = 0; col < table->columnCount(); ++col) {
                if (table->item(i, col)) {
                    table->item(i, col)->setBackground(QColor(255, 255, 0, 64)); // 浅黄色背景
                }
            }
        }
    }
    
    // 恢复表格信号
    table->blockSignals(false);
    
    // 恢复选中行
    if (currentRow >= 0 && currentRow < table->rowCount()) {
        table->selectRow(currentRow);
    }
    
    // 自动调整列宽以适应内容
    table->resizeColumnsToContents();
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
    // 初始化标志和状态变量
    gripper_open_ = true;
    vacuum_on_ = false;
    vacuum_power_ = 50;
    yolo_detection_enabled_ = false;
    enable_3d_rendering_ = true;
    selected_object_index_ = -1;
    object_detection_sync_ = nullptr;
    
    // 创建定时器用于更新UI
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &ArmControlGUI::updateUI);
    updateTimer->start(100); // 10Hz
    
    // 初始化空白图像等待摄像头连接
    QImage emptyImage(640, 480, QImage::Format_RGB888);
    emptyImage.fill(Qt::black);
    QPainter painter(&emptyImage);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 14, QFont::Bold));
    painter.drawText(emptyImage.rect(), Qt::AlignCenter, "等待摄像头连接...");
    
    left_camera_image_ = emptyImage;
    current_camera_image_ = emptyImage;
}

// 添加设置ROS订阅的函数
void ArmControlGUI::setupROSSubscriptions()
{
    // 订阅关节状态，包含舵机位置等信息
    joint_state_sub_ = nh_.subscribe("/arm1/joint_states", 10, &ArmControlGUI::jointStateCallback, this);
    
    // 订阅电机状态，实时获取电机位置和电流
    ros::Subscriber motor_state_sub = nh_.subscribe("/motor_states", 10, &ArmControlGUI::motorStateCallback, this);
    
    // 订阅舵机状态，实时获取舵机位置
    ros::Subscriber servo_state_sub = nh_.subscribe("/servo_states", 10, &ArmControlGUI::servoStateCallback, this);
    
    // 订阅合成的双目摄像头话题，无论分辨率如何，都使用这个作为GUI显示源
    stereo_merged_sub_ = nh_.subscribe("/stereo_camera/image_merged", 1, &ArmControlGUI::stereoMergedCallback, this);
    
    // 备份订阅，如果没有立体相机，则使用普通摄像头
    ros::Subscriber raw_camera_sub = nh_.subscribe("/camera/image_raw", 1, &ArmControlGUI::stereoMergedCallback, this);
    
    // 使用YOLO检测结果订阅
    detection_image_sub_ = nh_.subscribe("/yolo_detection/image", 1, &ArmControlGUI::detectionImageCallback, this);
    detection_poses_sub_ = nh_.subscribe("/yolo_detection/poses", 1, &ArmControlGUI::detectionPosesCallback, this);
    
    // 创建消息过滤器，同步处理图像和物体位置信息
    message_filters::Subscriber<sensor_msgs::Image> detection_image_filter(nh_, "/yolo_detection/image", 1);
    message_filters::Subscriber<geometry_msgs::PoseArray> detection_poses_filter(nh_, "/yolo_detection/poses", 1);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseArray> MySyncPolicy;
    object_detection_sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), detection_image_filter, detection_poses_filter);
    object_detection_sync_->registerCallback(boost::bind(&ArmControlGUI::objectDetectionCallback, this, _1, _2));
    
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
}

// 电机状态回调函数
void ArmControlGUI::motorStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // 处理电机状态数据
    if (!msg->name.empty() && !msg->position.empty()) {
        // 更新机械臂3D模型
        if (scene_3d_renderer_ && current_joint_values_.size() >= 6) {
            // 将电机位置更新到当前关节值中
            for (size_t i = 0; i < msg->name.size(); ++i) {
                const std::string& name = msg->name[i];
                
                // 根据电机名称更新对应关节
                if (name == "base_motor" && i < msg->position.size()) {
                    current_joint_values_[0] = msg->position[i];  // 底座旋转关节
                }
                else if (name == "lift_motor" && i < msg->position.size()) {
                    current_joint_values_[1] = msg->position[i];  // 升降关节
                }
                else if (name == "arm_motor" && i < msg->position.size()) {
                    current_joint_values_[3] = msg->position[i];  // 臂部旋转关节
                }
            }
            
            // 通知3D渲染器更新机械臂姿势
            scene_3d_renderer_->setRobotPose(current_joint_values_);
        }
    }
    
    // 显示电机运行状态
    if (!msg->name.empty() && !msg->velocity.empty() && !msg->effort.empty()) {
        // 找到最活跃的电机（速度最大）
        size_t active_motor = 0;
        double max_velocity = 0.0;
        
        for (size_t i = 0; i < msg->velocity.size(); ++i) {
            if (std::fabs(msg->velocity[i]) > max_velocity) {
                max_velocity = std::fabs(msg->velocity[i]);
                active_motor = i;
            }
        }
        
        // 如果有活跃的电机，显示其状态
        if (max_velocity > 0.1) {
            QString motor_name = QString::fromStdString(
                active_motor < msg->name.size() ? msg->name[active_motor] : "未知电机");
            
            // 在工具栏显示状态而不是状态栏，以免和其他消息冲突
            static QLabel* motorStatusLabel = nullptr;
            if (!motorStatusLabel) {
                motorStatusLabel = new QLabel(this);
                ui->toolBar->addWidget(motorStatusLabel);
            }
            
            double current = active_motor < msg->effort.size() ? msg->effort[active_motor] : 0.0;
            motorStatusLabel->setText(QString("%1: %2 rpm, %3 A").arg(
                motor_name).arg(max_velocity, 0, 'f', 1).arg(current, 0, 'f', 2));
        }
    }
}

// 舵机状态回调函数
void ArmControlGUI::servoStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // 处理舵机状态数据
    if (!msg->name.empty() && !msg->position.empty()) {
        // 更新机械臂3D模型
        if (scene_3d_renderer_ && current_joint_values_.size() >= 6) {
            // 将舵机位置更新到当前关节值中
            for (size_t i = 0; i < msg->name.size(); ++i) {
                const std::string& name = msg->name[i];
                
                // 根据舵机名称更新对应关节
                if (name == "wrist_servo" && i < msg->position.size()) {
                    current_joint_values_[4] = msg->position[i];  // 腕部旋转关节
                }
                else if (name == "gripper_servo" && i < msg->position.size()) {
                    // 吸盘长度或夹爪开合，映射到关节值5
                    current_joint_values_[5] = msg->position[i]; 
                    
                    // 更新吸盘状态
                    gripper_open_ = msg->position[i] < 0.5; // 假设<0.5表示开
                }
            }
            
            // 通知3D渲染器更新机械臂姿势
            scene_3d_renderer_->setRobotPose(current_joint_values_);
        }
    }
}