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
#include <QSizePolicy>
#include <QDebug>
#include <stdexcept>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
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
    
    // 设置DH参数和关节限制
    setupDHParameters();
    setupJointLimits();
    
    // 设置默认的立体视图模式
    if (!nh_.hasParam("/stereo_camera/stereo_method")) {
        nh_.setParam("/stereo_camera/stereo_method", "anaglyph");
    }
    
    // 初始化ROS通信
    initializeROS();
    
    // 连接摄像头切换按钮
    QPushButton* cameraSwitchButton = findChild<QPushButton*>("cameraSwitchButton");
    if (cameraSwitchButton) {
        connect(cameraSwitchButton, &QPushButton::clicked, this, &ArmControlGUI::onCameraSwitchButtonClicked);
    }
    
    // 更新GUI以显示初始关节值
    updateGUIJointValues();
    
    // 记录初始化完成信息
    logMessage("机械臂控制GUI初始化完成");
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

// 事件过滤器实现，用于处理相机视图的鼠标事件和键盘事件
bool ArmControlGUI::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == ui->cameraView) {
        // 处理鼠标点击
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                // 处理相机视图点击
                onCameraViewClicked(mouseEvent->pos());
                return true;
            }
        }
        // 处理鼠标双击 - 在立体视图和普通视图之间切换
        else if (event->type() == QEvent::MouseButtonDblClick) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                // 获取当前值
                std::string current_method;
                if (nh_.hasParam("/stereo_camera/stereo_method")) {
                    nh_.getParam("/stereo_camera/stereo_method", current_method);
                } else {
                    current_method = "anaglyph"; // 默认值
                }
                
                // 切换立体视图模式
                std::string new_method = (current_method == "anaglyph") ? "normal" : "anaglyph";
                nh_.setParam("/stereo_camera/stereo_method", new_method);
                
                // 记录日志
                logMessage(QString("切换立体视图模式: %1").arg(QString::fromStdString(new_method)));
                return true;
            }
        }
    }
    
    // 处理全局键盘事件
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
        // 按下's'键切换立体视图模式
        if (keyEvent->key() == Qt::Key_S) {
            // 获取当前值
            std::string current_method;
            if (nh_.hasParam("/stereo_camera/stereo_method")) {
                nh_.getParam("/stereo_camera/stereo_method", current_method);
            } else {
                current_method = "anaglyph"; // 默认值
            }
            
            // 切换立体视图模式
            std::string new_method = (current_method == "anaglyph") ? "normal" : "anaglyph";
            nh_.setParam("/stereo_camera/stereo_method", new_method);
            
            // 记录日志
            logMessage(QString("按键切换立体视图模式: %1").arg(QString::fromStdString(new_method)));
            return true;
        }
    }
    
    return QMainWindow::eventFilter(watched, event);
}

// 初始化函数实现
void ArmControlGUI::initializeGUI()
{
    // 设置窗口标题
    this->setWindowTitle("机械臂控制面板");
    
    // 设置界面样式
    QFile styleFile(":/qss/style.qss");
    if (styleFile.open(QFile::ReadOnly)) {
        QString styleSheet = QLatin1String(styleFile.readAll());
        this->setStyleSheet(styleSheet);
    }
    
    // 初始化UI元素
    setupUi();
    
    // 创建菜单
    createMenus();
    
    // 连接信号槽
    connectSignalSlots();
    
    // 初始化状态栏
    ui->statusbar->showMessage("系统准备就绪", 5000);
    
    // 初始化日志区域
    ui->logTextEdit->append("GUI初始化完成");
    
    // 确保调用initializeMembers来初始化DH参数和关节限制
    initializeMembers();
    
    // 设置末端位置控制器的范围
    if (ui->pos_x && ui->pos_y && ui->pos_z) {
        // 设置X轴范围
        ui->pos_x->setRange(-50.0, 50.0);
        ui->pos_x->setValue(20.0);
        
        // 设置Y轴范围
        ui->pos_y->setRange(-50.0, 50.0);
        ui->pos_y->setValue(0.0);
        
        // 设置Z轴范围
        ui->pos_z->setRange(0.0, 50.0);
        ui->pos_z->setValue(15.0);
        
        logMessage("末端位置控制器已配置");
    } else {
        logMessage("警告：未找到末端位置控制器组件");
    }
    
    // 计算并显示初始末端位置
    if (!current_joint_values_.empty()) {
        geometry_msgs::Pose initial_pose = forwardKinematics(current_joint_values_);
        double x_cm = initial_pose.position.x * 100.0;
        double y_cm = initial_pose.position.y * 100.0;
        double z_cm = initial_pose.position.z * 100.0;
        
        updateEndEffectorPosition(x_cm, y_cm, z_cm);
        logMessage(QString("初始末端位置: X=%1cm Y=%2cm Z=%3cm").arg(x_cm).arg(y_cm).arg(z_cm));
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
    
    // 添加自定义按钮连接 - 用于处理原来使用自动连接的按钮
    QPushButton* emergencyStopBtn = ui->centralwidget->findChild<QPushButton*>("emergencyStopBtn");
    if (emergencyStopBtn) {
        connect(emergencyStopBtn, &QPushButton::clicked, this, [this]() {
            // 发送停止命令
            logMessage("紧急停止");
            
            // 通过向ROS发布停止命令来停止机械臂
            std_msgs::String stop_cmd;
            stop_cmd.data = "stop";
            arm_command_pub_.publish(stop_cmd);
        });
    }
    
}

void ArmControlGUI::initializeROS()
{
    // 设置发布器 - 关节控制
    joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm1/joint_command", 10);
    vacuum_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/arm1/vacuum_command", 10);
    vacuum_power_pub_ = nh_.advertise<std_msgs::Float64>("/arm1/vacuum_power", 10);
    arm_command_pub_ = nh_.advertise<std_msgs::String>("/arm_commands", 10);
    
    // 设置继电器控制发布器
    relay_order_pub_ = nh_.advertise<std_msgs::String>("RelayOrder", 10);
    
    // 设置摄像头视图模式切换发布器
    camera_view_mode_pub_ = nh_.advertise<std_msgs::Int32>("/stereo_camera/view_mode", 10);
    
    // 订阅关节状态
    joint_state_sub_ = nh_.subscribe("/arm1/joint_states", 10, &ArmControlGUI::jointStateCallback, this);
    
    // 订阅当前视图图像（而不是合成立体图像）
    stereo_merged_sub_ = nh_.subscribe("/stereo_camera/current_view", 1, &ArmControlGUI::stereoMergedCallback, this);
    
    // 订阅深度图像
    depth_image_sub_ = nh_.subscribe("/stereo_camera/depth", 1, &ArmControlGUI::depthImageCallback, this);
    
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
    
    // 设置YOLO控制客户端
    yolo_control_client_ = nh_.serviceClient<std_srvs::SetBool>("/yolo_detector/enable");
    
    // 记录日志
    logMessage("ROS通信初始化完成");
}

void ArmControlGUI::initializeOpenGL()
{
    qDebug() << "开始初始化3D渲染环境...";
    
    // 检查是否已经创建了scene_3d_renderer_
    if (scene_3d_renderer_) {
        qDebug() << "警告: 3D渲染器已存在，先删除旧实例";
        delete scene_3d_renderer_;
        scene_3d_renderer_ = nullptr;
    }
    
    // 检查openGLView是否有效
    if (!ui->openGLView) {
        qDebug() << "错误: openGLView为空";
        return;
    }
    
    // 确保QWidget有合适的尺寸
    ui->openGLView->setMinimumSize(200, 200);
    
    // 创建新的布局（如果已有布局则先清除）
    QLayout* oldLayout = ui->openGLView->layout();
    if (oldLayout) {
        qDebug() << "移除旧布局";
        delete oldLayout;
    }
    
    // 创建新布局
    QHBoxLayout* layout = new QHBoxLayout(ui->openGLView);
    layout->setContentsMargins(0, 0, 0, 0);
    ui->openGLView->setLayout(layout);
    
    try {
        // 创建3D渲染器实例
        scene_3d_renderer_ = new Scene3DRenderer(ui->openGLView);
        if (!scene_3d_renderer_) {
            qDebug() << "错误: 无法创建3D渲染器实例";
            return;
        }
        
        // 添加到布局
        layout->addWidget(scene_3d_renderer_);
        
        // 连接信号
        connect(scene_3d_renderer_, &Scene3DRenderer::objectSelected, 
                this, &ArmControlGUI::on3DViewObjectSelected);
        
        // 更新场景
        updateScene3D();
        
        qDebug() << "3D渲染初始化成功完成";
    } catch (const std::exception& e) {
        qDebug() << "3D渲染初始化异常:" << e.what();
    } catch (...) {
        qDebug() << "3D渲染初始化未知异常";
    }
    
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
        qDebug() << "updateScene3D: 3D渲染器未初始化";
        return;
    }
    
    try {
        // 更新场景中的物体
        updateSceneObjects();
        
        // 更新机械臂姿态
        scene_3d_renderer_->setRobotPose(current_joint_values_);
        
        // 更新选中的物体
        scene_3d_renderer_->setSelectedObject(selected_object_index_);
    } catch (const std::exception& e) {
        qDebug() << "updateScene3D异常:" << e.what();
    } catch (...) {
        qDebug() << "updateScene3D未知异常";
    }
}

// 更新场景中的物体
void ArmControlGUI::updateSceneObjects()
{
    // 清空场景中的物体
    scene_objects_.clear();
    
    try {
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
        } else {
            qDebug() << "updateSceneObjects: 3D渲染器为空";
        }
    } catch (const std::exception& e) {
        qDebug() << "updateSceneObjects异常:" << e.what();
    } catch (...) {
        qDebug() << "updateSceneObjects未知异常";
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
    // 阻止UI自动更新导致回弹
    ui->joint1_slider->blockSignals(true);
    ui->joint1_spin->blockSignals(true);
    
    // 更新UI值
    ui->joint1_spin->setValue(value);
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[0] = value * M_PI / 180.0; // 转换为弧度
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint1_slider->blockSignals(false);
    ui->joint1_spin->blockSignals(false);
}

void ArmControlGUI::onJoint2SliderChanged(int value)
{
    // 阻止UI自动更新导致回弹
    ui->joint2_slider->blockSignals(true);
    ui->joint2_spin->blockSignals(true);
    
    // 更新UI值
    ui->joint2_spin->setValue(value);
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[1] = value;
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint2_slider->blockSignals(false);
    ui->joint2_spin->blockSignals(false);
}

void ArmControlGUI::onJoint3SliderChanged(int value)
{
    // 阻止UI自动更新导致回弹
    ui->joint3_slider->blockSignals(true);
    ui->joint3_spin->blockSignals(true);
    
    // 更新UI值
    ui->joint3_spin->setValue(value);
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[2] = value * M_PI / 180.0; // 转换为弧度
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint3_slider->blockSignals(false);
    ui->joint3_spin->blockSignals(false);
}

void ArmControlGUI::onJoint4SliderChanged(int value)
{
    // 阻止UI自动更新导致回弹
    ui->joint4_slider->blockSignals(true);
    ui->joint4_spin->blockSignals(true);
    
    // 更新UI值
    ui->joint4_spin->setValue(value);
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[3] = value * M_PI / 180.0; // 转换为弧度
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint4_slider->blockSignals(false);
    ui->joint4_spin->blockSignals(false);
}

void ArmControlGUI::onJoint6SliderChanged(int value)
{
    // 阻止UI自动更新导致回弹
    ui->joint6_slider->blockSignals(true);
    ui->joint6_spin->blockSignals(true);
    
    // 更新UI值
    ui->joint6_spin->setValue(value);
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[5] = value;
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint6_slider->blockSignals(false);
    ui->joint6_spin->blockSignals(false);
}

void ArmControlGUI::onJoint1SpinChanged(double value)
{
    // 阻止UI自动更新导致回弹
    ui->joint1_slider->blockSignals(true);
    ui->joint1_spin->blockSignals(true);
    
    // 更新滑块值
    ui->joint1_slider->setValue(static_cast<int>(value));
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[0] = value * M_PI / 180.0; // 转换为弧度
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint1_slider->blockSignals(false);
    ui->joint1_spin->blockSignals(false);
}

void ArmControlGUI::onJoint2SpinChanged(double value)
{
    // 阻止UI自动更新导致回弹
    ui->joint2_slider->blockSignals(true);
    ui->joint2_spin->blockSignals(true);
    
    // 更新滑块值
    ui->joint2_slider->setValue(static_cast<int>(value));
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[1] = value;
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint2_slider->blockSignals(false);
    ui->joint2_spin->blockSignals(false);
}

void ArmControlGUI::onJoint3SpinChanged(double value)
{
    // 阻止UI自动更新导致回弹
    ui->joint3_slider->blockSignals(true);
    ui->joint3_spin->blockSignals(true);
    
    // 更新滑块值
    ui->joint3_slider->setValue(static_cast<int>(value));
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[2] = value * M_PI / 180.0; // 转换为弧度
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint3_slider->blockSignals(false);
    ui->joint3_spin->blockSignals(false);
}

void ArmControlGUI::onJoint4SpinChanged(double value)
{
    // 阻止UI自动更新导致回弹
    ui->joint4_slider->blockSignals(true);
    ui->joint4_spin->blockSignals(true);
    
    // 更新滑块值
    ui->joint4_slider->setValue(static_cast<int>(value));
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[3] = value * M_PI / 180.0; // 转换为弧度
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint4_slider->blockSignals(false);
    ui->joint4_spin->blockSignals(false);
}

void ArmControlGUI::onJoint6SpinChanged(double value)
{
    // 阻止UI自动更新导致回弹
    ui->joint6_slider->blockSignals(true);
    ui->joint6_spin->blockSignals(true);
    
    // 更新滑块值
    ui->joint6_slider->setValue(static_cast<int>(value));
    
    // 设置关节值
    std::vector<double> joint_values = current_joint_values_;
    joint_values[5] = value;
    
    // 发送统一的关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 恢复信号
    ui->joint6_slider->blockSignals(false);
    ui->joint6_spin->blockSignals(false);
}

// 末端执行器控制槽实现
void ArmControlGUI::onMoveToPositionClicked()
{
    // 使用findChild查找坐标输入控件
    QDoubleSpinBox* xSpin = ui->centralwidget->findChild<QDoubleSpinBox*>("xSpinBox");
    QDoubleSpinBox* ySpin = ui->centralwidget->findChild<QDoubleSpinBox*>("ySpinBox");
    QDoubleSpinBox* zSpin = ui->centralwidget->findChild<QDoubleSpinBox*>("zSpinBox");
    
    // 如果找不到控件，使用默认值
    double x = xSpin ? xSpin->value() : 0.0;
    double y = ySpin ? ySpin->value() : 0.0;
    double z = zSpin ? zSpin->value() : 10.0;
    
    // 记录操作
    logMessage(QString("移动到坐标: (%1, %2, %3)").arg(x).arg(y).arg(z));
    
    // 创建目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x / 100.0; // 转换为米
    target_pose.position.y = y / 100.0;
    target_pose.position.z = z / 100.0;
    target_pose.orientation.w = 1.0; // 简单的默认朝向
    
    // 使用逆运动学计算对应的关节值
    std::vector<double> joint_values = poseToJoints(target_pose);
    
    // 发送关节控制命令
    sendJointCommand(joint_values);
    
    // 等待命令执行完成
    QApplication::processEvents();
    
    // 更新当前末端位置
    current_end_position_ = QVector3D(x, y, z);
    updateEndEffectorPose();
}

void ArmControlGUI::onHomeButtonClicked()
{
    logMessage("移动到初始位置");
    
    // 设置初始位置的关节值
    std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0, M_PI/2.0, 5.0};
    
    // 发送关节控制命令
    sendJointCommand(home_position);
    
    // 等待命令执行完成
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
    // 减少日志输出频率，添加静态计数器
    static int log_counter = 0;
    if (log_counter++ % 100 == 0) { // 每100次消息才记录一次日志
        logMessage(QString("收到检测图像: 尺寸=%1x%2, 通道=%3").arg(msg->width).arg(msg->height).arg(msg->encoding == "rgb8" ? 3 : 1));
    }
    
    // 将ROS图像消息转换为OpenCV图像
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        
        // 转换为QImage用于显示
        current_camera_image_ = cvMatToQImage(cv_ptr->image);
        
        // 成功接收到图像，重置错误计数器
        stereo_camera_error_count_ = 0;
        is_camera_available_ = true;
        
        // 在主线程中更新UI
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
    }
    catch (cv_bridge::Exception& e)
    {
        handleCameraError("检测图像回调中的cv_bridge异常: " + std::string(e.what()));
    }
    catch (std::exception& e)
    {
        handleCameraError("检测图像回调中的标准异常: " + std::string(e.what()));
    }
    catch (...)
    {
        handleCameraError("检测图像回调中的未知异常");
    }
}

void ArmControlGUI::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // 收到检测位姿消息
    int num_objects = msg->poses.size();
    // 减少日志输出频率，添加静态计数器
    static int log_counter = 0;
    if (log_counter++ % 100 == 0) { // 每100次消息才记录一次日志
        logMessage(QString("收到检测位姿消息: %1 个物体, frame_id=%2").arg(num_objects).arg(QString::fromStdString(msg->header.frame_id)));
    }
    
    if (num_objects == 0)
    {
        // 如果没有检测到物体，清空检测列表
        if (!detected_objects_.empty()) {
            // 仅当之前有对象而现在没有时才记录并清空
            if (log_counter % 100 == 0) {
                logMessage("检测位姿为空，清空表格");
            }
            detected_objects_.clear();
            QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
        }
        return;
    }
    
    // 处理检测到的每个物体
    // 首先检查是否需要更新列表，避免不必要的清空和重建
    bool need_update = false;
    
    // 检测结果数量不同时需要更新
    if (detected_objects_.size() != msg->poses.size()) {
        need_update = true;
    }
    
    // 只有需要更新时才清空旧的检测结果
    if (need_update) {
        detected_objects_.clear();
        
        // 处理检测到的每个物体
        for (size_t i = 0; i < msg->poses.size(); ++i)
        {
            DetectedObject obj;
            obj.id = "obj_" + std::to_string(i+1);
            obj.type = "未知物体";
            
            // 记录位姿信息
            obj.pose = msg->poses[i];
            
            // 转换位置到厘米单位显示
            obj.x = obj.pose.position.x * 100.0; // 转换为厘米
            obj.y = obj.pose.position.y * 100.0; // 转换为厘米
            obj.z = obj.pose.position.z * 100.0; // 转换为厘米
            
            // 添加到检测列表
            detected_objects_.push_back(obj);
        }
        
        // 在主线程中更新检测表格
        QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
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
        
        // 添加节流逻辑，只在位置明显变化时才更新UI
        static double last_x = 0.0, last_y = 0.0, last_z = 0.0;
        double x_cm = pose.position.x * 100.0;
        double y_cm = pose.position.y * 100.0;
        double z_cm = pose.position.z * 100.0;
        
        if (fabs(x_cm - last_x) > 1.0 || fabs(y_cm - last_y) > 1.0 || fabs(z_cm - last_z) > 1.0) {
            // 更新位置信息到UI标签，而不是记录到日志
            updateEndEffectorPosition(x_cm, y_cm, z_cm);
            
            last_x = x_cm;
            last_y = y_cm;
            last_z = z_cm;
        }
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
    // 根据当前视图模式选择要显示的图像
    QImage* sourceImage = nullptr;
    
    switch(camera_view_mode_) {
        case 0: // 左图
            sourceImage = &left_camera_image_;
            break;
        case 1: // 右图
            if (!right_camera_image_.isNull()) {
                sourceImage = &right_camera_image_;
            } else {
                // 如果右图不可用，使用左图
                sourceImage = &left_camera_image_;
            }
            break;
        case 2: // 深度图
        if (!current_depth_image_.isNull()) {
            sourceImage = &current_depth_image_;
        } else {
                // 如果深度图不可用，使用左图
                sourceImage = &left_camera_image_;
            }
            break;
        default:
            sourceImage = &left_camera_image_;
    }
    
    // 如果没有图像，跳过
    if (sourceImage == nullptr || sourceImage->isNull()) {
        // 如果没有图像但需要显示UI，则创建占位图像
        if (!is_camera_available_) {
            createPlaceholderImage();
        }
        return;
    }
    
    // 获取当前相机视图组件的大小
    QSize viewSize = ui->cameraView->size();
    
    // 改进缩放逻辑，确保图像合理填充视图区域，但不会被过度拉伸
    // 这里使用Qt::KeepAspectRatio，确保维持长宽比
    QImage scaledImage = sourceImage->scaled(viewSize.width() * 0.95, viewSize.height() * 0.95, 
                                            Qt::KeepAspectRatio, Qt::SmoothTransformation);
    
    // 创建一个带有标记的副本
    QImage displayImage = scaledImage.copy();
    QPainter painter(&displayImage);
    
    // 显示摄像头状态信息
    if (!is_camera_available_) {
        // 如果摄像头不可用，添加状态信息
        painter.setPen(QPen(Qt::red, 2));
        painter.setFont(QFont("Arial", 14, QFont::Bold));
        painter.drawText(10, 30, "摄像头不可用");
    } else if (camera_view_mode_ != 2) { // 只在非深度图模式下显示物体标记
        // 摄像头可用时，添加检测到的物体标记
        for (size_t i = 0; i < detected_objects_.size(); i++) {
            // 将3D世界坐标转为图像坐标
            QVector3D objPos(detected_objects_[i].x, detected_objects_[i].y, detected_objects_[i].z);
            QPoint imgPos = point3DToImage(objPos);
            
            // 调整为当前显示比例
            float scaleX = static_cast<float>(scaledImage.width()) / sourceImage->width();
            float scaleY = static_cast<float>(scaledImage.height()) / sourceImage->height();
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
    }
    
    // 添加视图模式标签
    painter.setPen(QPen(Qt::white, 2));
    painter.setFont(QFont("Arial", 12, QFont::Bold));
    QString modeText;
    switch(camera_view_mode_) {
        case 0: modeText = "左摄像头视图"; break;
        case 1: modeText = "右摄像头视图"; break;
        case 2: modeText = "深度图视图"; break;
        default: modeText = "未知视图"; break;
    }
    painter.drawText(10, displayImage.height() - 10, modeText);
    
    // 设置图像到相机视图
    ui->cameraView->setPixmap(QPixmap::fromImage(displayImage));
    
    // 立即处理事件，避免UI卡顿和闪烁
    QApplication::processEvents();
    
    // 如果有检测标签和视图，更新它们
    QLabel* detectionStatusLabel = ui->centralwidget->findChild<QLabel*>("detectionStatusLabel");
    QLabel* detectionView = ui->centralwidget->findChild<QLabel*>("detectionView");
    
    if (detectionStatusLabel && detectionView) {
        // 更新状态文本
        QString status;
        if (!is_camera_available_) {
            status = "摄像头不可用，尝试重连中...";
        } else {
            status = QString("检测到 %1 个物体").arg(detected_objects_.size());
        }
        detectionStatusLabel->setText(status);
        
        // 如果有检测图像，显示它 - 调整其尺寸以合理填充视图
        if (!left_camera_image_.isNull()) {
            // 获取检测视图的大小
            QSize detectionViewSize = detectionView->size();
            // 确保图像按比例缩放，合理填充检测视图区域
            detectionView->setPixmap(QPixmap::fromImage(left_camera_image_.scaled(
                detectionViewSize.width() * 0.95, detectionViewSize.height() * 0.95,
                Qt::KeepAspectRatio, Qt::SmoothTransformation)));
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
    
    // 存储当前关节值
    current_joint_values_ = joint_values;
    
    // 计算末端执行器位置（简化版本，实际应使用完整的正向运动学）
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
    
    // 仅在值明显变化时更新位置显示（避免频繁更新）
    static double last_x = 0.0, last_y = 0.0, last_z = 0.0;
    if (fabs(x - last_x) > 1.0 || fabs(y - last_y) > 1.0 || fabs(z - last_z) > 1.0) {
        // 更新位置标签，而不是记录到日志
        updateEndEffectorPosition(x, y, z);
        
        last_x = x;
        last_y = y;
        last_z = z;
    }
    
    // 更新3D视图
    if (scene_3d_renderer_) {
        scene_3d_renderer_->setRobotPose(current_joint_values_);
    }
    
    // 记录日志
    QString jointStr = "";
    for (size_t i = 0; i < joint_values.size(); i++) {
        if (i > 0) jointStr += ", ";
        jointStr += QString::number(joint_values[i], 'f', 2);
    }
    logMessage(QString("发送关节命令: [%1]").arg(jointStr));
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
    
    // 发布吸附命令和功率设置
    vacuum_cmd_pub_.publish(vacuum_cmd);
    vacuum_power_pub_.publish(power_cmd);
    
    // 更新状态
    vacuum_on_ = on;
    vacuum_power_ = power;
}

void ArmControlGUI::sendPickCommand(const std::string& object_id)
{
    logMessage(QString("准备抓取对象: %1").arg(QString::fromStdString(object_id)));
    
    // 查找要抓取的物体
    geometry_msgs::Pose target_pose;
    bool found = false;
    
    for (const auto& obj : detected_objects_)
    {
        if (obj.id == object_id)
        {
            target_pose = obj.pose;
            found = true;
            
            // 转换检测到的物体位置到机械臂坐标系(单位厘米)
            double x = obj.x;
            double y = obj.y;
            double z = obj.z;
            
            logMessage(QString("物体位置: X=%1, Y=%2, Z=%3").arg(x).arg(y).arg(z));
            
            // 1. 计算物体上方位置的关节值（预抓取位置）
            geometry_msgs::Pose pre_grasp_pose = target_pose;
            pre_grasp_pose.position.z += 0.05; // 高出物体5cm
            
            std::vector<double> pre_grasp_joints = poseToJoints(pre_grasp_pose);
            
            // 2. 移动到预抓取位置
            logMessage("步骤1: 移动到预抓取位置");
            sendJointCommand(pre_grasp_joints);
            QApplication::processEvents();
            
            // 3. 计算抓取位置的关节值
            std::vector<double> grasp_joints = poseToJoints(target_pose);
            
            // 4. 移动到抓取位置
            logMessage("步骤2: 移动到抓取位置");
            sendJointCommand(grasp_joints);
            QApplication::processEvents();
            
            // 5. 打开吸盘
            logMessage("步骤3: 开启吸盘");
            sendVacuumCommand(true, vacuum_power_);
            QApplication::processEvents();
            ros::Duration(0.5).sleep(); // 等待吸取
            
            // 6. 抬升物体（回到预抓取位置）
            logMessage("步骤4: 抬升物体");
            sendJointCommand(pre_grasp_joints);
            QApplication::processEvents();
            
            break;
        }
    }
    
    if (!found)
    {
        logMessage("找不到指定ID的物体！");
        return;
    }
    
    // 更新当前位置状态
    updateJointInfo();
}

// 发送放置命令到指定位置
void ArmControlGUI::sendPlaceCommand(double x, double y, double z)
{
    logMessage(QString("准备移动到位置：X=%1 Y=%2 Z=%3").arg(x).arg(y).arg(z));
    
    // 创建目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x / 100.0; // 转换为米
    target_pose.position.y = y / 100.0;
    target_pose.position.z = z / 100.0;
    
    // 设置方向（默认朝下，吸盘朝下方向）
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;
    
    try {
        // 使用逆向运动学计算目标关节角度
        std::vector<double> target_joints = inverseKinematics(target_pose);
        
        // 检查计算结果
        if (target_joints.size() != joint_limits_.size()) {
            logMessage("错误：逆向运动学计算失败，无法移动到目标位置");
            return;
        }
        
        // 检查关节限制
        if (!checkJointLimits(target_joints)) {
            logMessage("错误：目标位置超出机械臂工作范围，请选择其他位置");
            return;
        }
        
        // 向机械臂发送关节命令
        sendJointCommand(target_joints);
        
        // 更新目标位置显示
        updateEndEffectorPosition(x, y, z);
        
        // 更新场景3D视图
        if (scene_3d_renderer_) {
            scene_3d_renderer_->setRobotPose(target_joints);
        }
        
        logMessage(QString("机械臂正在移动到位置：X=%1 Y=%2 Z=%3").arg(x).arg(y).arg(z));
    } catch (const std::exception& e) {
        logMessage(QString("错误：计算目标位置的关节角度时出现异常：%1").arg(e.what()));
    } catch (...) {
        logMessage("错误：计算目标位置的关节角度时出现未知异常");
    }
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
    return inverseKinematics(pose);
}

geometry_msgs::Pose ArmControlGUI::jointsToPos(const std::vector<double>& joint_values)
{
    return forwardKinematics(joint_values);
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

// 电机控制函数已移除，统一使用关节控制

void ArmControlGUI::sendRelayOrder(const std::string& command)
{
    // 创建继电器控制消息
    std_msgs::String msg;
    msg.data = command;
    
    // 发布到正确的话题
    relay_order_pub_.publish(msg);
    
    // 记录操作
    logMessage(QString("发送继电器命令: %1 (吸附=%2, 螺丝=%3)")
              .arg(command.c_str())
              .arg(command[0] == '1' ? "开" : "关")
              .arg(command[1] == '1' ? "开" : "关"));
    
    // 等待命令执行完成
    ros::Duration(0.01).sleep();
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

// 路径规划相关的槽函数已移除

// 新增统一的物体检测回调函数，处理YOLO格式的检测结果
void ArmControlGUI::objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg, 
                                         const geometry_msgs::PoseArray::ConstPtr& poses_msg)
{
    // 减少日志输出频率，添加静态计数器
    static int log_counter = 0;
    if (log_counter++ % 100 == 0) { // 每100次消息才记录一次日志
        logMessage(QString("收到同步检测数据: 图像=%1x%2, 位姿=%3个").arg(img_msg->width).arg(img_msg->height).arg(poses_msg->poses.size()));
    }
    
    // 处理图像数据
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // 转换为QImage
    current_camera_image_ = cvMatToQImage(cv_ptr->image);
    
    // 处理位姿数据
    if (log_counter % 100 == 0) {
        logMessage("处理物体检测结果");
    }
    
    // 检查是否需要更新检测列表
    bool need_update = false;
    
    // 检测结果数量不同时需要更新
    if (detected_objects_.size() != poses_msg->poses.size()) {
        need_update = true;
    }
    
    // 只有需要更新时才清空旧的检测结果
    if (need_update) {
        detected_objects_.clear();
        
        for (size_t i = 0; i < poses_msg->poses.size(); ++i)
        {
            DetectedObject obj;
            obj.id = "obj_" + std::to_string(i+1);
            obj.type = "未知物体";
            
            // 记录位姿信息
            obj.pose = poses_msg->poses[i];
            
            // 转换位置到厘米单位显示
            obj.x = obj.pose.position.x * 100.0; // 转换为厘米
            obj.y = obj.pose.position.y * 100.0; // 转换为厘米
            obj.z = obj.pose.position.z * 100.0; // 转换为厘米
            
            // 添加到检测列表
            detected_objects_.push_back(obj);
        }
        
        if (log_counter % 100 == 0) {
            logMessage(QString("同步检测到 %1 个物体，准备更新UI").arg(detected_objects_.size()));
        }
        
        // 在主线程中更新UI
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
        QMetaObject::invokeMethod(this, "updateDetectionsTable", Qt::QueuedConnection);
    } else {
        // 即使不需要更新检测表，也要更新相机视图
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
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

// 夹持控制已统一集成到关节控制

// 舵机控制已统一集成到关节控制

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
        
        // 重置错误计数器（成功接收到图像）
        stereo_camera_error_count_ = 0;
        camera_reconnect_timer_.stop(); // 停止重连计时器
        
        // 处理双目图像 - 1280*480的图像分为左右两部分
        cv::Mat single_view;
        cv::Mat left_view;
        cv::Mat right_view;
        cv::Mat depth_map;  // 声明在外部作用域
        
        if (camera_image.cols == 1280 && camera_image.rows == 480) {
            // 这是一个双目图像，提取左右图像
            cv::Rect left_roi(0, 0, 640, 480);  // 左半部分
            cv::Rect right_roi(640, 0, 640, 480);  // 右半部分
            
            left_view = camera_image(left_roi).clone();
            right_view = camera_image(right_roi).clone();
            
            // 获取立体视图方法参数（红青立体图或普通视图）
            static bool use_anaglyph = true; // 默认使用立体图模式
            
            // 每15帧检查一次参数，避免频繁查询参数服务器
            static int frame_counter = 0;
            if (frame_counter++ % 15 == 0) {
                if (nh_.hasParam("/stereo_camera/stereo_method")) {
                    std::string method;
                    nh_.getParam("/stereo_camera/stereo_method", method);
                    use_anaglyph = (method == "anaglyph");
                }
            }
            
            if (use_anaglyph) {
                // 创建红青立体图(anaglyph)
                cv::Mat anaglyph(left_view.rows, left_view.cols, CV_8UC3);
                cv::Mat left_channels[3], right_channels[3], out_channels[3];
                
                // 分离通道
                cv::split(left_view, left_channels);
                cv::split(right_view, right_channels);
                
                // 创建红青立体图 - 左眼红色，右眼青色(蓝绿)
                out_channels[0] = right_channels[0]; // 右眼蓝色通道
                out_channels[1] = right_channels[1]; // 右眼绿色通道
                out_channels[2] = left_channels[2];  // 左眼红色通道
                
                // 合并通道创建立体图
                cv::merge(out_channels, 3, anaglyph);
                
                // 使用立体图作为合成视图
                single_view = anaglyph;
                
                // 添加提示信息到图像上
                cv::putText(single_view, "立体视图模式 (需要红青眼镜)", 
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                           cv::Scalar(255, 255, 255), 2);
            } else {
                // 使用左眼视图作为普通视图
                single_view = left_view.clone();
                
                // 添加提示信息到图像上
                cv::putText(single_view, "普通视图模式", 
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                           cv::Scalar(0, 255, 0), 2);
            }
            
            // 生成伪彩色深度图
            
            // 将左右图像转换为灰度图
            cv::Mat left_gray, right_gray;
            cv::cvtColor(left_view, left_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_view, right_gray, cv::COLOR_BGR2GRAY);
            
            // 计算视差图
            cv::Mat disparity;
            cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(0, 64, 11);
            stereo->compute(left_gray, right_gray, disparity);
            
            // 归一化视差图到0-255范围
            cv::Mat disp8;
            disparity.convertTo(disp8, CV_8UC1, 255/(stereo->getNumDisparities()*16.));
            
            // 应用伪彩色映射
            cv::Mat colored_depth;
            cv::applyColorMap(disp8, colored_depth, cv::COLORMAP_JET);
            
            // 添加深度值指示器（颜色条）
            int bar_width = 20;
            int bar_height = colored_depth.rows;
            cv::Mat color_bar = cv::Mat(bar_height, bar_width, CV_8UC3);
            
            for (int y = 0; y < bar_height; y++) {
                // 从下到上，颜色从蓝到红
                float normalized_y = 1.0f - (float)y / bar_height;
                cv::Mat color;
                cv::Mat temp(1, 1, CV_8UC1, cv::Scalar(normalized_y * 255));
                cv::applyColorMap(temp, color, cv::COLORMAP_JET);
                cv::line(color_bar, cv::Point(0, y), cv::Point(bar_width-1, y), color.at<cv::Vec3b>(0, 0), 1);
            }
            
            // 在颜色条上添加刻度
            int num_ticks = 5;
            for (int i = 0; i < num_ticks; i++) {
                int y = i * (bar_height - 1) / (num_ticks - 1);
                cv::line(color_bar, cv::Point(0, y), cv::Point(bar_width/2, y), cv::Scalar(255, 255, 255), 1);
                
                // 添加深度值文本
                std::string text = cv::format("%.1f", (1.0 - i / (float)(num_ticks - 1)) * 5.0); // 假设最大深度为5米
                int font_face = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.4;
                int thickness = 1;
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
                cv::putText(color_bar, text, 
                          cv::Point(bar_width/2 + 2, y + text_size.height/2), 
                          font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
            }
            
            // 在彩色深度图右侧添加颜色条
            cv::Mat result(colored_depth.rows, colored_depth.cols + color_bar.cols, CV_8UC3);
            colored_depth.copyTo(result(cv::Rect(0, 0, colored_depth.cols, colored_depth.rows)));
            color_bar.copyTo(result(cv::Rect(colored_depth.cols, 0, color_bar.cols, color_bar.rows)));
            
            // 添加标题和说明
            cv::putText(result, "Depth Map (m)", 
                      cv::Point(10, 30), 
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            
            depth_map = result;
            
        } else {
            // 其他尺寸的图像，直接使用
            single_view = camera_image;
            left_view = camera_image;
        }
        
        // 记录成功接收的图像帧
        static int frame_count = 0;
        frame_count++;
        
        if (frame_count % 30 == 0) {  // 每30帧记录一次
            ROS_INFO("成功接收到立体合并图像：帧 %d (尺寸: %dx%d → 合成: %dx%d)", 
                   frame_count, camera_image.cols, camera_image.rows, 
                   single_view.cols, single_view.rows);
        }
        
        // 转换左视图为QImage
        QImage left_qimage;
        if (left_view.channels() == 3) {
            // BGR转RGB
            cv::Mat left_rgb;
            cv::cvtColor(left_view, left_rgb, cv::COLOR_BGR2RGB);
            left_qimage = QImage(left_rgb.data, left_rgb.cols, left_rgb.rows,
                              left_rgb.step, QImage::Format_RGB888).copy();
        } else if (left_view.channels() == 1) {
            // 灰度图
            left_qimage = QImage(left_view.data, left_view.cols, left_view.rows,
                              left_view.step, QImage::Format_Grayscale8).copy();
        }
        
        // 转换右视图为QImage（如果有）
        QImage right_qimage;
        if (!right_view.empty() && right_view.channels() == 3) {
            cv::Mat right_rgb;
            cv::cvtColor(right_view, right_rgb, cv::COLOR_BGR2RGB);
            right_qimage = QImage(right_rgb.data, right_rgb.cols, right_rgb.rows,
                               right_rgb.step, QImage::Format_RGB888).copy();
        }
        
        // 转换合成视图为QImage
        QImage merged_qimage;
        if (single_view.channels() == 3) {
            // BGR转RGB
            cv::Mat rgb;
            cv::cvtColor(single_view, rgb, cv::COLOR_BGR2RGB);
            merged_qimage = QImage(rgb.data, rgb.cols, rgb.rows,
                                rgb.step, QImage::Format_RGB888).copy();
        } else if (single_view.channels() == 1) {
            // 灰度图
            merged_qimage = QImage(single_view.data, single_view.cols, single_view.rows,
                                single_view.step, QImage::Format_Grayscale8).copy();
        }
        
        // 转换深度图为QImage
        QImage depth_qimage;
        if (!depth_map.empty() && depth_map.channels() == 3) {
            cv::Mat depth_rgb;
            cv::cvtColor(depth_map, depth_rgb, cv::COLOR_BGR2RGB);
            depth_qimage = QImage(depth_rgb.data, depth_rgb.cols, depth_rgb.rows,
                               depth_rgb.step, QImage::Format_RGB888).copy();
        }
        
        // 保存图像
        left_camera_image_ = left_qimage;
        right_camera_image_ = right_qimage;
        current_camera_image_ = merged_qimage;
        current_depth_image_ = depth_qimage;
        
        // 标记摄像头可用
        is_camera_available_ = true;
        
        // 更新UI (在主线程中)
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
    }
    catch (cv_bridge::Exception& e) {
        handleCameraError("立体图像回调中的cv_bridge异常: " + std::string(e.what()));
    }
    catch (std::exception& e) {
        handleCameraError("立体图像回调中的标准异常: " + std::string(e.what()));
    }
    catch (...) {
        handleCameraError("立体图像回调中的未知异常");
    }
}

// 处理摄像头错误的辅助函数
void ArmControlGUI::handleCameraError(const std::string& error_msg)
{
    ROS_ERROR("%s", error_msg.c_str());
    
    // 增加错误计数
    stereo_camera_error_count_++;
    
    // 如果错误持续，切换到占位图像
    if (stereo_camera_error_count_ > 5) {
        if (is_camera_available_) {
            logMessage("摄像头不可用，切换到占位图像");
            is_camera_available_ = false;
            
            // 创建占位图像
            createPlaceholderImage();
            
            // 启动重连计时器，如果没有在运行
            if (!camera_reconnect_timer_.isActive()) {
                camera_reconnect_timer_.start(5000); // 5秒尝试重连一次
            }
        }
    }
}

// 创建占位图像
void ArmControlGUI::createPlaceholderImage()
{
    // 创建一个占位图像 - 640x480 灰色背景
    QImage placeholder(640, 480, QImage::Format_RGB888);
    placeholder.fill(QColor(80, 80, 80)); // 深灰色背景
    
    // 添加文本说明
    QPainter painter(&placeholder);
    painter.setPen(QPen(Qt::white));
    painter.setFont(QFont("Arial", 20, QFont::Bold));
    
    // 居中显示文本
    painter.drawText(placeholder.rect(), Qt::AlignCenter, "摄像头不可用\n尝试重连中...");
    
    // 存储占位图像
    left_camera_image_ = placeholder;
    current_camera_image_ = placeholder;
    
    // 更新UI
    QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
}

// 摄像头视图切换按钮点击处理
void ArmControlGUI::onCameraSwitchButtonClicked()
{
    // 循环切换模式：左图(0) -> 右图(1) -> 深度图(2) -> 左图(0)
    camera_view_mode_ = (camera_view_mode_ + 1) % 3;
    
    // 更新标签显示
    QLabel* viewModeLabel = findChild<QLabel*>("cameraViewModeLabel");
    if (viewModeLabel) {
        QString modeName;
        switch(camera_view_mode_) {
            case 0: modeName = "左图"; break;
            case 1: modeName = "右图"; break;
            case 2: modeName = "深度图"; break;
            default: modeName = "未知"; break;
        }
        viewModeLabel->setText(QString("当前：%1").arg(modeName));
    }
    
    // 发布视图模式消息
    std_msgs::Int32 mode_msg;
    mode_msg.data = camera_view_mode_;
    camera_view_mode_pub_.publish(mode_msg);
    
    // 记录日志
    QString modeStr;
    switch(camera_view_mode_) {
        case 0: modeStr = "左图"; break;
        case 1: modeStr = "右图"; break;
        case 2: modeStr = "深度图"; break;
        default: modeStr = "未知模式"; break;
    }
    logMessage(QString("切换摄像头视图模式：%1").arg(modeStr));
}

// 尝试重新连接摄像头
void ArmControlGUI::attemptCameraReconnect()
{
    // 只在摄像头不可用时尝试重新连接
    if (!is_camera_available_) {
        stereo_camera_error_count_++;
        
        // 每5次尝试才输出日志，避免日志过多
        if (stereo_camera_error_count_ % 5 == 0) {
            logMessage(QString("尝试重新连接摄像头 (尝试次数: %1)").arg(stereo_camera_error_count_));
        }
        
        // 尝试查找可用摄像头
        bool camera_found = findAvailableCamera();
        
        if (camera_found) {
            // 重置错误计数器
            stereo_camera_error_count_ = 0;
            is_camera_available_ = true;
            
            // 重启摄像头节点
            std_msgs::String cmd;
            cmd.data = "restart_camera";
            arm_command_pub_.publish(cmd);
            
            logMessage(QString("摄像头已重新连接，索引: %1").arg(available_camera_index_));
        }
        
        // 如果错误次数过多，减少重新连接频率
        if (stereo_camera_error_count_ > 30) {
            // 超过30次尝试后，延长重连间隔
            if (!camera_reconnect_timer_.isActive() || camera_reconnect_timer_.interval() < 10000) {
                camera_reconnect_timer_.start(10000); // 10秒重连一次
            }
        } else if (stereo_camera_error_count_ > 10) {
            // 超过10次尝试后，延长重连间隔
            if (!camera_reconnect_timer_.isActive() || camera_reconnect_timer_.interval() < 5000) {
                camera_reconnect_timer_.start(5000); // 5秒重连一次
            }
        } else {
            // 前10次尝试，保持较高频率
            if (!camera_reconnect_timer_.isActive() || camera_reconnect_timer_.interval() != 2000) {
                camera_reconnect_timer_.start(2000); // 2秒重连一次
            }
        }
    } else {
        // 如果摄像头已可用，停止重连定时器
        if (camera_reconnect_timer_.isActive()) {
            camera_reconnect_timer_.stop();
        }
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
    // 初始化DH参数和关节限制
    setupDHParameters();
    setupJointLimits();
    
    // 机械臂状态初始化
    current_joint_values_ = {0.0, 0.0, 0.0, 0.0, M_PI/2, 5.0};
    target_joint_values_ = {0.0, 0.0, 0.0, 0.0, M_PI/2, 5.0};
    
    // 关节限制
    joint_min_values_ = {-M_PI, 0.0, -M_PI/2, 0.0, M_PI/2, 5.0};
    joint_max_values_ = {M_PI, 43.0, M_PI/2, M_PI, M_PI/2, 15.0};
    
    // 末端执行器状态（吸盘）
    vacuum_on_ = false;
    vacuum_power_ = 50;
    
    // 更新UI中的真空功率滑块初始值
    if (ui && ui->vacuumPowerSlider) {
        ui->vacuumPowerSlider->setValue(vacuum_power_);
        ui->vacuumPowerLabel->setText(QString("%1%").arg(vacuum_power_));
    }
    
    // 控制模式
    current_control_mode_ = ArmControlMode::JOINT_CONTROL;
    
    // 视觉相关
    visual_servo_active_ = false;
    selected_object_index_ = -1;
    
    // 摄像头错误处理相关初始化
    stereo_camera_error_count_ = 0;
    is_camera_available_ = false;  // 初始化为不可用，直到找到可用摄像头
    available_camera_index_ = 0;   // 默认尝试索引0
    
    // 尝试查找可用摄像头
    if (findAvailableCamera()) {
        is_camera_available_ = true;
        logMessage(QString("已找到可用摄像头，索引为: %1").arg(available_camera_index_));
    } else {
        logMessage("无可用摄像头，将使用占位图像");
        // 创建占位图像
        QTimer::singleShot(1000, this, &ArmControlGUI::createPlaceholderImage);
    }
    
    // 设置摄像头重连定时器
    camera_reconnect_timer_.setSingleShot(false);  // 循环触发
    connect(&camera_reconnect_timer_, &QTimer::timeout, this, &ArmControlGUI::attemptCameraReconnect);
    
    // 更新定时器
    updateTimer = new QTimer(this);
    updateTimer->setInterval(500); // 2Hz更新，降低刷新频率，减少刷屏问题
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
    vacuum_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/arm1/vacuum_command", 10);
    vacuum_power_pub_ = nh_.advertise<std_msgs::Float64>("/arm1/vacuum_power", 10);
    arm_command_pub_ = nh_.advertise<std_msgs::String>("/arm_commands", 10);
    
    // YOLO控制客户端
    yolo_control_client_ = nh_.serviceClient<std_srvs::SetBool>("/yolo_detector/enable");
    
    // 日志消息
    logMessage("已订阅ROS话题，等待数据...");
}

// 设置DH参数
void ArmControlGUI::setupDHParameters()
{
    // 清空现有的DH参数
    dh_params_.clear();
    
    // 设置DH参数，格式：[type, d, theta, a, alpha]
    // type: 0=revolute（旋转关节）, 1=prismatic（移动关节）
    // 参数来源于arm_params.yaml
    
    // 初始化机械臂参数（单位：cm）
    double a1 = 13.0; // 第一个连杆长度
    double a2 = 13.0; // 第二个连杆长度
    double a3 = 25.0; // 第三个连杆长度
    
    // 添加所有关节的DH参数
    // [type, d, theta, a, alpha]
    dh_params_.push_back(std::make_tuple(0, 0.0, 0.0, a1, M_PI/2));  // 底座旋转关节(θ1)
    dh_params_.push_back(std::make_tuple(1, 0.0, M_PI/4, 0.0, 0.0)); // 伸缩关节(d2)
    dh_params_.push_back(std::make_tuple(0, a2, 0.0, 0.0, M_PI/2));  // 肩部关节(θ3)
    dh_params_.push_back(std::make_tuple(0, 0.0, 0.0, a3, M_PI/2));  // 肘部关节(θ4)
    dh_params_.push_back(std::make_tuple(0, 0.0, M_PI/2, 0.0, M_PI/2)); // 固定关节(θ5)
    dh_params_.push_back(std::make_tuple(1, 0.0, M_PI/2, 0.0, 0.0));  // 末端伸缩(d6)
    
    logMessage("DH参数已设置");
}

// 设置关节限制
void ArmControlGUI::setupJointLimits()
{
    // 清空现有的关节限制
    joint_limits_.clear();
    
    // 设置关节限制，格式：[最小值, 最大值]
    // 参数来源于arm_params.yaml
    
    // joint1: θ1 (rad) ±180°
    joint_limits_.push_back(std::make_pair(-M_PI, M_PI));
    
    // joint2: d2 (cm) 0-43cm
    joint_limits_.push_back(std::make_pair(0.0, 43.0));
    
    // joint3: θ3 (rad) ±90°
    joint_limits_.push_back(std::make_pair(-M_PI/2, M_PI/2));
    
    // joint4: θ4 (rad) 0-180°
    joint_limits_.push_back(std::make_pair(0.0, M_PI));
    
    // joint5: θ5 (rad) 固定在90°
    joint_limits_.push_back(std::make_pair(M_PI/2, M_PI/2));
    
    // joint6: d6 (cm) 5-15cm
    joint_limits_.push_back(std::make_pair(5.0, 15.0));
    
    logMessage("关节限制已设置");
}

// 实现on_moveToPositionButton_clicked函数
void ArmControlGUI::on_moveToPositionButton_clicked()
{
    // 直接调用现有的onMoveToPositionClicked函数
    onMoveToPositionClicked();
}

// 尝试查找可用的摄像头索引
bool ArmControlGUI::findAvailableCamera()
{
    logMessage("正在扫描可用摄像头设备...");
    
    // 1. 先通过系统命令获取实际存在的视频设备
    FILE* fp;
    char path[512];
    std::vector<std::string> video_devices;
    
    // 执行ls命令获取视频设备
    fp = popen("ls /dev/video* 2>/dev/null", "r");
    if (fp == NULL) {
        logMessage("无法执行系统命令以查找视频设备");
        return false;
    }
    
    // 读取所有视频设备路径
    while (fgets(path, sizeof(path), fp) != NULL) {
        // 移除末尾的换行符
        path[strcspn(path, "\n")] = 0;
        video_devices.push_back(path);
        logMessage(QString("找到视频设备: %1").arg(path));
    }
    pclose(fp);
    
    if (video_devices.empty()) {
        logMessage("系统中未检测到任何视频设备");
        available_camera_index_ = -1;
        return false;
    }
    
    // 2. 遍历检测到的视频设备，尝试打开
    for (const auto& device : video_devices) {
        // 从设备路径中提取索引号，例如从/dev/video0提取0
        int device_index = -1;
        if (sscanf(device.c_str(), "/dev/video%d", &device_index) == 1) {
            logMessage(QString("尝试打开设备: %1 (索引: %2)").arg(device.c_str()).arg(device_index));
            
            // 检查设备是否被占用
            FILE* check_fp;
            char check_cmd[512];
            sprintf(check_cmd, "fuser %s 2>/dev/null", device.c_str());
            check_fp = popen(check_cmd, "r");
            if (check_fp == NULL) {
                logMessage("无法检查设备占用状态");
            } else {
                char pid_buf[128];
                if (fgets(pid_buf, sizeof(pid_buf), check_fp) != NULL) {
                    // 设备被占用，提取PID
                    pid_buf[strcspn(pid_buf, "\n")] = 0;
                    
                    // 获取占用进程的详细信息
                    FILE* proc_fp;
                    char proc_cmd[512];
                    sprintf(proc_cmd, "ps -p %s -o comm= 2>/dev/null", pid_buf);
                    proc_fp = popen(proc_cmd, "r");
                    
                    char proc_name[128] = "未知进程";
                    if (proc_fp != NULL) {
                        if (fgets(proc_name, sizeof(proc_name), proc_fp) != NULL) {
                            proc_name[strcspn(proc_name, "\n")] = 0;
                        }
                        pclose(proc_fp);
                    }
                    
                    logMessage(QString("设备 %1 被进程 %2 (%3) 占用").arg(device.c_str()).arg(pid_buf).arg(proc_name));
                    logMessage(QString("可执行 'kill %1' 命令终止占用进程").arg(pid_buf));
                    continue; // 跳过被占用的设备
                }
                pclose(check_fp);
            }
            
            // 尝试打开摄像头
            cv::VideoCapture cap(device_index);
            if (cap.isOpened()) {
                cv::Mat test_frame;
                bool read_success = cap.read(test_frame);
                cap.release();
                
                if (read_success && !test_frame.empty()) {
                    logMessage(QString("成功打开摄像头设备 %1 (索引: %2)").arg(device.c_str()).arg(device_index));
                    available_camera_index_ = device_index;
                    return true;
                } else {
                    logMessage(QString("设备 %1 可以打开但无法读取帧").arg(device.c_str()));
                }
            } else {
                logMessage(QString("无法打开设备 %1").arg(device.c_str()));
            }
        }
    }
    
    logMessage("未找到可用的摄像头设备");
    available_camera_index_ = -1;
    return false;
}

// 添加一个新的函数来更新末端执行器位置标签
void ArmControlGUI::updateEndEffectorPosition(double x, double y, double z)
{
    // 设置当前位置标签的文本
    ui->currentPosXValue->setText(QString("X: %1").arg(QString::number(x, 'f', 1)));
    ui->currentPosYValue->setText(QString("Y: %1").arg(QString::number(y, 'f', 1)));
    ui->currentPosZValue->setText(QString("Z: %1").arg(QString::number(z, 'f', 1)));
}

// 修改topicCallback_pose函数，使末端位置不再显示在日志中
void ArmControlGUI::topicCallback_pose(const geometry_msgs::Pose& pose)
{
    // 确保在UI线程中处理
    if (ros::ok() && !ui_processing_)
    {
        ui_processing_ = true;
        
        // 更新末端位置到UI
        current_end_pose_ = pose;
        
        // 更新相机变换矩阵
        updateCameraTransform(pose);
        
        // 添加节流逻辑，只在位置明显变化时才更新UI
        static double last_x = 0.0, last_y = 0.0, last_z = 0.0;
        double x_cm = pose.position.x * 100.0;
        double y_cm = pose.position.y * 100.0;
        double z_cm = pose.position.z * 100.0;
        
        if (fabs(x_cm - last_x) > 1.0 || fabs(y_cm - last_y) > 1.0 || fabs(z_cm - last_z) > 1.0) {
            // 更新当前位置到末端执行器控制面板的标签中（而不是记录到日志）
            updateEndEffectorPosition(x_cm, y_cm, z_cm);
            
            last_x = x_cm;
            last_y = y_cm;
            last_z = z_cm;
        }
        
        ui_processing_ = false;
    }
}

void ArmControlGUI::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 将ROS图像转换为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth_mat = cv_ptr->image;
        
        // 确保图像不为空
        if (depth_mat.empty()) {
            ROS_WARN("接收到的深度图为空");
            return;
        }

        // 将深度图转换为伪彩色显示
        cv::Mat normalized_depth;
        cv::Mat colored_depth;
        
        // 找出深度图的最小最大值，用于归一化
        double min_val, max_val;
        cv::minMaxLoc(depth_mat, &min_val, &max_val);
        
        // 如果最大值和最小值相等，设置一个默认范围避免除零错误
        if (max_val == min_val) {
            min_val = 0;
            max_val = 1.0;
        }
        
        // 归一化到0-1范围
        depth_mat.convertTo(normalized_depth, CV_32FC1, 1.0 / (max_val - min_val), -min_val / (max_val - min_val));
        
        // 转换为8位图像，范围0-255
        cv::Mat depth_8bit;
        normalized_depth.convertTo(depth_8bit, CV_8UC1, 255.0);
        
        // 应用伪彩色映射
        cv::applyColorMap(depth_8bit, colored_depth, cv::COLORMAP_JET);
        
        // 添加深度值指示器（颜色条）
        int bar_width = 20;
        int bar_height = colored_depth.rows;
        cv::Mat color_bar = cv::Mat(bar_height, bar_width, CV_8UC3);
        
        for (int y = 0; y < bar_height; y++) {
            // 从下到上，颜色从蓝到红
            float normalized_y = 1.0f - (float)y / bar_height;
            cv::Mat color;
            cv::Mat temp(1, 1, CV_8UC1, cv::Scalar(normalized_y * 255));
            cv::applyColorMap(temp, color, cv::COLORMAP_JET);
            cv::line(color_bar, cv::Point(0, y), cv::Point(bar_width-1, y), color.at<cv::Vec3b>(0, 0), 1);
        }
        
        // 在颜色条上添加刻度
        int num_ticks = 5;
        for (int i = 0; i < num_ticks; i++) {
            int y = i * (bar_height - 1) / (num_ticks - 1);
            float depth_value = max_val - i * (max_val - min_val) / (num_ticks - 1);
            cv::line(color_bar, cv::Point(0, y), cv::Point(bar_width/2, y), cv::Scalar(255, 255, 255), 1);
            
            // 添加深度值文本
            std::string text = cv::format("%.2f", depth_value);
            int font_face = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 0.4;
            int thickness = 1;
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
            cv::putText(color_bar, text, 
                      cv::Point(bar_width/2 + 2, y + text_size.height/2), 
                      font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
        }
        
        // 在彩色深度图右侧添加颜色条
        cv::Mat result(colored_depth.rows, colored_depth.cols + color_bar.cols, CV_8UC3);
        colored_depth.copyTo(result(cv::Rect(0, 0, colored_depth.cols, colored_depth.rows)));
        color_bar.copyTo(result(cv::Rect(colored_depth.cols, 0, color_bar.cols, color_bar.rows)));
        
        // 添加标题和说明
        cv::putText(result, "Depth Map (m)", 
                  cv::Point(10, 30), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        // 将结果转换为QImage
        current_depth_image_ = cvMatToQImage(result);
        
        // 更新相机视图（线程安全方式）
        QMetaObject::invokeMethod(this, "updateCameraViews", Qt::QueuedConnection);
    }
    catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge异常: %s", e.what());
    }
    catch (const std::exception& e) {
        ROS_ERROR("处理深度图像时出现异常: %s", e.what());
    }
}

// 计算单个关节的DH变换矩阵
QMatrix4x4 ArmControlGUI::computeDHTransform(double theta, double d, double a, double alpha)
{
    QMatrix4x4 transform;
    
    // 使用DH参数计算变换矩阵
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);
    
    transform(0, 0) = ct;      transform(0, 1) = -st*ca;  transform(0, 2) = st*sa;   transform(0, 3) = a*ct;
    transform(1, 0) = st;      transform(1, 1) = ct*ca;   transform(1, 2) = -ct*sa;  transform(1, 3) = a*st;
    transform(2, 0) = 0.0;     transform(2, 1) = sa;      transform(2, 2) = ca;      transform(2, 3) = d;
    transform(3, 0) = 0.0;     transform(3, 1) = 0.0;     transform(3, 2) = 0.0;     transform(3, 3) = 1.0;
    
    return transform;
}

// 使用DH参数计算机械臂正向运动学
geometry_msgs::Pose ArmControlGUI::forwardKinematics(const std::vector<double>& joint_values)
{
    // 检查关节数量是否正确
    if (joint_values.size() != dh_params_.size()) {
        logMessage(QString("错误：关节数量不匹配，期望 %1 个关节，实际提供 %2 个")
                .arg(dh_params_.size()).arg(joint_values.size()));
        
        // 返回一个默认姿态
        geometry_msgs::Pose default_pose;
        default_pose.position.x = 0.0;
        default_pose.position.y = 0.0;
        default_pose.position.z = 0.0;
        default_pose.orientation.w = 1.0;
        return default_pose;
    }
    
    // 初始化变换矩阵为单位矩阵
    QMatrix4x4 transform;
    transform.setToIdentity();
    
    // 累积每个关节的变换
    for (size_t i = 0; i < dh_params_.size(); ++i) {
        // 获取关节类型和DH参数
        int joint_type = std::get<0>(dh_params_[i]);
        double d = std::get<1>(dh_params_[i]);
        double theta = std::get<2>(dh_params_[i]);
        double a = std::get<3>(dh_params_[i]);
        double alpha = std::get<4>(dh_params_[i]);
        
        // 根据关节类型更新相应参数
        if (joint_type == 0) { // 旋转关节
            theta += joint_values[i]; // 更新θ
        } else if (joint_type == 1) { // 移动关节
            d += joint_values[i];    // 更新d
        }
        
        // 计算此关节的变换矩阵
        QMatrix4x4 joint_transform = computeDHTransform(theta, d, a, alpha);
        
        // 累积变换
        transform = transform * joint_transform;
    }
    
    // 从变换矩阵提取位姿
    geometry_msgs::Pose pose;
    
    // 设置位置（单位：米）
    pose.position.x = transform(0, 3) / 100.0; // 转换为米
    pose.position.y = transform(1, 3) / 100.0;
    pose.position.z = transform(2, 3) / 100.0;
    
    // 从旋转矩阵提取四元数
    QMatrix3x3 rotation_matrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation_matrix(i, j) = transform(i, j);
        }
    }
    QQuaternion quat = QQuaternion::fromRotationMatrix(rotation_matrix);
    
    // 设置方向
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.scalar();
    
    return pose;
}

// 检查关节是否在限制范围内
bool ArmControlGUI::checkJointLimits(const std::vector<double>& joint_values)
{
    // 检查关节数量是否正确
    if (joint_values.size() != joint_limits_.size()) {
        logMessage(QString("错误：关节数量不匹配，期望 %1 个关节，实际提供 %2 个")
                .arg(joint_limits_.size()).arg(joint_values.size()));
        return false;
    }
    
    // 检查每个关节是否在限制范围内
    for (size_t i = 0; i < joint_values.size(); ++i) {
        if (joint_values[i] < joint_limits_[i].first || joint_values[i] > joint_limits_[i].second) {
            return false;
        }
    }
    
    return true;
}

// 逆向运动学计算
std::vector<double> ArmControlGUI::inverseKinematics(const geometry_msgs::Pose& target_pose, const std::vector<double>& initial_guess)
{
    // 使用数值优化方法（梯度下降）求解逆运动学
    
    // 如果没有提供初始猜测值，使用当前关节角度
    std::vector<double> current_joints = initial_guess;
    if (current_joints.empty()) {
        current_joints = current_joint_values_;
    }
    
    // 确保初始猜测值符合关节限制
    for (size_t i = 0; i < current_joints.size(); ++i) {
        current_joints[i] = std::max(joint_limits_[i].first, 
                           std::min(joint_limits_[i].second, current_joints[i]));
    }
    
    // 设置优化参数
    const int max_iterations = 100;
    const double convergence_threshold = 0.001; // 米
    const double learning_rate = 0.1;
    
    // 目标位置（厘米）
    QVector3D target_position(
        target_pose.position.x * 100.0, 
        target_pose.position.y * 100.0, 
        target_pose.position.z * 100.0
    );
    
    // 目标姿态
    QQuaternion target_orientation(
        target_pose.orientation.w, 
        target_pose.orientation.x, 
        target_pose.orientation.y, 
        target_pose.orientation.z
    );
    
    // 迭代优化
    for (int iter = 0; iter < max_iterations; ++iter) {
        // 计算当前关节角度下的末端位姿
        geometry_msgs::Pose current_pose = forwardKinematics(current_joints);
        
        // 计算位置误差（厘米）
        QVector3D current_position(
            current_pose.position.x * 100.0, 
            current_pose.position.y * 100.0, 
            current_pose.position.z * 100.0
        );
        
        QVector3D position_error = target_position - current_position;
        
        // 计算姿态误差（简化，仅使用位置误差）
        
        // 检查是否收敛
        double error_magnitude = position_error.length();
        if (error_magnitude < convergence_threshold) {
            logMessage(QString("逆运动学已收敛，迭代次数: %1, 误差: %2 cm").arg(iter).arg(error_magnitude));
            return current_joints;
        }
        
        // 计算雅可比矩阵（数值微分）
        std::vector<std::vector<double>> jacobian(3, std::vector<double>(current_joints.size(), 0.0));
        
        const double delta = 0.01; // 数值微分的步长
        
        for (size_t j = 0; j < current_joints.size(); ++j) {
            // 原关节值
            double original_value = current_joints[j];
            
            // 微小扰动
            current_joints[j] += delta;
            
            // 计算扰动后的位姿
            geometry_msgs::Pose perturbed_pose = forwardKinematics(current_joints);
            
            // 恢复关节值
            current_joints[j] = original_value;
            
            // 计算偏导数
            jacobian[0][j] = (perturbed_pose.position.x * 100.0 - current_position.x()) / delta;
            jacobian[1][j] = (perturbed_pose.position.y * 100.0 - current_position.y()) / delta;
            jacobian[2][j] = (perturbed_pose.position.z * 100.0 - current_position.z()) / delta;
        }
        
        // 使用转置雅可比矩阵计算关节角度更新
        for (size_t j = 0; j < current_joints.size(); ++j) {
            double update = 0.0;
            for (size_t k = 0; k < 3; ++k) {
                update += jacobian[k][j] * position_error[k];
            }
            
            // 更新关节角度
            current_joints[j] += learning_rate * update;
            
            // 确保关节角度在限制范围内
            current_joints[j] = std::max(joint_limits_[j].first, 
                               std::min(joint_limits_[j].second, current_joints[j]));
        }
    }
    
    logMessage("逆运动学未收敛，返回最佳近似解");
    return current_joints;
}

// 初始化UI元素
void ArmControlGUI::setupUi()
{
    // 设置日志显示区
    ui->logTextEdit->setReadOnly(true);
    
    // 设置关节控制范围
    ui->joint1_slider->setRange(-180, 180);
    ui->joint2_slider->setRange(0, 50);
    ui->joint3_slider->setRange(-90, 90);
    ui->joint4_slider->setRange(0, 180);
    ui->joint6_slider->setRange(5, 15);
    
    ui->joint1_spin->setRange(-180.0, 180.0);
    ui->joint2_spin->setRange(0.0, 50.0);
    ui->joint3_spin->setRange(-90.0, 90.0);
    ui->joint4_spin->setRange(0.0, 180.0);
    ui->joint6_spin->setRange(5.0, 15.0);
    
    // 设置吸盘功率滑块范围
    ui->vacuumPowerSlider->setRange(0, 100);
    ui->vacuumPowerSlider->setValue(50);  // 使用默认值50%，与vacuum_power_保持一致
    ui->vacuumPowerLabel->setText(QString("%1%").arg(50));
    
    // 设置检测物体表格
    QStringList headers;
    headers << "ID" << "类型" << "X(cm)" << "Y(cm)" << "Z(cm)" << "操作";
    ui->detectionsTable->setColumnCount(headers.size());
    ui->detectionsTable->setHorizontalHeaderLabels(headers);
    ui->detectionsTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    
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
}

// 创建菜单
void ArmControlGUI::createMenus()
{
    // 文件菜单
    QMenu* fileMenu = menuBar()->addMenu("文件");
    
    // 打开任务序列
    QAction* openAction = new QAction("打开任务序列", this);
    connect(openAction, &QAction::triggered, this, &ArmControlGUI::onOpenTaskSequence);
    fileMenu->addAction(openAction);
    
    // 保存任务序列
    QAction* saveAction = new QAction("保存任务序列", this);
    connect(saveAction, &QAction::triggered, this, &ArmControlGUI::onSaveTaskSequence);
    fileMenu->addAction(saveAction);
    
    fileMenu->addSeparator();
    
    // 退出
    QAction* exitAction = new QAction("退出", this);
    connect(exitAction, &QAction::triggered, this, &ArmControlGUI::onExitApplication);
    fileMenu->addAction(exitAction);
    
    // 设置菜单
    QMenu* settingsMenu = menuBar()->addMenu("设置");
    
    // 机械臂设置
    QAction* robotSettingsAction = new QAction("机械臂设置", this);
    connect(robotSettingsAction, &QAction::triggered, this, &ArmControlGUI::onRobotSettings);
    settingsMenu->addAction(robotSettingsAction);
    
    // 帮助菜单
    QMenu* helpMenu = menuBar()->addMenu("帮助");
    
    // 关于
    QAction* aboutAction = new QAction("关于", this);
    connect(aboutAction, &QAction::triggered, this, &ArmControlGUI::onAbout);
    helpMenu->addAction(aboutAction);
}

// 连接信号槽
void ArmControlGUI::connectSignalSlots()
{
    // 设置关节控制连接
    connect(ui->joint1_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint1SliderChanged);
    connect(ui->joint2_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint2SliderChanged);
    connect(ui->joint3_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint3SliderChanged);
    connect(ui->joint4_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint4SliderChanged);
    connect(ui->joint6_slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint6SliderChanged);
    
    connect(ui->joint1_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint1SpinChanged);
    connect(ui->joint2_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint2SpinChanged);
    connect(ui->joint3_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint3SpinChanged);
    connect(ui->joint4_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint4SpinChanged);
    connect(ui->joint6_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ArmControlGUI::onJoint6SpinChanged);
    
    // 设置吸盘控制连接
    connect(ui->vacuumPowerSlider, &QSlider::valueChanged, this, &ArmControlGUI::onVacuumPowerSliderChanged);
    connect(ui->vacuumOnButton, &QPushButton::clicked, this, &ArmControlGUI::onVacuumOnButtonClicked);
    connect(ui->vacuumOffButton, &QPushButton::clicked, this, &ArmControlGUI::onVacuumOffButtonClicked);
    
    // 连接笛卡尔坐标移动按钮
    connect(ui->moveToPositionButton, &QPushButton::clicked, this, &ArmControlGUI::onMoveToPositionClicked);
    
    // 连接末端控制按钮
    connect(ui->homeButton, &QPushButton::clicked, this, &ArmControlGUI::onHomeButtonClicked);
    
    // 连接检测表格点击事件
    connect(ui->detectionsTable, &QTableWidget::cellClicked, this, &ArmControlGUI::onDetectionsTableCellClicked);
    
    // 连接3D场景物体选择信号
    connect(scene_3d_renderer_, &Scene3DRenderer::objectSelected, this, &ArmControlGUI::on3DViewObjectSelected);
    
    // 安装事件过滤器
    this->installEventFilter(this);
    
    // 设置更新定时器
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &ArmControlGUI::onUpdateGUI);
    updateTimer->start(100);  // 每100ms更新一次
}