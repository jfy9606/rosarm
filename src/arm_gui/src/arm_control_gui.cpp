#include "arm_gui/arm_control_gui.h"
#include "ui_arm_control_main.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QTimer>
#include <QCloseEvent>
#include <cmath>

ArmControlGUI::ArmControlGUI(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent), ui(new Ui::ArmControlMainWindow), nh_(nh),
    vacuum_on_(false), vacuum_power_(50), selected_object_index_(-1),
    yolo_enabled_(false), camera_view_mode_(0), is_camera_available_(false),
    stereo_camera_error_count_(0), available_camera_index_(-1),
    ignore_slider_events_(false), ignore_spin_events_(false), ui_processing_(false),
    scene_3d_renderer_(nullptr), updateTimer(new QTimer(this))
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
    
    // 启动定时器
    updateTimer->start(100);  // 每100ms更新一次
    
    statusBar()->showMessage("机械臂控制界面已启动");
}

ArmControlGUI::~ArmControlGUI()
{
    if (scene_3d_renderer_) {
        delete scene_3d_renderer_;
    }
    delete ui;
}

void ArmControlGUI::initializeROS()
{
    try {
        // 初始化服务客户端
        fk_client_ = nh_.serviceClient<arm_trajectory::ForwardKinematics>("forward_kinematics");
        ik_client_ = nh_.serviceClient<arm_trajectory::InverseKinematics>("inverse_kinematics");
        
        // 初始化发布者
        joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_command", 1);
        vacuum_cmd_pub_ = nh_.advertise<std_msgs::Bool>("vacuum_command", 1);
        vacuum_power_pub_ = nh_.advertise<std_msgs::Int32>("vacuum_power", 1);
        
        // 初始化订阅者
        joint_state_sub_ = nh_.subscribe("joint_states", 1, 
                                      &ArmControlGUI::jointStateCallback, this);
    }
    catch (const std::exception& e) {
        QMessageBox::critical(this, "ROS初始化错误", QString("ROS初始化失败: %1").arg(e.what()));
    }
}

void ArmControlGUI::initializeMembers()
{
    current_joint_values_.resize(6, 0.0);
    current_end_position_ = QVector3D(0.0f, 0.0f, 0.0f);
    current_end_orientation_ = QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
}

void ArmControlGUI::setupUi()
{
    // 设置关节控件范围
    ui->joint1Slider->setRange(-180, 180);
    ui->joint2Slider->setRange(0, 50);
    ui->joint3Slider->setRange(-90, 90);
    ui->joint4Slider->setRange(0, 180);
    ui->joint6Slider->setRange(5, 15);
    
    ui->joint1Spin->setRange(-180.0, 180.0);
    ui->joint2Spin->setRange(0.0, 50.0);
    ui->joint3Spin->setRange(-90.0, 90.0);
    ui->joint4Spin->setRange(0.0, 180.0);
    ui->joint6Spin->setRange(5.0, 15.0);
}

void ArmControlGUI::connectSignalSlots()
{
    // 关节控制连接
    connect(ui->joint1Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint1SliderChanged);
    connect(ui->joint2Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint2SliderChanged);
    connect(ui->joint3Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint3SliderChanged);
    connect(ui->joint4Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint4SliderChanged);
    connect(ui->joint6Slider, &QSlider::valueChanged, this, &ArmControlGUI::onJoint6SliderChanged);
    
    connect(ui->joint1Spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &ArmControlGUI::onJoint1SpinChanged);
    connect(ui->joint2Spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &ArmControlGUI::onJoint2SpinChanged);
    connect(ui->joint3Spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &ArmControlGUI::onJoint3SpinChanged);
    connect(ui->joint4Spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &ArmControlGUI::onJoint4SpinChanged);
    connect(ui->joint6Spin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &ArmControlGUI::onJoint6SpinChanged);
    
    // 吸附控制连接
    connect(ui->vacuumPowerSlider, &QSlider::valueChanged, this, &ArmControlGUI::onVacuumPowerSliderChanged);
    connect(ui->vacuumOnButton, &QPushButton::clicked, this, &ArmControlGUI::onVacuumOnButtonClicked);
    connect(ui->vacuumOffButton, &QPushButton::clicked, this, &ArmControlGUI::onVacuumOffButtonClicked);
    
    // 其他按钮连接
    connect(ui->homeButton, &QPushButton::clicked, this, &ArmControlGUI::onHomeButtonClicked);
    
    // 定时器连接
    connect(updateTimer, &QTimer::timeout, this, &ArmControlGUI::onUpdateGUI);
}

void ArmControlGUI::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->position.size() >= 6) {
        // 更新当前关节值
        for (size_t i = 0; i < std::min(msg->position.size(), current_joint_values_.size()); ++i) {
            current_joint_values_[i] = msg->position[i];
        }
        
        // 更新UI
        if (!ui_processing_) {
            updateGUIJointValues();
        }
        
        // 计算末端位姿
        geometry_msgs::Pose end_pose = forwardKinematics(current_joint_values_);
        current_end_position_ = QVector3D(end_pose.position.x * 100.0f, 
                                         end_pose.position.y * 100.0f,
                                         end_pose.position.z * 100.0f);
        current_end_orientation_ = QQuaternion(end_pose.orientation.w,
                                             end_pose.orientation.x,
                                             end_pose.orientation.y,
                                             end_pose.orientation.z);
        current_end_pose_ = end_pose;
    }
}

void ArmControlGUI::onUpdateGUI()
{
    updateUI();
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
    // 在状态栏显示末端执行器位置
    QString posText = QString("末端位置: X=%1 Y=%2 Z=%3 cm").arg(
        current_end_position_.x(), 0, 'f', 2).arg(
        current_end_position_.y(), 0, 'f', 2).arg(
        current_end_position_.z(), 0, 'f', 2);
    
    statusBar()->showMessage(posText);
}

void ArmControlGUI::updateVacuumStatus()
{
    // 更新吸附按钮状态
    ui->vacuumOnButton->setEnabled(!vacuum_on_);
    ui->vacuumOffButton->setEnabled(vacuum_on_);
}

void ArmControlGUI::updateGUIJointValues()
{
    ignore_slider_events_ = true;
    ignore_spin_events_ = true;
    
    // 转换为度数显示
    double joint1_deg = radToDeg(current_joint_values_[0]);
    ui->joint1Slider->setValue(static_cast<int>(joint1_deg));
    ui->joint1Spin->setValue(joint1_deg);
    
    // Joint 2是直线移动，单位是厘米
    ui->joint2Slider->setValue(static_cast<int>(current_joint_values_[1]));
    ui->joint2Spin->setValue(current_joint_values_[1]);
    
    // 转换为度数显示
    double joint3_deg = radToDeg(current_joint_values_[2]);
    ui->joint3Slider->setValue(static_cast<int>(joint3_deg));
    ui->joint3Spin->setValue(joint3_deg);
    
    // 转换为度数显示
    double joint4_deg = radToDeg(current_joint_values_[3]);
    ui->joint4Slider->setValue(static_cast<int>(joint4_deg));
    ui->joint4Spin->setValue(joint4_deg);
    
    ignore_slider_events_ = false;
    ignore_spin_events_ = false;
}

// 辅助数学函数
double ArmControlGUI::degToRad(double deg)
{
    return deg * M_PI / 180.0;
}

double ArmControlGUI::radToDeg(double rad)
{
    return rad * 180.0 / M_PI;
}

QMatrix4x4 ArmControlGUI::computeDHTransform(double theta, double d, double a, double alpha)
{
    QMatrix4x4 transform;
    
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);
    
    transform(0, 0) = cos_theta;
    transform(0, 1) = -sin_theta * cos_alpha;
    transform(0, 2) = sin_theta * sin_alpha;
    transform(0, 3) = a * cos_theta;
    
    transform(1, 0) = sin_theta;
    transform(1, 1) = cos_theta * cos_alpha;
    transform(1, 2) = -cos_theta * sin_alpha;
    transform(1, 3) = a * sin_theta;
    
    transform(2, 0) = 0;
    transform(2, 1) = sin_alpha;
    transform(2, 2) = cos_alpha;
    transform(2, 3) = d;
    
    transform(3, 0) = 0;
    transform(3, 1) = 0;
    transform(3, 2) = 0;
    transform(3, 3) = 1;
    
    return transform;
}

geometry_msgs::Pose ArmControlGUI::forwardKinematics(const std::vector<double>& joint_values)
{
    // 调用服务
    arm_trajectory::ForwardKinematics srv;
    srv.request.arm_id = "arm1";
    srv.request.joint_values = joint_values;
    
    geometry_msgs::Pose result;
    
    if (fk_client_.call(srv)) {
        if (srv.response.success) {
            return srv.response.end_effector_pose;
        } else {
            ROS_WARN("Forward kinematics service failed: %s", srv.response.message.c_str());
        }
    } else {
        ROS_ERROR("Failed to call forward kinematics service");
    }
    
    // 如果服务调用失败，返回默认姿态
    result.position.x = 0.0;
    result.position.y = 0.0;
    result.position.z = 0.0;
    result.orientation.w = 1.0;
    result.orientation.x = 0.0;
    result.orientation.y = 0.0;
    result.orientation.z = 0.0;
    
    return result;
}

std::vector<double> ArmControlGUI::inverseKinematics(const geometry_msgs::Pose& target_pose,
                                                   const std::vector<double>& initial_guess)
{
    // 调用服务
    arm_trajectory::InverseKinematics srv;
    srv.request.arm_id = "arm1";
    srv.request.target_pose = target_pose;
    
    if (!initial_guess.empty()) {
        srv.request.use_initial_guess = true;
        srv.request.initial_guess = initial_guess;
    } else {
        srv.request.use_initial_guess = false;
    }
    
    if (ik_client_.call(srv)) {
        if (srv.response.success) {
            return srv.response.joint_values;
        } else {
            ROS_WARN("Inverse kinematics service failed: %s", srv.response.message.c_str());
        }
    } else {
        ROS_ERROR("Failed to call inverse kinematics service");
    }
    
    // 如果服务调用失败，返回当前关节状态
    return current_joint_values_;
}

// 控制函数
void ArmControlGUI::sendJointCommand(const std::vector<double>& joint_values)
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.position = joint_values;
    
    joint_command_pub_.publish(msg);
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
    std::vector<double> home_position = {0.0, 10.0, 0.0, 1.57, 1.57, 10.0};
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

// 未实现的方法存根
void ArmControlGUI::onMoveToPositionClicked() {}
void ArmControlGUI::on_moveToPositionButton_clicked() {}
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
void ArmControlGUI::updateCameraView() {}
void ArmControlGUI::updateDetectionsTable() {}
void ArmControlGUI::updateConnectionStatus() {}
void ArmControlGUI::updateJointControlWidgets() {}
void ArmControlGUI::initializeGUI() {}
void ArmControlGUI::initializeOpenGL() {}
void ArmControlGUI::initializeJointControlConnections() {}
void ArmControlGUI::setupROSSubscriptions() {}
void ArmControlGUI::createMenus() {}
void ArmControlGUI::setupCameraParameters() {}
bool ArmControlGUI::checkJointLimits(const std::vector<double>& joint_values) { return true; }
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
void ArmControlGUI::createPlaceholderImage() {}
void ArmControlGUI::attemptCameraReconnect() {}
bool ArmControlGUI::findAvailableCamera() { return false; }
QImage ArmControlGUI::cvMatToQImage(const cv::Mat& mat) { return QImage(); }
std::vector<double> ArmControlGUI::poseToJoints(const geometry_msgs::Pose& pose) { return std::vector<double>(); }
geometry_msgs::Pose ArmControlGUI::jointsToPos(const std::vector<double>& joint_values) { return geometry_msgs::Pose(); }
int ArmControlGUI::map(int value, int fromLow, int fromHigh, int toLow, int toHigh) { return 0; }
void ArmControlGUI::updateEndEffectorPosition(double x, double y, double z) {}
void ArmControlGUI::stereoMergedCallback(const sensor_msgs::Image::ConstPtr& msg) {}
void ArmControlGUI::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {}
void ArmControlGUI::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg) {}
void ArmControlGUI::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {}
void ArmControlGUI::yoloStatusCallback(const std_msgs::Bool::ConstPtr& msg) {}
void ArmControlGUI::objectDetectionCallback(const sensor_msgs::Image::ConstPtr& img_msg, 
                                           const geometry_msgs::PoseArray::ConstPtr& poses_msg) {}
void ArmControlGUI::updateScene3D() {}
void ArmControlGUI::updateSceneObjects() {}
void ArmControlGUI::onEndEffectorDragged(QVector3D position) {}
void ArmControlGUI::setupJointLimits() {}
void ArmControlGUI::setupDHParameters() {}
