#include "arm_gui/enhanced_arm_gui.h"
#include "ui_arm_control_main.h"

#include <QCloseEvent>
#include <QTimerEvent>
#include <QDateTime>
#include <QPixmap>
#include <QDebug>

EnhancedArmGUI::EnhancedArmGUI(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::ArmControlMainWindow),
      nh_(nh),
      current_joint_values_(6, 0.0),
      vacuum_on_(false),
      vacuum_power_(50),
      current_view_mode_(0),
      ignore_slider_events_(false),
      ignore_spin_events_(false),
      is_connected_(false)
{
    // 设置UI
    ui->setupUi(this);
    
    // 初始化ROS
    initializeROS();
    
    // 设置关节控制范围
    setupJointControls();
    
    // 设置信号和槽的连接
    setupConnections();
    
    // 设置相机参数
    setupCameraParameters();
    
    // 创建占位图像
    QPixmap placeholder(640, 480);
    placeholder.fill(Qt::black);
    ui->cameraView->setPixmap(placeholder);
    
    // 启动UI更新定时器 (50ms, 20Hz)
    ui_update_timer_id_ = startTimer(50);
    
    // 记录日志
    logMessage("系统初始化完成");
}

EnhancedArmGUI::~EnhancedArmGUI()
{
    // 停止定时器
    if (ui_update_timer_id_ > 0) {
        killTimer(ui_update_timer_id_);
    }
    
    // 确保关闭真空吸盘
    if (is_connected_) {
        sendVacuumCommand(false, 0);
    }
    
    delete ui;
}

void EnhancedArmGUI::closeEvent(QCloseEvent* event)
{
    // 确保关闭真空吸盘
    if (is_connected_) {
        sendVacuumCommand(false, 0);
    }
    
    event->accept();
}

void EnhancedArmGUI::timerEvent(QTimerEvent* event)
{
    if (event->timerId() == ui_update_timer_id_) {
        updateUI();
    }
}

void EnhancedArmGUI::initializeROS()
{
    try {
        // 初始化发布者
        joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_command", 1);
        vacuum_cmd_pub_ = nh_.advertise<std_msgs::Bool>("/vacuum/command", 1);
        vacuum_power_pub_ = nh_.advertise<std_msgs::Int32>("/vacuum/power", 1);
        view_mode_pub_ = nh_.advertise<std_msgs::Header>("/stereo_vision/view_mode", 1);
        
        // 初始化服务客户端
        joint_control_client_ = nh_.serviceClient<servo_wrist::JointControl>("/joint_control");
        vacuum_control_client_ = nh_.serviceClient<servo_wrist::VacuumControl>("/vacuum_control");
        home_position_client_ = nh_.serviceClient<servo_wrist::HomePosition>("/home_position");
        set_view_mode_client_ = nh_.serviceClient<stereo_vision::SetViewMode>("/set_view_mode");
        
        // 初始化订阅者
        joint_state_sub_ = nh_.subscribe("/joint_states", 1, &EnhancedArmGUI::jointStateCallback, this);
        left_image_sub_ = nh_.subscribe("/left_camera/image_raw", 1, &EnhancedArmGUI::leftImageCallback, this);
        right_image_sub_ = nh_.subscribe("/right_camera/image_raw", 1, &EnhancedArmGUI::rightImageCallback, this);
        depth_image_sub_ = nh_.subscribe("/stereo_camera/depth/image_raw", 1, &EnhancedArmGUI::depthImageCallback, this);
        
        is_connected_ = true;
        logMessage("ROS连接成功");
    }
    catch (const std::exception& e) {
        is_connected_ = false;
        logMessage(QString("ROS连接失败: %1").arg(e.what()));
    }
}

void EnhancedArmGUI::setupConnections()
{
    // 关节滑块连接
    connect(ui->joint1Slider, &QSlider::valueChanged, this, &EnhancedArmGUI::onJointSliderChanged);
    connect(ui->joint2Slider, &QSlider::valueChanged, this, &EnhancedArmGUI::onJointSliderChanged);
    connect(ui->joint3Slider, &QSlider::valueChanged, this, &EnhancedArmGUI::onJointSliderChanged);
    connect(ui->joint4Slider, &QSlider::valueChanged, this, &EnhancedArmGUI::onJointSliderChanged);
    connect(ui->joint5Slider, &QSlider::valueChanged, this, &EnhancedArmGUI::onJointSliderChanged);
    connect(ui->joint6Slider, &QSlider::valueChanged, this, &EnhancedArmGUI::onJointSliderChanged);
    
    // 关节数值框连接
    connect(ui->joint1Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &EnhancedArmGUI::onJointSpinChanged);
    connect(ui->joint2Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &EnhancedArmGUI::onJointSpinChanged);
    connect(ui->joint3Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &EnhancedArmGUI::onJointSpinChanged);
    connect(ui->joint4Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &EnhancedArmGUI::onJointSpinChanged);
    connect(ui->joint5Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &EnhancedArmGUI::onJointSpinChanged);
    connect(ui->joint6Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &EnhancedArmGUI::onJointSpinChanged);
    
    // 末端控制连接
    connect(ui->moveToPositionButton, &QPushButton::clicked, this, &EnhancedArmGUI::onMoveToPositionClicked);
    
    // 真空吸盘控制连接
    connect(ui->vacuumOnButton, &QPushButton::clicked, this, &EnhancedArmGUI::onVacuumOnClicked);
    connect(ui->vacuumOffButton, &QPushButton::clicked, this, &EnhancedArmGUI::onVacuumOffClicked);
    connect(ui->vacuumPowerSlider, &QSlider::valueChanged, this, &EnhancedArmGUI::onVacuumPowerChanged);
    
    // 相机视图控制连接
    connect(ui->leftViewButton, &QPushButton::clicked, this, &EnhancedArmGUI::onLeftViewClicked);
    connect(ui->rightViewButton, &QPushButton::clicked, this, &EnhancedArmGUI::onRightViewClicked);
    connect(ui->depthViewButton, &QPushButton::clicked, this, &EnhancedArmGUI::onDepthViewClicked);
    
    // 其他控制连接
    connect(ui->homeButton, &QPushButton::clicked, this, &EnhancedArmGUI::onHomeButtonClicked);
}

void EnhancedArmGUI::setupJointControls()
{
    // 设置关节1滑块和数值框范围 (底座旋转)
    ui->joint1Slider->setRange(-180, 180);
    ui->joint1Spin->setRange(-180.0, 180.0);
    
    // 设置关节2滑块和数值框范围 (肩部关节)
    ui->joint2Slider->setRange(0, 50);
    ui->joint2Spin->setRange(0.0, 50.0);
    
    // 设置关节3滑块和数值框范围 (肘部关节)
    ui->joint3Slider->setRange(-90, 90);
    ui->joint3Spin->setRange(-90.0, 90.0);
    
    // 设置关节4滑块和数值框范围 (腕部旋转)
    ui->joint4Slider->setRange(0, 180);
    ui->joint4Spin->setRange(0.0, 180.0);
    
    // 设置关节5滑块和数值框范围 (腕部俯仰)
    ui->joint5Slider->setRange(-90, 90);
    ui->joint5Spin->setRange(-90.0, 90.0);
    
    // 设置关节6滑块和数值框范围 (夹爪控制)
    ui->joint6Slider->setRange(5, 15);
    ui->joint6Spin->setRange(5.0, 15.0);
}

void EnhancedArmGUI::setupCameraParameters()
{
    // 这里可以设置相机的内参、外参等
    // 暂时不需要实现
}

void EnhancedArmGUI::updateUI()
{
    // 更新关节状态标签
    if (!current_joint_values_.empty() && current_joint_values_.size() >= 6) {
        QString jointStatus = QString("关节状态: J1=%1° J2=%2° J3=%3° J4=%4° J5=%5° J6=%6°")
            .arg(current_joint_values_[0], 0, 'f', 1)
            .arg(current_joint_values_[1], 0, 'f', 1)
            .arg(current_joint_values_[2], 0, 'f', 1)
            .arg(current_joint_values_[3], 0, 'f', 1)
            .arg(current_joint_values_[4], 0, 'f', 1)
            .arg(current_joint_values_[5], 0, 'f', 1);
        ui->jointStatusLabel->setText(jointStatus);
    }
    
    // 更新末端位置标签
    QString endEffectorStatus = QString("末端位置: X=%1cm Y=%2cm Z=%3cm")
        .arg(current_end_pose_.position.x * 100, 0, 'f', 1)
        .arg(current_end_pose_.position.y * 100, 0, 'f', 1)
        .arg(current_end_pose_.position.z * 100, 0, 'f', 1);
    ui->endEffectorStatusLabel->setText(endEffectorStatus);
    
    // 更新相机视图
    if (!current_display_image_.isNull()) {
        ui->cameraView->setPixmap(QPixmap::fromImage(current_display_image_).scaled(
            ui->cameraView->width(), ui->cameraView->height(), Qt::KeepAspectRatio));
    }
    
    // 处理ROS消息
    ros::spinOnce();
}

void EnhancedArmGUI::onJointSliderChanged(int value)
{
    if (ignore_slider_events_) return;
    
    // 防止循环触发
    ignore_spin_events_ = true;
    
    // 更新对应的数值框
    QSlider* slider = qobject_cast<QSlider*>(sender());
    if (slider == ui->joint1Slider) ui->joint1Spin->setValue(value);
    else if (slider == ui->joint2Slider) ui->joint2Spin->setValue(value);
    else if (slider == ui->joint3Slider) ui->joint3Spin->setValue(value);
    else if (slider == ui->joint4Slider) ui->joint4Spin->setValue(value);
    else if (slider == ui->joint5Slider) ui->joint5Spin->setValue(value);
    else if (slider == ui->joint6Slider) ui->joint6Spin->setValue(value);
    
    // 发送关节命令
    sendJointCommand();
    
    ignore_spin_events_ = false;
}

void EnhancedArmGUI::onJointSpinChanged(double value)
{
    if (ignore_spin_events_) return;
    
    // 防止循环触发
    ignore_slider_events_ = true;
    
    // 更新对应的滑块
    QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(sender());
    if (spinBox == ui->joint1Spin) ui->joint1Slider->setValue(static_cast<int>(value));
    else if (spinBox == ui->joint2Spin) ui->joint2Slider->setValue(static_cast<int>(value));
    else if (spinBox == ui->joint3Spin) ui->joint3Slider->setValue(static_cast<int>(value));
    else if (spinBox == ui->joint4Spin) ui->joint4Slider->setValue(static_cast<int>(value));
    else if (spinBox == ui->joint5Spin) ui->joint5Slider->setValue(static_cast<int>(value));
    else if (spinBox == ui->joint6Spin) ui->joint6Slider->setValue(static_cast<int>(value));
    
    // 发送关节命令
    sendJointCommand();
    
    ignore_slider_events_ = false;
}

void EnhancedArmGUI::onMoveToPositionClicked()
{
    // 获取目标位置
    double x = ui->posXSpin->value() / 100.0; // 转换为米
    double y = ui->posYSpin->value() / 100.0;
    double z = ui->posZSpin->value() / 100.0;
    
    // 这里应该调用逆运动学服务，但我们简化为直接发送消息
    logMessage(QString("移动到位置: X=%1m Y=%2m Z=%3m").arg(x).arg(y).arg(z));
    
    // 在实际应用中，这里应该调用逆运动学服务
    // 暂时不实现
}

void EnhancedArmGUI::onVacuumOnClicked()
{
    sendVacuumCommand(true, ui->vacuumPowerSlider->value());
    logMessage(QString("开启真空吸盘，功率: %1%").arg(ui->vacuumPowerSlider->value()));
}

void EnhancedArmGUI::onVacuumOffClicked()
{
    sendVacuumCommand(false, 0);
    logMessage("关闭真空吸盘");
}

void EnhancedArmGUI::onVacuumPowerChanged(int value)
{
    if (vacuum_on_) {
        sendVacuumCommand(true, value);
        logMessage(QString("调整真空吸盘功率: %1%").arg(value));
    }
    
    vacuum_power_ = value;
}

void EnhancedArmGUI::onLeftViewClicked()
{
    setViewMode(0);
    logMessage("切换到左视图");
}

void EnhancedArmGUI::onRightViewClicked()
{
    setViewMode(1);
    logMessage("切换到右视图");
}

void EnhancedArmGUI::onDepthViewClicked()
{
    setViewMode(2);
    logMessage("切换到深度视图");
}

void EnhancedArmGUI::onHomeButtonClicked()
{
    sendHomeCommand();
    logMessage("移动到初始位置");
}

void EnhancedArmGUI::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->position.size() >= 6) {
        current_joint_values_ = msg->position;
        
        // 更新UI控件，但不触发回调
        ignore_slider_events_ = true;
        ignore_spin_events_ = true;
        
        ui->joint1Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[0])));
        ui->joint2Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[1])));
        ui->joint3Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[2])));
        ui->joint4Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[3])));
        ui->joint5Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[4])));
        ui->joint6Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[5])));
        
        ui->joint1Spin->setValue(radToDeg(current_joint_values_[0]));
        ui->joint2Spin->setValue(radToDeg(current_joint_values_[1]));
        ui->joint3Spin->setValue(radToDeg(current_joint_values_[2]));
        ui->joint4Spin->setValue(radToDeg(current_joint_values_[3]));
        ui->joint5Spin->setValue(radToDeg(current_joint_values_[4]));
        ui->joint6Spin->setValue(radToDeg(current_joint_values_[5]));
        
        ignore_slider_events_ = false;
        ignore_spin_events_ = false;
    }
}

void EnhancedArmGUI::leftImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 尝试使用不同的编码格式
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            // 如果BGR8失败，尝试其他格式
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
                if (cv_ptr->image.channels() == 1) {
                    // 如果是单通道图像，转换为BGR
                    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2BGR);
                }
                else if (cv_ptr->image.channels() == 4) {
                    // 如果是RGBA图像，转换为BGR
                    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGRA2BGR);
                }
            }
            catch (cv_bridge::Exception& e2) {
                logMessage(QString("左相机图像转换错误，尝试所有格式都失败: %1, %2").arg(e.what()).arg(e2.what()));
                return;
            }
        }
        
        // 检查图像是否为空或损坏
        if (cv_ptr->image.empty() || cv_ptr->image.rows <= 0 || cv_ptr->image.cols <= 0) {
            logMessage("左相机图像为空或损坏");
            return;
        }
        
        left_image_ = cv_ptr->image;
        
        if (current_view_mode_ == 0) {
            current_display_image_ = cvMatToQImage(left_image_);
        }
    }
    catch (std::exception& e) {
        logMessage(QString("左相机图像处理错误: %1").arg(e.what()));
    }
}

void EnhancedArmGUI::rightImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 尝试使用不同的编码格式
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            // 如果BGR8失败，尝试其他格式
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
                if (cv_ptr->image.channels() == 1) {
                    // 如果是单通道图像，转换为BGR
                    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2BGR);
                }
                else if (cv_ptr->image.channels() == 4) {
                    // 如果是RGBA图像，转换为BGR
                    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGRA2BGR);
                }
            }
            catch (cv_bridge::Exception& e2) {
                logMessage(QString("右相机图像转换错误，尝试所有格式都失败: %1, %2").arg(e.what()).arg(e2.what()));
                return;
            }
        }
        
        // 检查图像是否为空或损坏
        if (cv_ptr->image.empty() || cv_ptr->image.rows <= 0 || cv_ptr->image.cols <= 0) {
            logMessage("右相机图像为空或损坏");
            return;
        }
        
        right_image_ = cv_ptr->image;
        
        if (current_view_mode_ == 1) {
            current_display_image_ = cvMatToQImage(right_image_);
        }
    }
    catch (std::exception& e) {
        logMessage(QString("右相机图像处理错误: %1").arg(e.what()));
    }
}

void EnhancedArmGUI::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // 尝试使用不同的编码格式
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            // 如果BGR8失败，尝试其他格式
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
                if (cv_ptr->image.channels() == 1) {
                    // 如果是单通道图像，转换为BGR
                    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2BGR);
                }
                else if (cv_ptr->image.channels() == 4) {
                    // 如果是RGBA图像，转换为BGR
                    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGRA2BGR);
                }
            }
            catch (cv_bridge::Exception& e2) {
                logMessage(QString("深度图像转换错误，尝试所有格式都失败: %1, %2").arg(e.what()).arg(e2.what()));
                return;
            }
        }
        
        // 检查图像是否为空或损坏
        if (cv_ptr->image.empty() || cv_ptr->image.rows <= 0 || cv_ptr->image.cols <= 0) {
            logMessage("深度图像为空或损坏");
            return;
        }
        
        depth_image_ = cv_ptr->image;
        
        if (current_view_mode_ == 2) {
            current_display_image_ = cvMatToQImage(depth_image_);
        }
    }
    catch (std::exception& e) {
        logMessage(QString("深度图像处理错误: %1").arg(e.what()));
    }
}

void EnhancedArmGUI::sendJointCommand()
{
    std::vector<double> joint_values = {
        degToRad(ui->joint1Spin->value()),
        degToRad(ui->joint2Spin->value()),
        degToRad(ui->joint3Spin->value()),
        degToRad(ui->joint4Spin->value()),
        degToRad(ui->joint5Spin->value()),
        degToRad(ui->joint6Spin->value())
    };
    
    sendJointCommand(joint_values);
}

void EnhancedArmGUI::sendJointCommand(const std::vector<double>& joint_values)
{
    if (!is_connected_) return;
    
    // 使用服务调用
    if (joint_control_client_.exists()) {
        servo_wrist::JointControl srv;
        srv.request.position = joint_values;
        
        if (joint_control_client_.call(srv)) {
            if (srv.response.success) {
                // 成功发送
            } else {
                logMessage(QString("关节控制失败: %1").arg(srv.response.message.c_str()));
            }
        } else {
            logMessage("调用关节控制服务失败");
        }
    } else {
        // 使用话题发布
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
        joint_msg.position = joint_values;
        joint_command_pub_.publish(joint_msg);
    }
}

void EnhancedArmGUI::sendVacuumCommand(bool on, int power)
{
    if (!is_connected_) return;
    
    // 发布消息
    std_msgs::Bool on_msg;
    on_msg.data = on;
    vacuum_cmd_pub_.publish(on_msg);
    
    std_msgs::Int32 power_msg;
    power_msg.data = power;
    vacuum_power_pub_.publish(power_msg);
    
    // 使用服务调用
    if (vacuum_control_client_.exists()) {
        servo_wrist::VacuumControl srv;
        srv.request.enable = on;
        srv.request.power = power;
        
        if (vacuum_control_client_.call(srv)) {
            if (srv.response.success) {
                // 成功发送
            } else {
                logMessage(QString("真空吸盘控制失败: %1").arg(srv.response.message.c_str()));
            }
        } else {
            logMessage("调用真空吸盘控制服务失败");
        }
    }
    
    vacuum_on_ = on;
    vacuum_power_ = on ? power : 0;
}

void EnhancedArmGUI::sendHomeCommand()
{
    if (!is_connected_) return;
    
    if (home_position_client_.exists()) {
        servo_wrist::HomePosition srv;
        
        if (home_position_client_.call(srv)) {
            if (srv.response.success) {
                logMessage("成功移动到初始位置");
            } else {
                logMessage(QString("移动到初始位置失败: %1").arg(srv.response.message.c_str()));
            }
        } else {
            logMessage("调用初始位置服务失败");
        }
    } else {
        logMessage("初始位置服务不可用");
    }
}

void EnhancedArmGUI::setViewMode(int mode)
{
    if (mode < 0 || mode > 2) return;
    
    current_view_mode_ = mode;
    
    // 使用服务调用
    if (set_view_mode_client_.exists()) {
        stereo_vision::SetViewMode srv;
        srv.request.view_mode = mode;
        
        if (set_view_mode_client_.call(srv)) {
            if (srv.response.success) {
                // 成功设置
            } else {
                logMessage(QString("设置视图模式失败: %1").arg(srv.response.message.c_str()));
            }
        } else {
            logMessage("调用设置视图模式服务失败");
        }
    } else {
        // 使用话题发布
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = QString::number(mode).toStdString();
        view_mode_pub_.publish(header);
    }
    
    // 更新显示图像
    switch (mode) {
        case 0:
            if (!left_image_.empty()) {
                current_display_image_ = cvMatToQImage(left_image_);
            }
            break;
        case 1:
            if (!right_image_.empty()) {
                current_display_image_ = cvMatToQImage(right_image_);
            }
            break;
        case 2:
            if (!depth_image_.empty()) {
                current_display_image_ = cvMatToQImage(depth_image_);
            }
            break;
    }
}

QImage EnhancedArmGUI::cvMatToQImage(const cv::Mat& mat)
{
    if (mat.empty()) {
        return QImage();
    }
    
    // 8-bit, 3 channel
    if (mat.type() == CV_8UC3) {
        QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped(); // BGR to RGB
    }
    // 8-bit, 1 channel
    else if (mat.type() == CV_8UC1) {
        QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
        return image;
    }
    // 8-bit, 4 channel
    else if (mat.type() == CV_8UC4) {
        QImage image(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.rgbSwapped(); // BGRA to ARGB
    }
    
    return QImage();
}

void EnhancedArmGUI::logMessage(const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    QString logEntry = QString("[%1] %2").arg(timestamp).arg(message);
    
    ui->logTextEdit->append(logEntry);
    ui->logTextEdit->ensureCursorVisible();
}

double EnhancedArmGUI::degToRad(double deg)
{
    return deg * M_PI / 180.0;
}

double EnhancedArmGUI::radToDeg(double rad)
{
    return rad * 180.0 / M_PI;
}