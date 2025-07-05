#include <QApplication>
#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QWidget>
#include <QTimer>
#include <QGroupBox>
#include <QStatusBar>
#include <QFrame>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QTabWidget>
#include <QScrollArea>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

// 创建一个简单的GUI类，作为替代
class SimpleArmGUI : public QMainWindow
{
    Q_OBJECT

public:
    SimpleArmGUI(ros::NodeHandle& nh, QWidget* parent = nullptr) 
        : QMainWindow(parent), nh_(nh), updateTimer(new QTimer(this))
    {
        // 设置窗口标题和大小
        setWindowTitle("Robotic Arm Control Interface");
        resize(1024, 768);
        
        // 创建中央窗口部件
        QWidget* centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);
        
        // 创建主布局
        QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);
        
        // 创建左侧控制面板
        QWidget* controlPanel = new QWidget(this);
        QVBoxLayout* controlLayout = new QVBoxLayout(controlPanel);
        controlPanel->setMaximumWidth(400);
        
        // 创建关节控制组
        createJointControlGroup(controlLayout);
        
        // 创建末端执行器控制组
        createEndEffectorGroup(controlLayout);
        
        // 创建真空吸盘控制组
        createVacuumGroup(controlLayout);
        
        // 创建回到初始位置按钮
        homeButton = new QPushButton("Return to Home Position", this);
        controlLayout->addWidget(homeButton);
        
        // 添加弹性空间
        controlLayout->addStretch(1);
        
        // 创建右侧视图面板
        QWidget* viewPanel = new QWidget(this);
        QVBoxLayout* viewLayout = new QVBoxLayout(viewPanel);
        
        // 创建相机视图标签
        cameraView = new QLabel("Camera View (Waiting for connection...)", this);
        cameraView->setMinimumSize(640, 480);
        cameraView->setAlignment(Qt::AlignCenter);
        cameraView->setFrameStyle(QFrame::Panel | QFrame::Sunken);
        viewLayout->addWidget(cameraView);
        
        // 创建相机控制布局
        QHBoxLayout* cameraControlLayout = new QHBoxLayout();
        
        // 添加左视图按钮
        leftViewButton = new QPushButton("Left View", this);
        cameraControlLayout->addWidget(leftViewButton);
        
        // 添加右视图按钮
        rightViewButton = new QPushButton("Right View", this);
        cameraControlLayout->addWidget(rightViewButton);
        
        // 添加相机控制到视图布局
        viewLayout->addLayout(cameraControlLayout);
        
        // 添加控制面板和视图面板到主布局
        mainLayout->addWidget(controlPanel);
        mainLayout->addWidget(viewPanel, 1);  // 视图面板占据更多空间
        
        // 创建状态栏
        statusBar()->showMessage("Robotic Arm Control Interface Started");
        
        // 连接按钮信号
        connect(leftViewButton, &QPushButton::clicked, this, &SimpleArmGUI::onLeftViewClicked);
        connect(rightViewButton, &QPushButton::clicked, this, &SimpleArmGUI::onRightViewClicked);
        connect(homeButton, &QPushButton::clicked, this, &SimpleArmGUI::onHomeClicked);
        connect(vacuumOnButton, &QPushButton::clicked, this, &SimpleArmGUI::onVacuumOnClicked);
        connect(vacuumOffButton, &QPushButton::clicked, this, &SimpleArmGUI::onVacuumOffClicked);
        connect(moveToPositionButton, &QPushButton::clicked, this, &SimpleArmGUI::onMoveToPositionClicked);
        
        // 连接关节控制信号
        connectJointControls();
        
        // 设置定时器
        connect(updateTimer, &QTimer::timeout, this, &SimpleArmGUI::updateUI);
        updateTimer->start(100);  // 100ms更新一次
        
        // 初始化ROS订阅
        initializeROS();
        
        // 初始化关节值
        current_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        updateJointControlWidgets();
    }
    
    ~SimpleArmGUI() {
        if (updateTimer) {
            updateTimer->stop();
            delete updateTimer;
        }
    }

private slots:
    void onLeftViewClicked() {
        // 发布视图模式消息 - 左视图 (0)
        std_msgs::Int32 msg;
        msg.data = 0;
        viewModePub.publish(msg);
        statusBar()->showMessage("Switched to Left View");
        ROS_INFO("View mode changed to LEFT (0)");
    }
    
    void onRightViewClicked() {
        // 发布视图模式消息 - 右视图 (1)
        std_msgs::Int32 msg;
        msg.data = 1;
        viewModePub.publish(msg);
        statusBar()->showMessage("Switched to Right View");
        ROS_INFO("View mode changed to RIGHT (1)");
    }
    
    void onHomeClicked() {
        statusBar()->showMessage("Attempting to return to home position...");
        ROS_INFO("Requesting arm to return to home position");
        // 发布回到初始位置命令
        std_msgs::Bool msg;
        msg.data = true;
        homeCommandPub.publish(msg);
    }
    
    void onVacuumOnClicked() {
        statusBar()->showMessage("Attempting to turn vacuum gripper ON...");
        ROS_INFO("Requesting vacuum gripper ON");
        // 发布开启真空吸盘命令
        std_msgs::Bool msg;
        msg.data = true;
        vacuumCommandPub.publish(msg);
    }
    
    void onVacuumOffClicked() {
        statusBar()->showMessage("Attempting to turn vacuum gripper OFF...");
        ROS_INFO("Requesting vacuum gripper OFF");
        // 发布关闭真空吸盘命令
        std_msgs::Bool msg;
        msg.data = false;
        vacuumCommandPub.publish(msg);
    }
    
    void onMoveToPositionClicked() {
        // 获取末端执行器目标位置
        double x = posXSpin->value();
        double y = posYSpin->value();
        double z = posZSpin->value();
        
        statusBar()->showMessage(QString("Moving to position: X=%1, Y=%2, Z=%3").arg(x).arg(y).arg(z));
        ROS_INFO("Moving to position: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);
        
        // 这里应该发送末端执行器位置命令
        // 由于我们没有完整实现逆运动学，这里只是示例
    }
    
    void updateUI() {
        // 定期检查ROS连接状态
        if (!ros::ok()) {
            statusBar()->showMessage("ROS connection lost!");
            ROS_WARN("ROS connection appears to be lost");
        }
    }
    
    // 关节滑块变化槽函数
    void onJoint1SliderChanged(int value) {
        if (!ignoreSliderEvents) {
            ignoreSpinEvents = true;
            joint1Spin->setValue(value);
            ignoreSpinEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint2SliderChanged(int value) {
        if (!ignoreSliderEvents) {
            ignoreSpinEvents = true;
            joint2Spin->setValue(value);
            ignoreSpinEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint3SliderChanged(int value) {
        if (!ignoreSliderEvents) {
            ignoreSpinEvents = true;
            joint3Spin->setValue(value);
            ignoreSpinEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint4SliderChanged(int value) {
        if (!ignoreSliderEvents) {
            ignoreSpinEvents = true;
            joint4Spin->setValue(value);
            ignoreSpinEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint5SliderChanged(int value) {
        if (!ignoreSliderEvents) {
            ignoreSpinEvents = true;
            joint5Spin->setValue(value);
            ignoreSpinEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint6SliderChanged(int value) {
        if (!ignoreSliderEvents) {
            ignoreSpinEvents = true;
            joint6Spin->setValue(value);
            ignoreSpinEvents = false;
            sendJointCommand();
        }
    }
    
    // 关节微调框变化槽函数
    void onJoint1SpinChanged(double value) {
        if (!ignoreSpinEvents) {
            ignoreSliderEvents = true;
            joint1Slider->setValue(static_cast<int>(value));
            ignoreSliderEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint2SpinChanged(double value) {
        if (!ignoreSpinEvents) {
            ignoreSliderEvents = true;
            joint2Slider->setValue(static_cast<int>(value));
            ignoreSliderEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint3SpinChanged(double value) {
        if (!ignoreSpinEvents) {
            ignoreSliderEvents = true;
            joint3Slider->setValue(static_cast<int>(value));
            ignoreSliderEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint4SpinChanged(double value) {
        if (!ignoreSpinEvents) {
            ignoreSliderEvents = true;
            joint4Slider->setValue(static_cast<int>(value));
            ignoreSliderEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint5SpinChanged(double value) {
        if (!ignoreSpinEvents) {
            ignoreSliderEvents = true;
            joint5Slider->setValue(static_cast<int>(value));
            ignoreSliderEvents = false;
            sendJointCommand();
        }
    }
    
    void onJoint6SpinChanged(double value) {
        if (!ignoreSpinEvents) {
            ignoreSliderEvents = true;
            joint6Slider->setValue(static_cast<int>(value));
            ignoreSliderEvents = false;
            sendJointCommand();
        }
    }
    
    void onVacuumPowerSliderChanged(int value) {
        // 发布真空吸盘功率命令
        std_msgs::Int32 msg;
        msg.data = value;
        vacuumPowerPub.publish(msg);
        
        statusBar()->showMessage(QString("Vacuum power set to: %1%").arg(value));
    }

private:
    // 创建关节控制组
    void createJointControlGroup(QVBoxLayout* parentLayout) {
        QGroupBox* jointGroup = new QGroupBox("Joint Control", this);
        QVBoxLayout* jointLayout = new QVBoxLayout(jointGroup);
        
        // 创建滚动区域以容纳所有关节控制
        QScrollArea* scrollArea = new QScrollArea(this);
        scrollArea->setWidgetResizable(true);
        QWidget* scrollContent = new QWidget(scrollArea);
        QVBoxLayout* scrollLayout = new QVBoxLayout(scrollContent);
        
        // 创建6个关节控制
        createJointControl(scrollLayout, "Joint 1", -180, 180, joint1Slider, joint1Spin);
        createJointControl(scrollLayout, "Joint 2", 0, 50, joint2Slider, joint2Spin);
        createJointControl(scrollLayout, "Joint 3", -90, 90, joint3Slider, joint3Spin);
        createJointControl(scrollLayout, "Joint 4", 0, 180, joint4Slider, joint4Spin);
        createJointControl(scrollLayout, "Joint 5", -90, 90, joint5Slider, joint5Spin);
        createJointControl(scrollLayout, "Joint 6", 5, 15, joint6Slider, joint6Spin);
        
        scrollArea->setWidget(scrollContent);
        jointLayout->addWidget(scrollArea);
        parentLayout->addWidget(jointGroup);
    }
    
    // 创建单个关节控制
    void createJointControl(QVBoxLayout* parentLayout, const QString& name, 
                           int minVal, int maxVal, 
                           QSlider*& slider, QDoubleSpinBox*& spin) {
        QGroupBox* controlGroup = new QGroupBox(name, this);
        QVBoxLayout* controlLayout = new QVBoxLayout(controlGroup);
        
        // 创建滑块
        slider = new QSlider(Qt::Horizontal, this);
        slider->setRange(minVal, maxVal);
        slider->setValue(0);
        controlLayout->addWidget(slider);
        
        // 创建微调框和标签的水平布局
        QHBoxLayout* spinLayout = new QHBoxLayout();
        
        // 创建微调框
        spin = new QDoubleSpinBox(this);
        spin->setRange(minVal, maxVal);
        spin->setValue(0);
        spin->setSingleStep(1.0);
        spinLayout->addWidget(spin);
        
        // 添加单位标签
        QLabel* unitLabel = new QLabel("deg", this);
        spinLayout->addWidget(unitLabel);
        
        controlLayout->addLayout(spinLayout);
        parentLayout->addWidget(controlGroup);
    }
    
    // 创建末端执行器控制组
    void createEndEffectorGroup(QVBoxLayout* parentLayout) {
        QGroupBox* endEffectorGroup = new QGroupBox("End Effector Control", this);
        QVBoxLayout* endEffectorLayout = new QVBoxLayout(endEffectorGroup);
        
        // 创建位置控制网格
        QGridLayout* positionLayout = new QGridLayout();
        
        // X坐标
        positionLayout->addWidget(new QLabel("X Position (cm):", this), 0, 0);
        posXSpin = new QDoubleSpinBox(this);
        posXSpin->setRange(-50, 50);
        posXSpin->setValue(30);
        positionLayout->addWidget(posXSpin, 0, 1);
        
        // Y坐标
        positionLayout->addWidget(new QLabel("Y Position (cm):", this), 1, 0);
        posYSpin = new QDoubleSpinBox(this);
        posYSpin->setRange(-50, 50);
        posYSpin->setValue(0);
        positionLayout->addWidget(posYSpin, 1, 1);
        
        // Z坐标
        positionLayout->addWidget(new QLabel("Z Position (cm):", this), 2, 0);
        posZSpin = new QDoubleSpinBox(this);
        posZSpin->setRange(0, 50);
        posZSpin->setValue(20);
        positionLayout->addWidget(posZSpin, 2, 1);
        
        endEffectorLayout->addLayout(positionLayout);
        
        // 添加移动按钮
        moveToPositionButton = new QPushButton("Move to Position", this);
        endEffectorLayout->addWidget(moveToPositionButton);
        
        // 添加末端执行器状态标签
        endEffectorStatusLabel = new QLabel("End effector position: Waiting for update...", this);
        endEffectorLayout->addWidget(endEffectorStatusLabel);
        
        parentLayout->addWidget(endEffectorGroup);
    }
    
    // 创建真空吸盘控制组
    void createVacuumGroup(QVBoxLayout* parentLayout) {
        QGroupBox* vacuumGroup = new QGroupBox("Vacuum Gripper Control", this);
        QVBoxLayout* vacuumLayout = new QVBoxLayout(vacuumGroup);
        
        // 创建真空吸盘按钮布局
        QHBoxLayout* vacuumButtonLayout = new QHBoxLayout();
        
        // 添加开启按钮
        vacuumOnButton = new QPushButton("ON", this);
        vacuumButtonLayout->addWidget(vacuumOnButton);
        
        // 添加关闭按钮
        vacuumOffButton = new QPushButton("OFF", this);
        vacuumButtonLayout->addWidget(vacuumOffButton);
        
        // 添加按钮布局到真空吸盘布局
        vacuumLayout->addLayout(vacuumButtonLayout);
        
        // 添加吸力控制
        QHBoxLayout* powerLayout = new QHBoxLayout();
        powerLayout->addWidget(new QLabel("Power:", this));
        
        vacuumPowerSlider = new QSlider(Qt::Horizontal, this);
        vacuumPowerSlider->setRange(0, 100);
        vacuumPowerSlider->setValue(50);
        powerLayout->addWidget(vacuumPowerSlider);
        
        vacuumLayout->addLayout(powerLayout);
        
        parentLayout->addWidget(vacuumGroup);
    }
    
    // 连接关节控制信号
    void connectJointControls() {
        // 连接滑块信号
        connect(joint1Slider, &QSlider::valueChanged, this, &SimpleArmGUI::onJoint1SliderChanged);
        connect(joint2Slider, &QSlider::valueChanged, this, &SimpleArmGUI::onJoint2SliderChanged);
        connect(joint3Slider, &QSlider::valueChanged, this, &SimpleArmGUI::onJoint3SliderChanged);
        connect(joint4Slider, &QSlider::valueChanged, this, &SimpleArmGUI::onJoint4SliderChanged);
        connect(joint5Slider, &QSlider::valueChanged, this, &SimpleArmGUI::onJoint5SliderChanged);
        connect(joint6Slider, &QSlider::valueChanged, this, &SimpleArmGUI::onJoint6SliderChanged);
        
        // 连接微调框信号
        connect(joint1Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SimpleArmGUI::onJoint1SpinChanged);
        connect(joint2Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SimpleArmGUI::onJoint2SpinChanged);
        connect(joint3Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SimpleArmGUI::onJoint3SpinChanged);
        connect(joint4Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SimpleArmGUI::onJoint4SpinChanged);
        connect(joint5Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SimpleArmGUI::onJoint5SpinChanged);
        connect(joint6Spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SimpleArmGUI::onJoint6SpinChanged);
        
        // 连接真空吸盘功率滑块
        connect(vacuumPowerSlider, &QSlider::valueChanged, this, &SimpleArmGUI::onVacuumPowerSliderChanged);
    }
    
    // 初始化ROS
    void initializeROS() {
        // 创建发布者
        jointCommandPub = nh_.advertise<sensor_msgs::JointState>("/joint_command", 1);
        viewModePub = nh_.advertise<std_msgs::Int32>("/stereo_camera/view_mode", 1);
        vacuumCommandPub = nh_.advertise<std_msgs::Bool>("/vacuum_command", 1);
        vacuumPowerPub = nh_.advertise<std_msgs::Int32>("/vacuum_power", 1);
        homeCommandPub = nh_.advertise<std_msgs::Bool>("/home_position", 1);
        
        // 创建订阅者
        jointStateSub = nh_.subscribe("/joint_states", 1, &SimpleArmGUI::jointStateCallback, this);
    }
    
    // 发送关节命令
    void sendJointCommand() {
        sensor_msgs::JointState joint_cmd;
        joint_cmd.header.stamp = ros::Time::now();
        
        // 设置关节名称
        joint_cmd.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        
        // 设置关节位置（转换为弧度）
        joint_cmd.position = {
            degToRad(joint1Spin->value()),
            degToRad(joint2Spin->value()),
            degToRad(joint3Spin->value()),
            degToRad(joint4Spin->value()),
            degToRad(joint5Spin->value()),
            degToRad(joint6Spin->value())
        };
        
        // 发布关节命令
        jointCommandPub.publish(joint_cmd);
        
        // 更新当前关节值
        current_joint_values_ = joint_cmd.position;
        
        statusBar()->showMessage("Joint command sent");
    }
    
    // 关节状态回调
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 更新当前关节值
        if (msg->position.size() >= 6) {
            current_joint_values_ = msg->position;
            
            // 更新UI
            updateJointControlWidgets();
        }
    }
    
    // 更新关节控制部件
    void updateJointControlWidgets() {
        if (current_joint_values_.size() < 6) return;
        
        // 暂时忽略信号以避免循环
        ignoreSliderEvents = true;
        ignoreSpinEvents = true;
        
        // 更新滑块和微调框（转换为角度）
        joint1Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[0])));
        joint2Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[1])));
        joint3Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[2])));
        joint4Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[3])));
        joint5Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[4])));
        joint6Slider->setValue(static_cast<int>(radToDeg(current_joint_values_[5])));
        
        joint1Spin->setValue(radToDeg(current_joint_values_[0]));
        joint2Spin->setValue(radToDeg(current_joint_values_[1]));
        joint3Spin->setValue(radToDeg(current_joint_values_[2]));
        joint4Spin->setValue(radToDeg(current_joint_values_[3]));
        joint5Spin->setValue(radToDeg(current_joint_values_[4]));
        joint6Spin->setValue(radToDeg(current_joint_values_[5]));
        
        // 恢复信号处理
        ignoreSliderEvents = false;
        ignoreSpinEvents = false;
    }
    
    // 角度与弧度转换
    double degToRad(double deg) {
        return deg * M_PI / 180.0;
    }
    
    double radToDeg(double rad) {
        return rad * 180.0 / M_PI;
    }

private:
    ros::NodeHandle& nh_;
    
    // ROS发布者
    ros::Publisher viewModePub;
    ros::Publisher jointCommandPub;
    ros::Publisher vacuumCommandPub;
    ros::Publisher vacuumPowerPub;
    ros::Publisher homeCommandPub;
    
    // ROS订阅者
    ros::Subscriber jointStateSub;
    
    // UI元素 - 相机视图
    QLabel* cameraView;
    QPushButton* leftViewButton;
    QPushButton* rightViewButton;
    
    // UI元素 - 关节控制
    QSlider* joint1Slider;
    QSlider* joint2Slider;
    QSlider* joint3Slider;
    QSlider* joint4Slider;
    QSlider* joint5Slider;
    QSlider* joint6Slider;
    
    QDoubleSpinBox* joint1Spin;
    QDoubleSpinBox* joint2Spin;
    QDoubleSpinBox* joint3Spin;
    QDoubleSpinBox* joint4Spin;
    QDoubleSpinBox* joint5Spin;
    QDoubleSpinBox* joint6Spin;
    
    // UI元素 - 末端执行器控制
    QDoubleSpinBox* posXSpin;
    QDoubleSpinBox* posYSpin;
    QDoubleSpinBox* posZSpin;
    QPushButton* moveToPositionButton;
    QLabel* endEffectorStatusLabel;
    
    // UI元素 - 真空吸盘控制
    QPushButton* vacuumOnButton;
    QPushButton* vacuumOffButton;
    QSlider* vacuumPowerSlider;
    
    // UI元素 - 其他
    QPushButton* homeButton;
    
    // 定时器
    QTimer* updateTimer;
    
    // 状态变量
    std::vector<double> current_joint_values_;
    bool ignoreSliderEvents = false;
    bool ignoreSpinEvents = false;
};

// 定义Q_OBJECT宏处理函数
#include "arm_gui_node.moc"

int main(int argc, char** argv)
{
    // 初始化ROS
    ros::init(argc, argv, "arm_gui_node");
    ros::NodeHandle nh;
    
    // 初始化Qt应用
    QApplication app(argc, argv);
    
    // 创建和显示主窗口
    SimpleArmGUI gui(nh);
    gui.show();
    
    // 设置ROS异步处理
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // 运行Qt事件循环
    int result = app.exec();
    
    // 关闭ROS
    ros::shutdown();
    
    return result;
} 