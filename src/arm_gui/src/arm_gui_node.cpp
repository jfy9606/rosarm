#include "arm_gui/arm_control_gui.h"
#include <QApplication>
#include <QTimer>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // 初始化ROS
    ros::init(argc, argv, "arm_gui_node");
    ros::NodeHandle nh;
    
    // 初始化Qt应用
    QApplication app(argc, argv);
    
    // 设置应用样式
    app.setStyle("Fusion");
    
    // 创建和显示主窗口
    ArmControlGUI gui(nh);
    gui.show();
    
    // 创建定时器用于处理ROS消息
    QTimer rosTimer;
    QObject::connect(&rosTimer, &QTimer::timeout, []() {
        if (ros::ok()) {
            ros::spinOnce();
        } else {
            QApplication::quit();
        }
    });
    rosTimer.start(10); // 100Hz
    
    // 启动Qt事件循环
    return app.exec();
} 