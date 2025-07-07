#include "gui/main_window.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "arm_gui_node");
    ros::NodeHandle nh;
    
    // 初始化Qt应用
    QApplication app(argc, argv);
    
    // 创建主窗口
    MainWindow window(nh);
    window.show();
    
    // 启动Qt事件循环
    return app.exec();
} 