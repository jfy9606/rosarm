#include "main_window.h"
#include <QApplication>
#include <QMessageBox>
#include <QProcessEnvironment>
#include <ros/ros.h>
#include <cstdlib>
#include <iostream>

int main(int argc, char** argv)
{
    // 检查DISPLAY环境变量
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    QString displayEnv = env.value("DISPLAY", "");
    
    if (displayEnv.isEmpty()) {
        std::cerr << "Error: DISPLAY environment variable not set. X11 forwarding might not be properly configured." << std::endl;
        std::cerr << "Please ensure X11 forwarding is enabled if running in WSL or over SSH." << std::endl;
        std::cerr << "For WSL, consider using 'export DISPLAY=:0' and installing an X server like VcXsrv." << std::endl;
        
        // 尝试自动设置DISPLAY
        if (env.contains("WSL_DISTRO")) {
            std::cerr << "WSL detected, attempting to set DISPLAY=:0" << std::endl;
            qputenv("DISPLAY", ":0");
        }
    }

    try {
        // 初始化ROS节点
        ros::init(argc, argv, "arm_gui_node");
        ros::NodeHandle nh;
        
        // 初始化Qt应用
        QApplication app(argc, argv);
        
        // 确保ROS主节点可以访问
        if (!ros::master::check()) {
            QMessageBox::critical(nullptr, "ROS Connection Error", 
                                "Cannot connect to ROS master. Is roscore running?");
            return 1;
        }
        
        // 创建主窗口
        MainWindow window(nh);
        window.show();
        
        // 启动Qt事件循环
        return app.exec();
    } catch (const std::exception& e) {
        std::cerr << "Error initializing GUI: " << e.what() << std::endl;
        return 1;
    }
} 