#include <ros/ros.h>
#include <QApplication>
#include <QTimer>
#include <QSplashScreen>
#include <QPixmap>
#include <QPainter>
#include "arm_gui/arm_control_gui.h"
#include <iostream>
#include <signal.h>

// 全局变量，用于保持GUI引用
QApplication* g_app = nullptr;
ArmControlGUI* g_gui = nullptr;

// 信号处理函数
void sigintHandler(int sig)
{
    ROS_INFO("接收到中断信号，正常退出程序...");
    if (g_app) {
        g_app->quit();
    }
    ros::shutdown();
}

int main(int argc, char **argv)
{
    try {
        std::cout << "启动机械臂GUI程序..." << std::endl;
        
        // 设置信号处理
        signal(SIGINT, sigintHandler);
        
        // 初始化ROS节点
        ros::init(argc, argv, "arm_gui_node", ros::init_options::NoSigintHandler);
        ros::NodeHandle nh;
        
        std::cout << "ROS节点初始化完成" << std::endl;
        
        // 初始化Qt应用
        g_app = new QApplication(argc, argv);
        g_app->setQuitOnLastWindowClosed(false);  // 防止最后一个窗口关闭时退出应用
        
        std::cout << "Qt应用初始化完成" << std::endl;
        
        // 显示启动画面
        QPixmap splashPixmap(400, 300);
        splashPixmap.fill(Qt::darkGray);
        QPainter painter(&splashPixmap);
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 20, QFont::Bold));
        painter.drawText(splashPixmap.rect(), Qt::AlignCenter, "机械臂控制系统\n正在加载...");
        painter.end();
        
        QSplashScreen splash(splashPixmap);
        splash.show();
        g_app->processEvents();
        
        // 创建GUI实例
        std::cout << "创建GUI实例..." << std::endl;
        g_gui = new ArmControlGUI(nh);
        g_gui->show();
        
        // 关闭启动画面
        splash.finish(g_gui);
        
        std::cout << "GUI显示完成" << std::endl;
        
        // 设置定时器用于处理ROS消息
        QTimer rosTimer;
        QObject::connect(&rosTimer, &QTimer::timeout, []() {
            if (ros::ok()) {
                ros::spinOnce();
            } else {
                if (g_app) g_app->quit();
            }
        });
        rosTimer.start(10); // 10ms
        
        // 设置定时器确保GUI保持可见
        QTimer visibilityTimer;
        QObject::connect(&visibilityTimer, &QTimer::timeout, []() {
            if (g_gui && !g_gui->isVisible()) {
                g_gui->show();
                g_gui->raise();
                g_gui->activateWindow();
                std::cout << "GUI窗口重新显示" << std::endl;
            }
        });
        visibilityTimer.start(1000); // 每秒检查一次
        
        // 输出调试信息
        ROS_INFO("机械臂GUI已启动");
        std::cout << "进入Qt事件循环..." << std::endl;
        
        // 进入Qt事件循环
        int result = g_app->exec();
        
        // 清理资源
        delete g_gui;
        delete g_app;
        
        return result;
    }
    catch (std::exception& e) {
        std::cerr << "发生异常: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "发生未知异常" << std::endl;
        return 1;
    }
} 