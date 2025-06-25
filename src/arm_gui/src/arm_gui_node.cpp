#include <ros/ros.h>
#include <QApplication>
#include <QTimer>
#include "arm_gui/arm_control_gui.h"
#include <iostream>
#include <signal.h>

// 信号处理函数
void sigintHandler(int sig)
{
    ROS_INFO("接收到中断信号，退出程序...");
    QApplication::quit();
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
        QApplication app(argc, argv);
        
        std::cout << "Qt应用初始化完成" << std::endl;
        
        // 创建GUI实例
        std::cout << "创建GUI实例..." << std::endl;
        ArmControlGUI* gui = new ArmControlGUI(nh);
        gui->show();
        
        std::cout << "GUI显示完成" << std::endl;
        
        // 设置定时器用于处理ROS消息
        QTimer rosTimer;
        QObject::connect(&rosTimer, &QTimer::timeout, []() {
            if (ros::ok()) {
                ros::spinOnce();
            } else {
                QApplication::quit();
            }
        });
        rosTimer.start(10); // 10ms
        
        // 输出调试信息
        ROS_INFO("机械臂GUI已启动");
        std::cout << "进入Qt事件循环..." << std::endl;
        
        // 进入Qt事件循环
        return app.exec();
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