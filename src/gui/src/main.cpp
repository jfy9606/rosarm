#include "main_window.h"
#include <QApplication>
#include <QMessageBox>
#include <QProcessEnvironment>
#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <signal.h>

// 信号处理函数，用于捕获崩溃
void signalHandler(int signal) {
    std::cerr << "GUI捕获到信号: " << signal << std::endl;
    if (signal == SIGSEGV) {
        std::cerr << "捕获到段错误(SIGSEGV)! GUI即将终止" << std::endl;
    } else if (signal == SIGABRT) {
        std::cerr << "捕获到中止信号(SIGABRT)! GUI即将终止" << std::endl;
    }
    exit(1);
}

int main(int argc, char** argv)
{
    // 注册信号处理器以捕获崩溃
    signal(SIGSEGV, signalHandler);
    signal(SIGABRT, signalHandler);
    
    // 设置标准输出和标准错误不缓冲，以便及时看到日志
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    
    std::cout << "GUI启动中..." << std::endl;
    
    // 检查DISPLAY环境变量
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    QString displayEnv = env.value("DISPLAY", "");
    
    if (displayEnv.isEmpty()) {
        std::cerr << "错误: DISPLAY环境变量未设置。X11显示可能配置不正确。" << std::endl;
        std::cerr << "如果在WSL中运行，请确保启用了X11转发并安装了X服务器。" << std::endl;
        std::cerr << "尝试设置 export DISPLAY=:0 或适当的DISPLAY值。" << std::endl;
        
        // 尝试自动设置DISPLAY
        if (env.contains("WSL_DISTRO") || env.contains("WSL_INTEROP")) {
            std::cerr << "检测到WSL环境，尝试设置显示..." << std::endl;
            qputenv("DISPLAY", ":0");
        }
    } else {
        std::cout << "使用DISPLAY=" << displayEnv.toStdString() << std::endl;
    }

    // 设置Qt环境变量以避免一些X11问题
    qputenv("QT_X11_NO_MITSHM", "1");
    qputenv("QT_SCALE_FACTOR", "1");

    try {
        std::cout << "初始化ROS节点..." << std::endl;
        
        // 尝试初始化ROS，使用更长的超时时间
        int init_attempts = 0;
        bool initialized = false;
        
        while (!initialized && init_attempts < 3) {
            try {
                // 使用独立线程模式初始化，以避免阻塞GUI
                ros::init(argc, argv, "arm_gui_node", ros::init_options::NoSigintHandler);
                initialized = true;
            } catch (const std::exception& e) {
                std::cerr << "ROS初始化失败(尝试 " << init_attempts+1 << "/3): " << e.what() << std::endl;
                std::cerr << "等待5秒后重试..." << std::endl;
                sleep(5);
                init_attempts++;
            }
        }
        
        if (!initialized) {
            std::cerr << "无法初始化ROS，请确保ROS环境正确设置" << std::endl;
            return 1;
        }
        
        // 检查ROS master
        if (!ros::master::check()) {
            std::cerr << "无法连接到ROS master。请先运行 'roscore'。" << std::endl;
            std::cerr << "尝试运行: 'roscore &'" << std::endl;
            return 1;
        }
        
        std::cout << "创建ROS节点句柄..." << std::endl;
        ros::NodeHandle nh;
        
        // 检查ROS是否能正常通信
        try {
            std::cout << "测试ROS通信..." << std::endl;
            ros::Time::init();
            ros::spinOnce();
            std::cout << "ROS通信测试成功" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "ROS通信测试失败: " << e.what() << std::endl;
            std::cerr << "这可能是由于网络配置问题导致的" << std::endl;
            std::cerr << "请检查ROS_IP和ROS_MASTER_URI环境变量" << std::endl;
            std::cerr << "当前ROS_MASTER_URI=" << (getenv("ROS_MASTER_URI") ? getenv("ROS_MASTER_URI") : "未设置") << std::endl;
            std::cerr << "当前ROS_IP=" << (getenv("ROS_IP") ? getenv("ROS_IP") : "未设置") << std::endl;
            std::cerr << "当前ROS_HOSTNAME=" << (getenv("ROS_HOSTNAME") ? getenv("ROS_HOSTNAME") : "未设置") << std::endl;
            return 1;
        }
        
        std::cout << "初始化Qt应用..." << std::endl;
        QApplication app(argc, argv);
        
        std::cout << "创建主窗口..." << std::endl;
        MainWindow window(nh);
        window.show();
        std::cout << "GUI启动成功，开始事件循环" << std::endl;
        
        // 启动ROS事件处理线程
        ros::AsyncSpinner spinner(1);
        spinner.start();
        
        // 启动Qt事件循环
        return app.exec();
    } catch (const std::exception& e) {
        std::cerr << "GUI初始化异常: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "GUI初始化时出现未知异常!" << std::endl;
        return 1;
    }
} 