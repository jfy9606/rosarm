#include "arm_gui/arm_control_gui.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // 初始化ROS
    ros::init(argc, argv, "arm_gui_node");
    ros::NodeHandle nh;
    
    // 初始化Qt应用
    QApplication app(argc, argv);
    
    // 创建主窗口
    ArmControlGUI gui(nh);
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