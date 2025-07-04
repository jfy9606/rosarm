#include "arm_gui/enhanced_arm_gui.h"
#include <QApplication>
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
    EnhancedArmGUI gui(nh);
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