#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "position_sender");
    ros::NodeHandle nh;
    
    // 创建发布者，发送3D位置命令
    ros::Publisher position_pub = nh.advertise<geometry_msgs::Point>("arm_target_position", 10);
    
    // 等待连接建立
    ros::Duration(1.0).sleep();
    
    while (ros::ok()) {
        // 创建3D点消息
        geometry_msgs::Point point;
        
        // 提示用户输入目标位置
        std::cout << "\n请输入机械臂末端目标位置 (x y z)，单位mm，输入'q'退出: ";
        std::string input;
        std::getline(std::cin, input);
        
        // 检查是否退出
        if (input == "q" || input == "Q") {
            break;
        }
        
        // 解析输入的坐标
        std::istringstream iss(input);
        if (!(iss >> point.x >> point.y >> point.z)) {
            std::cout << "输入格式错误！请按照'x y z'的格式输入三个数值" << std::endl;
            continue;
        }
        
        std::cout << "发送目标位置: x=" << point.x << ", y=" << point.y << ", z=" << point.z << std::endl;
        
        // 发送位置命令
        position_pub.publish(point);
        
        // 处理回调
        ros::spinOnce();
        
        // 给机械臂一些时间移动到位置
        ros::Duration(2.0).sleep();
    }
    
    return 0;
} 