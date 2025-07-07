#include <ros/ros.h>
#include <servo_wrist/SerControl.h>  // Replace with the actual message type

#include "SCServo.h"

SMS_STS sm_st;

//设置波特率
int baud_rate = 1000000;

//定义了一个回调函数msgCallback，当接收到指定话题的消息时，这个函数会被调用
void msgCallback(const servo_wrist::SerControl::ConstPtr& msg) {
    // Process the received message and call WritePosEx
    int servo_id = msg->servo_id;
    int target_position = msg->target_position;
    int velocity = msg->velocity;
    int acceleration = msg->acceleration;

    sm_st.WritePosEx(servo_id, target_position, velocity, acceleration);

    // Optionally, print the received values
    std::cout << "Received msg - Servo ID: " << servo_id
              << ", Target Position: " << target_position
              << ", Velocity: " << velocity
              << ", Acceleration: " << acceleration << std::endl;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "argc error!" << std::endl;
        return 0;
    }
    std::cout << "serial:" << argv[1] << std::endl;

    if (!sm_st.begin(baud_rate, argv[1])) {
        std::cout << "Failed to init sms/sts motor!" << std::endl;
        return 0;
    }

    // Initialize ROS
    ros::init(argc, argv, "servo_control_node");
    ros::NodeHandle nh;

    // Subscribe to the topic that will receive the servo control message
    ros::Subscriber sub = nh.subscribe("servo_control_topic", 10, msgCallback);

    // ROS main loop
    ros::spin();

    // Cleanup and end
    sm_st.end();
    return 1;
}
